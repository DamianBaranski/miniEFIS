#include "wifi_debug.h"
#include "app_settings.h"
#include "can_bus.h"
#include "crash_report.h"
#include "screen_manager.h"
#include "update_screen.h"
#include "magnetometer.h"
#include "web_log.h"

#include "board_pins.h"
#include "lvgl_port.h"

#include "esp_app_desc.h"
#include "esp_core_dump.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_image_format.h"
#include "esp_ipc_isr.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_private/system_internal.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "wifi_debug";

/* How long to try the network before opening our own access point. */
static const int64_t JOIN_TIMEOUT_US = 20 * 1000 * 1000;

/* Between attempts to join: soon while starting out, rarely once the
 * access point is up - each attempt scans, which takes the access point
 * off its channel for a moment. */
static const int64_t RETRY_US = 5 * 1000 * 1000;
static const int64_t FALLBACK_RETRY_US = 60 * 1000 * 1000;

/*
 * Below the IMU task (3), which shares core 0 with the Wi-Fi driver:
 * serving a page or writing an update must not hold up the sampling.
 */
static const int TASK_PRIORITY = 2;
static const int TASK_STACK_BYTES = 4096;
static const int SERVER_STACK_BYTES = 6144;

static const char *NVS_NAMESPACE = "wifi_dbg";

/* Uploads and log reads go through this, in PSRAM. One flash sector. */
static const size_t TRANSFER_BYTES = 4096;

/*
 * An update is written a 64 KB flash block at a time - one erase
 * command each - with a pause after each block in which the tasks held
 * up by it catch up and core 0's idle task gets to run.
 */
static const size_t WRITE_BLOCK_BYTES = 64 * 1024;
static const uint32_t BLOCK_PAUSE_MS = 30;

static TaskHandle_t s_task = NULL;

/* Set from other tasks, acted on by ours. */
static volatile bool s_wanted = false;
static volatile bool s_new_credentials = false;
static volatile bool s_sta_started = false;
static volatile bool s_attempting = false;

/* Our task's own. */
static bool s_initialised = false;
static bool s_running = false;
static bool s_fallback = false;         /* access point up */
static int64_t s_join_started_us = 0;
static int64_t s_last_attempt_us = 0;
static httpd_handle_t s_server = NULL;
static esp_netif_t *s_sta_netif = NULL;

static char s_ssid[33] = "";
static char s_password[65] = "";

static wifi_debug_status_t s_status;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- credentials ---------------------------------------------------------- */

/* The web page's, if it has saved any, otherwise menuconfig's. */
static void load_credentials(void)
{
    strlcpy(s_ssid, CONFIG_WIFI_DEBUG_SSID, sizeof(s_ssid));
    strlcpy(s_password, CONFIG_WIFI_DEBUG_PASSWORD, sizeof(s_password));

    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return;

    char ssid[sizeof(s_ssid)];
    char password[sizeof(s_password)];
    size_t ssid_length = sizeof(ssid);
    size_t password_length = sizeof(password);

    if(nvs_get_str(handle, "ssid", ssid, &ssid_length) == ESP_OK &&
       nvs_get_str(handle, "pass", password, &password_length) == ESP_OK)
    {
        strlcpy(s_ssid, ssid, sizeof(s_ssid));
        strlcpy(s_password, password, sizeof(s_password));
    }

    nvs_close(handle);
}

static esp_err_t save_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if(err != ESP_OK)
        return err;

    err = nvs_set_str(handle, "ssid", ssid);

    if(err == ESP_OK)
        err = nvs_set_str(handle, "pass", password);

    if(err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);

    return err;
}

/* Back to menuconfig's network, or none: the access point. */
static void forget_credentials(void)
{
    nvs_handle_t handle;

    if(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return;

    const esp_err_t err = nvs_erase_all(handle);

    if(err == ESP_OK)
        nvs_commit(handle);

    nvs_close(handle);

    if(err == ESP_OK)
        ESP_LOGI(TAG, "saved network forgotten");
}

/* ---- status --------------------------------------------------------------- */

static void status_update(void (*change)(wifi_debug_status_t *status))
{
    portENTER_CRITICAL(&s_status_lock);
    change(&s_status);
    portEXIT_CRITICAL(&s_status_lock);
}

void wifi_debug_get_status(wifi_debug_status_t *status)
{
    portENTER_CRITICAL(&s_status_lock);
    *status = s_status;
    portEXIT_CRITICAL(&s_status_lock);
}

/* ---- the running image ---------------------------------------------------- */

/*
 * A freshly installed image boots on trial: unless it says it is good
 * before the next restart, the bootloader goes back to the old one.
 */
static void confirm_running_image(void)
{
    esp_ota_img_states_t state;

    if(esp_ota_get_state_partition(esp_ota_get_running_partition(), &state)
           != ESP_OK || state != ESP_OTA_IMG_PENDING_VERIFY)
        return;

    if(esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
        ESP_LOGI(TAG, "new firmware confirmed, no rollback");
}

/*
 * esp_restart(), without its race on this chip.
 *
 * ESP-IDF 5.2's esp_restart_noos() on the S3 turns the cache off - one
 * cache, shared by both cores - and only then stalls the other core. If
 * that core is running code or touching data in flash or PSRAM in
 * between, it takes a cache error, and the panic handler faults in turn
 * since everything it reads comes back 0xbad00bad. Most cores sit idle
 * in IRAM often enough not to notice; here LVGL keeps core 1 drawing
 * nearly all the time, and two restarts in six crashed this way.
 *
 * So core 1 is parked first, busy-waiting in a high priority interrupt
 * that runs from IRAM, and the reset done from here. esp_restart()'s
 * shutdown handlers are skipped, as a panic skips them: the reset that
 * follows puts the radio and every peripheral back to their defaults.
 */
static void restart(void)
{
    /* What the skipped shutdown handler would do: leave the network
     * cleanly rather than vanish from it. */
    esp_wifi_stop();

    vTaskSuspendAll();
    esp_ipc_isr_stall_other_cpu();
    esp_restart_noos();
}

/* Long enough for the reply to reach the browser; after a failed
 * update, also for the screen saying so to be read. */
static const uint64_t RESTART_DELAY_US = 1000 * 1000;
static const uint64_t UPDATE_FAILED_RESTART_US = 4000 * 1000;

static void restart_soon(uint64_t delay_us)
{
    static esp_timer_handle_t timer = NULL;

    if(timer == NULL)
    {
        const esp_timer_create_args_t args = {
            .callback = [](void *) { restart(); },
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "restart",
            .skip_unhandled_events = true,
        };

        if(esp_timer_create(&args, &timer) != ESP_OK)
            restart();
    }

    esp_timer_stop(timer);
    esp_timer_start_once(timer, delay_us);
}

/* ---- web page ------------------------------------------------------------- */

static const char PAGE[] = R"html(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MiniEFIS</title>
<style>
body{font:14px system-ui,sans-serif;margin:0;background:#10161c;color:#dde}
header{padding:10px 16px;background:#1a2028;display:flex;gap:16px;flex-wrap:wrap}
header b{color:#ffd400}
section{padding:8px 16px}
h2{font-size:14px;color:#ffd400;margin:12px 0 6px}
#log{background:#000;color:#cfd;font:12px monospace;height:40vh;overflow:auto;
white-space:pre-wrap;padding:8px;margin:0}
#charts{display:grid;grid-template-columns:repeat(auto-fill,minmax(360px,1fr));gap:10px}
.chart{background:#000;border:1px solid #2a3440;padding:4px 6px}
.chart .t{color:#aab;font-size:12px;display:flex;justify-content:space-between}
.chart canvas{width:100%;height:150px;display:block}
.lg span{margin-left:10px;font:12px monospace}
#compass{display:flex;gap:10px;flex-wrap:wrap}
#compass .chart{flex:1 1 380px;max-width:520px}
#compass canvas{height:auto;aspect-ratio:1}
.cs{font:12px monospace;color:#aab;margin-top:4px;min-height:3.5em;white-space:pre-wrap}
.ok{color:#4f4}.bad{color:#fa4}
#can{border-collapse:collapse;font:12px monospace}
#can td,#can th{padding:2px 8px;border-bottom:1px solid #223;text-align:left;vertical-align:top}
#can th{color:#aab;font-weight:normal}
#can tr.stale td{color:#667}
.W{color:#ffd400}.E{color:#ff5050}
button,input{font:inherit;padding:5px 10px}
progress{width:240px}
</style></head><body>
<header><b>MiniEFIS</b><span id="info">...</span></header>
<section>
<h2>Charts <select id="window"><option value="30">30 s</option>
<option value="60" selected>1 min</option><option value="300">5 min</option></select>
<button id="pause" onclick="paused=!paused;this.textContent=paused?'resume':'pause'">pause</button>
<span id="telstate" style="font-weight:normal;color:#889"></span></h2>
<div id="charts"></div>
</section>
<section>
<h2>Compass</h2>
<div id="compass">
<div class="chart"><div class="t"><span>Swing: the field in the horizontal plane, uT</span></div>
<canvas id="swing"></canvas><div class="cs" id="swingstate"></div></div>
<div class="chart"><div class="t"><span>Levelled field, live, uT (forward up)</span></div>
<canvas id="level"></canvas><div class="cs" id="levelstate"></div></div>
</div>
</section>
<section>
<h2>CAN bus <span id="canstate" style="font-weight:normal;color:#889">...</span></h2>
<table id="can"></table>
<div class="cs" id="cannodes"></div>
</section>
<section>
<h2>Log <label style="font-weight:normal;color:#aab">
<input type="checkbox" id="follow" checked> follow</label>
<button onclick="logEl.textContent=''">clear</button>
<a href="/log?from=0" download="log.txt" style="color:#8bf">download</a></h2>
<pre id="log"></pre>
</section>
<section>
<h2>Firmware update</h2>
<input type="file" id="file" accept=".bin">
<button onclick="upload()">Install</button>
<progress id="progress" max="1" value="0"></progress> <span id="result"></span>
<p style="color:#889">build/miniefis.bin. The unit restarts into it;
if it does not come up, the next restart goes back to this one.</p>
</section>
<section>
<h2>Device</h2>
<button onclick="restart()">Restart</button>
<a href="/coredump" style="color:#8bf">core dump of the last crash</a>
<p style="color:#889">Sensor plots: open Diagnostics or Compass on the unit, then
<code>python3 tools/sensor_plot.py --url http://<span id="host"></span></code></p>
<h2>Network to join</h2>
<form method="post" action="/wifi">
<input name="ssid" placeholder="network"> <input name="password"
type="password" placeholder="password"> <button>Save and join</button>
</form>
</section>
<script>
const logEl=document.getElementById('log');
let from=0;
function line(t){
  t=t.replace(/\x1b\[[0-9;]*m/g,'');
  const s=document.createElement('span');
  if(/^W \(/.test(t))s.className='W';
  if(/^E \(/.test(t))s.className='E';
  s.textContent=t;logEl.appendChild(s);
}
let partial='';
async function poll(){
  try{
    const r=await fetch('/log?from='+from);
    const next=parseInt(r.headers.get('X-Log-Next'));
    const text=partial+await r.text();
    const lines=text.split('\n');partial=lines.pop();
    for(const l of lines)line(l+'\n');
    while(logEl.childNodes.length>3000)logEl.removeChild(logEl.firstChild);
    if(document.getElementById('follow').checked)logEl.scrollTop=logEl.scrollHeight;
    const more=next-from>=4000;from=next;
    setTimeout(poll,more?0:1000);
  }catch(e){setTimeout(poll,3000);}
}
async function status(){
  try{
    const s=await(await fetch('/status')).json();
    if(s.uptime_s<lastUptime){from=0;partial='';line('--- restarted ---\n');}
    lastUptime=s.uptime_s;
    document.getElementById('info').textContent=
      s.version+' ('+s.built+'), '+s.partition+', up '+s.uptime_s+' s, '+
      'heap '+s.heap_internal+'/'+s.heap_psram+' B, '+s.network+
      (s.rssi?' '+s.rssi+' dBm':'')+' - '+s.reset;
  }catch(e){document.getElementById('info').textContent='not responding';}
}
let lastUptime=0;
function upload(){
  const f=document.getElementById('file').files[0];if(!f)return;
  const x=new XMLHttpRequest(),res=document.getElementById('result');
  x.upload.onprogress=e=>document.getElementById('progress').value=e.loaded/e.total;
  x.onload=()=>res.textContent=x.responseText;
  x.onerror=()=>res.textContent='upload failed';
  res.textContent='uploading...';
  x.open('POST','/update');x.send(f);
}
function restart(){if(confirm('Restart?'))fetch('/restart',{method:'POST'});}
document.getElementById('host').textContent=location.host;

/* ---- charts: the $TEL / $SYS telemetry, drawn on canvases ---- */
const CHARTS=[
 ['Attitude, deg','TEL',[['roll_deg','roll','#4af'],['pitch_deg','pitch','#fa4']]],
 ['Heading, deg','TEL',[['hdg_deg','heading','#fff']],0,360],
 ['Rates, bias removed, deg/s','TEL',[['p_dps','p','#f66'],['q_dps','q','#6f6'],['r_dps','r','#6af']]],
 ['Acceleration, body, g','TEL',[['ax_g','x','#f66'],['ay_g','y','#6f6'],['az_g','z','#6af']]],
 ['Accelerometer weight (1 = full)','TEL',[['accel_trust','trust','#6f6']],0,1],
 ['Filter uncertainty, deg','TEL',[['tilt_sd_deg','tilt sd','#fa4'],['hdg_sd_deg','heading sd','#f6f']]],
 ['Turn rate deg/s, slip deg','TEL',[['turn_dps','turn','#ff6'],['slip_deg','slip','#6ff']]],
 ['Gyro bias, deg/s','TEL',[['bias_x_dps','x','#f66'],['bias_y_dps','y','#6f6'],['bias_z_dps','z','#6af']]],
 ['Altitude, m','TEL',[['alt_m','altitude','#fff']]],
 ['Vertical speed, m/s','TEL',[['vs_ms','v/s','#6f6']]],
 ['Magnetic field, body, uT','TEL',[['mf_ut','fwd','#f66'],['mr_ut','right','#6f6'],['md_ut','down','#6af'],['b_ut','|B|','#fff']]],
 ['Pressure, Pa','TEL',[['p_pa','pressure','#ccc']]],
 ['Field strength uT, dip deg','TEL',[['b_ut','|B|','#fff'],['dip_deg','dip','#f6f']]],
 ['Heading: filter, and each magnetometer sample, deg','TEL',[['hdg_deg','filter','#fff'],['maghdg_deg','sample','#fa4']],0,360],
 ['Display: fps, draw ms','SYS',[['fps','fps','#6f6'],['draw_ms','draw','#fa4'],['frame_ms','frame','#f66']]],
 ['IMU interval, ms','SYS',[['dt_min_ms','min','#6af'],['dt_max_ms','max','#f66']]],
 ['Attitude jitter, peak to peak over 1 s, deg','SYS',[['roll_pp_deg','roll','#4af'],['pitch_pp_deg','pitch','#fa4']]],
 ['Internal heap, KB','SYS',[['heap_kb','free','#6f6'],['heap_min_kb','least','#f66']]]
];
const tel={cols:{},rows:{TEL:[],SYS:[]}};
let tpos=0,tpartial='',paused=false;
const charts=CHARTS.map(([title,src,series,min,max])=>{
  const d=document.createElement('div');d.className='chart';
  d.innerHTML='<div class="t"><span>'+title+'</span><span class="lg"></span></div><canvas></canvas>';
  document.getElementById('charts').appendChild(d);
  return {src,series,min,max,canvas:d.querySelector('canvas'),legend:d.querySelector('.lg')};
});
function telLine(l){
  const star=l.lastIndexOf('*');if(l[0]!=='$'||star<0)return;
  const f=l.slice(1,star).split(',');const name=f[0];
  if(name.endsWith('H')){tel.cols[name.slice(0,-1)]=f.slice(2);return;}
  const cols=tel.cols[name];if(!cols||cols.length!==f.length-1)return;
  const row={};cols.forEach((c,i)=>row[c]=parseFloat(f[i+1]));
  if(name==='TEL')level(row);
  const rows=tel.rows[name];if(!rows)return;
  if(rows.length&&row.t_ms<rows[rows.length-1].t_ms)rows.length=0; /* restarted */
  rows.push(row);
  const keep=row.t_ms-310000;while(rows.length&&rows[0].t_ms<keep)rows.shift();
}
/* The calibrated field turned into the level frame with the attitude,
   as the filter does it: forward and right along the horizon, down. */
function level(r){
  const R=Math.PI/180,sr=Math.sin(r.roll_deg*R),cr=Math.cos(r.roll_deg*R);
  const sp=Math.sin(r.pitch_deg*R),cp=Math.cos(r.pitch_deg*R);
  const x=r.mf_ut,y=r.mr_ut,z=r.md_ut;
  r.lf_ut=x*cp+(y*sr+z*cr)*sp;r.lr_ut=y*cr-z*sr;r.ld_ut=-x*sp+(y*sr+z*cr)*cp;
  const h=Math.hypot(r.lf_ut,r.lr_ut);
  r.dip_deg=Math.atan2(r.ld_ut,h)/R;
  let hd=Math.atan2(-r.lr_ut,r.lf_ut)/R;r.maghdg_deg=hd<0?hd+360:hd;
}
/* A square plot of points: scale to fit, equal axes, origin cross. */
function plane(cv,pts,extra){
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth*dpr,h=cv.clientWidth*dpr;
  if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h;}
  const g=cv.getContext('2d');g.clearRect(0,0,w,h);
  let reach=extra.reach||1;
  for(const [a,b] of pts)reach=Math.max(reach,Math.abs(a),Math.abs(b));
  reach*=1.15;const s=w/2/reach,X=a=>w/2+a*s,Y=b=>h/2-b*s;
  g.strokeStyle='#334';g.lineWidth=dpr;
  g.beginPath();g.moveTo(0,h/2);g.lineTo(w,h/2);g.moveTo(w/2,0);g.lineTo(w/2,h);g.stroke();
  g.fillStyle='#667';g.font=(10*dpr)+'px monospace';
  g.fillText('+'+reach.toFixed(0),w-40*dpr,h/2-4*dpr);g.fillText('+'+reach.toFixed(0),w/2+4*dpr,12*dpr);
  g.fillStyle='#8af';
  for(const [a,b] of pts)g.fillRect(X(a)-1.5*dpr,Y(b)-1.5*dpr,3*dpr,3*dpr);
  return {g,X,Y,s,dpr};
}
function ellipse(p,cx,cy,ax,ay,colour){
  p.g.strokeStyle=colour;p.g.lineWidth=2*p.dpr;p.g.beginPath();
  p.g.ellipse(p.X(cx),p.Y(cy),ax*p.s,ay*p.s,0,0,2*Math.PI);p.g.stroke();
}
function cross(p,x,y,colour){
  p.g.strokeStyle=colour;p.g.lineWidth=2*p.dpr;const d=6*p.dpr;p.g.beginPath();
  p.g.moveTo(p.X(x)-d,p.Y(y));p.g.lineTo(p.X(x)+d,p.Y(y));
  p.g.moveTo(p.X(x),p.Y(y)-d);p.g.lineTo(p.X(x),p.Y(y)+d);p.g.stroke();
}
function mark(ok,text){return '<span class="'+(ok?'ok':'bad')+'">'+text+'</span>';}
let compassInfo=null;
function drawLevel(){
  const rows=tel.rows.TEL;if(!rows.length)return;
  const span=parseFloat(document.getElementById('window').value)*1000;
  const t0=rows[rows.length-1].t_ms-span;
  const pts=rows.filter(r=>r.t_ms>=t0&&isFinite(r.lf_ut)).map(r=>[r.lr_ut,r.lf_ut]);
  const dip=compassInfo?compassInfo.limits.dip:67, expect=50*Math.cos(dip*Math.PI/180);
  const p=plane(document.getElementById('level'),pts,{reach:expect});
  ellipse(p,0,0,expect,expect,'#4a4');
  if(pts.length){
    let cx=0,cy=0;for(const [a,b] of pts){cx+=a;cy+=b;}cx/=pts.length;cy/=pts.length;
    cross(p,0,0,'#556');cross(p,cx,cy,'#fa4');
    const last=rows[rows.length-1];
    document.getElementById('levelstate').innerHTML=
      'Turn the unit round: the dots should follow the green circle, centred on the cross.\n'+
      'centre of the dots '+cx.toFixed(1)+', '+cy.toFixed(1)+' uT (orange) - off centre is heading error\n'+
      'now: |H| '+Math.hypot(last.lf_ut,last.lr_ut).toFixed(1)+' uT, dip '+last.dip_deg.toFixed(1)+
      ' deg (expected '+dip+'), heading from this sample '+last.maghdg_deg.toFixed(0);
  }else document.getElementById('levelstate').textContent='no calibrated magnetometer readings';
}
function drawSwing(c){
  const st=document.getElementById('swingstate');
  if(!c.present){st.textContent='No magnetometer fitted.';plane(document.getElementById('swing'),[],{});return;}
  if(!c.calibrating){
    plane(document.getElementById('swing'),[],{});
    const s=c.saved;
    st.innerHTML='No swing in progress - start one on the unit: Settings, Calibrate.\n'+
      (s.valid?'Saved: field '+s.field.toFixed(1)+' uT, offset '+s.offset.map(v=>v.toFixed(1)).join(' ')+
       ' uT, scale '+s.scale.map(v=>v.toFixed(3)).join(' '):'Not calibrated.');
    return;}
  const pr=c.progress,L=c.limits,pts=c.points||[];
  const p=plane(document.getElementById('swing'),pts,{});
  if(c.fit){ellipse(p,c.fit.cu,c.fit.cv,c.fit.au,c.fit.av,'#fd0');cross(p,c.fit.cu,c.fit.cv,'#fd0');}
  cross(p,0,0,'#556');
  st.innerHTML=(pr.ready?mark(true,'READY - save it on the unit'):'turning...')+'\n'+
    mark(pr.coverage>=L.coverage,'circle '+(pr.coverage*100).toFixed(0)+'% (needs '+(L.coverage*100).toFixed(0)+')')+'  '+
    mark(pr.tilt<=L.tilt,'tilt '+pr.tilt.toFixed(1)+' deg (max '+L.tilt+')')+'  '+
    mark(pr.fit_error<=L.fit_error,'fit error '+(pr.fit_error*100).toFixed(1)+'% (max '+(L.fit_error*100).toFixed(0)+')')+'\n'+
    'samples '+pr.samples+', horizontal field '+pr.radius.toFixed(1)+' uT'+
    (c.fit?', hard iron in the plane '+c.fit.cu.toFixed(1)+', '+c.fit.cv.toFixed(1)+' uT (yellow cross)':'');
}
async function pollCompass(){
  try{
    compassInfo=await(await fetch('/compass')).json();
    if(!paused)drawSwing(compassInfo);
    setTimeout(pollCompass,compassInfo.calibrating?700:3000);
  }catch(e){setTimeout(pollCompass,3000);}
}
pollCompass();
function draw(c){
  const cv=c.canvas,dpr=window.devicePixelRatio||1;
  const w=cv.clientWidth*dpr,h=cv.clientHeight*dpr;
  if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h;}
  const g=cv.getContext('2d');g.clearRect(0,0,w,h);
  const rows=tel.rows[c.src];if(!rows.length)return;
  const span=parseFloat(document.getElementById('window').value)*1000;
  const t1=rows[rows.length-1].t_ms,t0=t1-span;
  const vis=rows.filter(r=>r.t_ms>=t0);
  let lo=c.min,hi=c.max;
  if(lo===undefined){lo=Infinity;hi=-Infinity;
    for(const r of vis)for(const [k] of c.series){const v=r[k];if(isFinite(v)){if(v<lo)lo=v;if(v>hi)hi=v;}}
    if(!isFinite(lo)){lo=0;hi=1;}
    const pad=(hi-lo)*0.1||Math.max(Math.abs(hi)*0.05,0.5);lo-=pad;hi+=pad;}
  const X=t=>(t-t0)/span*(w-50*dpr)+46*dpr, Y=v=>h-4*dpr-(v-lo)/(hi-lo)*(h-8*dpr);
  g.font=(10*dpr)+'px monospace';g.fillStyle='#667';g.strokeStyle='#223';
  for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4,y=Y(v);
    g.beginPath();g.moveTo(46*dpr,y);g.lineTo(w,y);g.stroke();
    g.fillText(Math.abs(hi-lo)<10?v.toFixed(2):v.toFixed(0),2,
               Math.min(Math.max(y+3*dpr,10*dpr),h-2*dpr));}
  g.lineWidth=1.5*dpr;
  let legend='';
  for(const [k,label,colour] of c.series){
    g.strokeStyle=colour;g.beginPath();let up=false;
    for(const r of vis){const v=r[k];
      if(!isFinite(v)){up=false;continue;}
      if(up)g.lineTo(X(r.t_ms),Y(v));else{g.moveTo(X(r.t_ms),Y(v));up=true;}}
    g.stroke();
    const last=vis.length?vis[vis.length-1][k]:NaN;
    legend+='<span style="color:'+colour+'">'+label+' '+(isFinite(last)?last.toFixed(2):'---')+'</span>';
  }
  c.legend.innerHTML=legend;
}
async function pollTel(){
  try{
    const r=await fetch('/telemetry?from='+tpos);
    const next=r.headers.get('X-Log-Next');
    const text=tpartial+await r.text();
    const lines=text.split('\n');tpartial=lines.pop();
    for(const l of lines)telLine(l.trim());
    const more=parseInt(next)-parseInt(tpos)>=4000;
    tpos=next;
    const n=tel.rows.TEL.length;
    document.getElementById('telstate').textContent=n?'':'waiting for telemetry...';
    if(!paused){charts.forEach(draw);drawLevel();}
    setTimeout(pollTel,more?0:300);
  }catch(e){setTimeout(pollTel,2000);}
}
pollTel();
document.getElementById('window').onchange=()=>charts.forEach(draw);

/* ---- CAN bus: every message received, its age and values ---- */
function drawCan(c){
  const b=c.bus;
  document.getElementById('canstate').innerHTML=
    mark(b.state==='running',b.state)+', '+(b.bitrate/1000)+' kbit/s, TX GPIO'+b.tx_pin+
    ' RX GPIO'+b.rx_pin+' - '+b.frames_per_s+' frames/s, '+b.frames+' total, unknown '+
    b.unknown+', bad length '+b.bad_length+', '+mark(!b.bus_errors,'bus errors '+b.bus_errors)+
    ', lost '+b.rx_lost+', bus off '+b.bus_off+', error counters TX '+b.tec+' RX '+b.rec;
  let h='<tr><th>ID</th><th>message</th><th>from</th><th>age</th><th>count</th><th>values</th></tr>';
  for(const m of c.msgs){
    const age=m.age_ms===null?'never':m.age_ms<10000?m.age_ms+' ms':(m.age_ms/1000).toFixed(0)+' s';
    let v=Object.entries(m.v).map(([k,x])=>k+' '+(x===null?'---':x)).join('  ');
    if(m.name==='GPS_TIME'&&m.count&&(m.v.flags&1))
      v+='  '+new Date(m.v.utc_s*1000+m.v.ms).toISOString();
    h+='<tr class="'+(m.fresh?'':'stale')+'"><td>0x'+m.id.toString(16).toUpperCase().padStart(3,'0')+
      '</td><td>'+m.name+'</td><td>'+m.from+'</td><td>'+(m.count?mark(m.fresh,age):age)+
      '</td><td>'+m.count+'</td><td>'+(m.count?v:'')+'</td></tr>';
  }
  document.getElementById('can').innerHTML=h;
  const T={2:'GPS',3:'air data',4:'engine unit'},S=['booting','running','degraded','fault'];
  document.getElementById('cannodes').innerHTML=c.nodes.length?'nodes: '+c.nodes.map(n=>
    mark(n.fresh&&n.state===1,'#'+n.node+' '+(T[n.type]||'type '+n.type)+' '+(S[n.state]||n.state)+
      ', fw '+n.fw+', up '+n.uptime_s+' s'+(n.fresh?'':' - gone'))).join('   '):'no heartbeats heard';
}
async function pollCan(){
  try{const c=await(await fetch('/can')).json();if(!paused)drawCan(c);}
  catch(e){document.getElementById('canstate').textContent='not responding';}
  setTimeout(pollCan,1000);
}
pollCan();
poll();status();setInterval(status,3000);
</script></body></html>
)html";

static esp_err_t handle_page(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE, sizeof(PAGE) - 1);
}

/* ---- status --------------------------------------------------------------- */

/* A JSON string body: quotes, backslashes and control characters. */
static void json_escape(const char *text, char *out, size_t size)
{
    size_t used = 0;

    for(; *text != '\0' && used + 7 < size; text++)
    {
        const unsigned char c = (unsigned char)*text;

        if(c == '"' || c == '\\')
        {
            out[used++] = '\\';
            out[used++] = (char)c;
        }
        else if(c < 0x20)
        {
            used += snprintf(out + used, size - used, "\\u%04x", c);
        }
        else
        {
            out[used++] = (char)c;
        }
    }

    out[used] = '\0';
}

static esp_err_t handle_status(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();

    wifi_debug_status_t status;
    wifi_debug_get_status(&status);

    char network[96];

    if(status.joined)
        snprintf(network, sizeof(network), "%s %s", status.ssid, status.ip);
    else
        snprintf(network, sizeof(network), "access point %s",
                 status.ap_ssid);

    int rssi = 0;
    wifi_ap_record_t ap;

    if(status.joined && esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        rssi = ap.rssi;

    char reset[200];
    json_escape(crash_report_text(), reset, sizeof(reset));

    char escaped_network[200];
    json_escape(network, escaped_network, sizeof(escaped_network));

    char json[640];

    snprintf(json, sizeof(json),
             "{\"version\":\"%s\",\"built\":\"%s %s\",\"partition\":\"%s\","
             "\"uptime_s\":%lld,\"heap_internal\":%u,\"heap_psram\":%u,"
             "\"network\":\"%s\",\"rssi\":%d,\"reset\":\"%s\"}",
             app->version, app->date, app->time,
             esp_ota_get_running_partition()->label,
             (long long)(esp_timer_get_time() / 1000000),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             escaped_network, rssi, reset);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

/* ---- log ------------------------------------------------------------------ */

/*
 * GET /log?from=N and /stream?from=N: what was written from position N
 * on, up to TRANSFER_BYTES of it, with the position to ask for next in
 * X-Log-Next. from=end returns nothing but that position, for a reader
 * that wants only what comes from now on.
 */
static esp_err_t send_ring(httpd_req_t *req,
                           size_t (*read)(uint32_t *, char *, size_t),
                           uint32_t (*end)(void))
{
    uint32_t position = 0;
    char query[32];
    char value[16];

    if(httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
       httpd_query_key_value(query, "from", value, sizeof(value)) == ESP_OK)
        position = (strcmp(value, "end") == 0)
                       ? end() : (uint32_t)strtoul(value, NULL, 10);

    char *buffer = (char *)heap_caps_malloc(TRANSFER_BYTES, MALLOC_CAP_SPIRAM);

    if(buffer == NULL)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "out of memory");

    const size_t length = read(&position, buffer, TRANSFER_BYTES);

    char next[12];
    snprintf(next, sizeof(next), "%lu", (unsigned long)position);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "X-Log-Next", next);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    const esp_err_t err = httpd_resp_send(req, buffer, (ssize_t)length);

    free(buffer);

    return err;
}

static esp_err_t handle_log(httpd_req_t *req)
{
    return send_ring(req, web_log_read, web_log_end);
}

/* The $DIAG/$COMP sentences, while the diagnostics or compass screen is
 * showing - for tools/sensor_plot.py --url. */
static esp_err_t handle_stream(httpd_req_t *req)
{
    return send_ring(req, web_stream_read, web_stream_end);
}

/* The $TEL/$SYS sentences, always (diagnostics.h) - the page's charts. */
static esp_err_t handle_telemetry(httpd_req_t *req)
{
    return send_ring(req, web_telemetry_read, web_telemetry_end);
}

/* ---- screenshot and screen ------------------------------------------------- */

/*
 * GET /screenshot.bmp: the picture on the panel now, as a 16 bit BMP -
 * the framebuffer's own RGB565, nothing converted, bottom row first as
 * BMP has it.
 *
 * Copied whole first, in a few milliseconds, then sent: sending takes a
 * good fraction of a second, and LVGL swaps buffers every 40 ms and
 * starts repainting the one it gets back, so read out while sending,
 * the picture came out striped with half drawn frames.
 */
static esp_err_t handle_screenshot(httpd_req_t *req)
{
    const uint16_t *shown = (const uint16_t *)lvgl_port_shown_frame();

    if(shown == NULL)
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                                   "no frame drawn yet");

    const uint32_t width = BOARD_LCD_WIDTH;
    const uint32_t height = BOARD_LCD_HEIGHT;
    const uint32_t row_bytes = width * 2;          /* a multiple of 4 */
    const uint32_t pixel_bytes = row_bytes * height;

    uint16_t *frame = (uint16_t *)heap_caps_malloc(pixel_bytes,
                                                   MALLOC_CAP_SPIRAM);

    if(frame == NULL)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "out of memory");

    memcpy(frame, (const void *)lvgl_port_shown_frame(), pixel_bytes);
    const uint32_t header_bytes = 14 + 40 + 12;

    uint8_t header[14 + 40 + 12] = {};

    auto put32 = [&](int at, uint32_t value) {
        for(int i = 0; i < 4; i++)
            header[at + i] = (uint8_t)(value >> (8 * i));
    };

    header[0] = 'B';
    header[1] = 'M';
    put32(2, header_bytes + pixel_bytes);
    put32(10, header_bytes);
    put32(14, 40);                  /* BITMAPINFOHEADER */
    put32(18, width);
    put32(22, height);              /* positive: bottom row first */
    header[26] = 1;                 /* planes */
    header[28] = 16;                /* bits per pixel */
    put32(30, 3);                   /* BI_BITFIELDS */
    put32(34, pixel_bytes);
    put32(54, 0xF800);              /* red, green, blue masks: RGB565 */
    put32(58, 0x07E0);
    put32(62, 0x001F);

    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    esp_err_t err = httpd_resp_send_chunk(req, (const char *)header,
                                          sizeof(header));

    static const uint32_t ROWS_PER_CHUNK = 4;
    uint8_t *buffer =
        (uint8_t *)heap_caps_malloc(row_bytes * ROWS_PER_CHUNK,
                                    MALLOC_CAP_SPIRAM);

    if(buffer == NULL)
        err = ESP_ERR_NO_MEM;

    for(uint32_t done = 0; err == ESP_OK && done < height;
        done += ROWS_PER_CHUNK)
    {
        for(uint32_t r = 0; r < ROWS_PER_CHUNK; r++)
            memcpy(buffer + r * row_bytes,
                   frame + (height - 1 - (done + r)) * width, row_bytes);

        err = httpd_resp_send_chunk(req, (const char *)buffer,
                                    row_bytes * ROWS_PER_CHUNK);
    }

    free(buffer);
    free(frame);

    if(err == ESP_OK)
        err = httpd_resp_send_chunk(req, NULL, 0);

    return err;
}

/* GET /screen?id=N: show screen N (screen_manager.h's order). */
static esp_err_t handle_screen(httpd_req_t *req)
{
    char query[32];
    char value[8];

    if(httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
       httpd_query_key_value(query, "id", value, sizeof(value)) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "screen?id=N, 0 to 5");

    const int id = atoi(value);

    if(id < 0 || id >= SCREEN_COUNT)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "no such screen");

    screen_manager_request((screen_id_t)id);

    return httpd_resp_sendstr(req, "ok");
}

/* ---- compass calibration ---------------------------------------------------- */

/*
 * GET /compass: the compass swing, for the page to draw - whether one
 * is in progress, how far it has got, every point kept in the
 * horizontal plane and the ellipse fitted to them - and the calibration
 * saved now. JSON.
 */
static esp_err_t handle_compass(httpd_req_t *req)
{
    MagCalibrator::Snapshot *snap = (MagCalibrator::Snapshot *)
        heap_caps_malloc(sizeof(MagCalibrator::Snapshot), MALLOC_CAP_SPIRAM);

    static const size_t JSON_BYTES = 32 * 1024;
    char *json = (char *)heap_caps_malloc(JSON_BYTES, MALLOC_CAP_SPIRAM);

    if(snap == NULL || json == NULL)
    {
        free(snap);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "out of memory");
    }

    const bool calibrating = magnetometer_calibration_snapshot(snap);
    const MagCalibration saved = magnetometer_calibration();
    const MagCalibratorConfig limits;

    size_t n = 0;

    auto add = [&](const char *format, auto... values) {
        if(n < JSON_BYTES)
            n += snprintf(json + n, JSON_BYTES - n, format, values...);
    };

    add("{\"present\":%s,\"calibrating\":%s,",
        mag_present() ? "true" : "false", calibrating ? "true" : "false");

    add("\"saved\":{\"valid\":%s,\"offset\":[%.2f,%.2f,%.2f],"
        "\"scale\":[%.4f,%.4f,%.4f],\"field\":%.2f},",
        saved.valid ? "true" : "false",
        saved.offset.x, saved.offset.y, saved.offset.z,
        saved.scale.x, saved.scale.y, saved.scale.z, saved.field);

    add("\"limits\":{\"coverage\":%.3f,\"tilt\":%.1f,\"fit_error\":%.3f,"
        "\"samples\":%u,\"dip\":%d}",
        limits.min_coverage_deg / 360.0f, limits.max_tilt_spread_deg,
        limits.max_fit_error, (unsigned)limits.min_samples,
        (int)CONFIG_MAG_DIP_DEG);

    if(calibrating)
    {
        const MagCalibrator::Progress &p = snap->progress;

        add(",\"progress\":{\"samples\":%u,\"coverage\":%.3f,\"tilt\":%.2f,"
            "\"radius\":%.2f,\"fit_error\":%.4f,\"ready\":%s}",
            (unsigned)p.samples, p.coverage, p.tilt_spread_deg, p.radius,
            p.fit_error, p.ready ? "true" : "false");

        if(snap->fitted)
            add(",\"fit\":{\"cu\":%.3f,\"cv\":%.3f,\"au\":%.3f,\"av\":%.3f}",
                snap->centre_u, snap->centre_v, snap->axis_u, snap->axis_v);

        add(",\"points\":[");

        for(int i = 0; i < snap->count; i++)
            add("%s[%.2f,%.2f]", i ? "," : "", snap->u[i], snap->v[i]);

        add("]");
    }

    add("}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    const esp_err_t err = (n < JSON_BYTES)
        ? httpd_resp_send(req, json, (ssize_t)n)
        : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                              "too long");

    free(snap);
    free(json);

    return err;
}

/* ---- CAN bus -------------------------------------------------------------- */

static const char *can_state_name(can_bus_state_t state)
{
    switch(state)
    {
    case CAN_BUS_RUNNING:       return "running";
    case CAN_BUS_ERROR_PASSIVE: return "error passive";
    case CAN_BUS_BUS_OFF:       return "bus off, recovering";
    default:                    return "not started";
    }
}

/*
 * GET /can: everything received from the CAN bus - the bus's own
 * counters, then every message of the protocol with how long ago it
 * last came, whether that is still within its timeout, and its values
 * (null where the sender marked one not available), then the nodes
 * heard from. JSON.
 */
static esp_err_t handle_can(httpd_req_t *req)
{
    can_data_t *data = (can_data_t *)
        heap_caps_malloc(sizeof(can_data_t), MALLOC_CAP_SPIRAM);

    static const size_t JSON_BYTES = 12 * 1024;
    char *json = (char *)heap_caps_malloc(JSON_BYTES, MALLOC_CAP_SPIRAM);

    if(data == NULL || json == NULL)
    {
        free(data);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "out of memory");
    }

    can_bus_get(data);
    const int64_t now_us = esp_timer_get_time();

    size_t n = 0;

    auto add = [&](const char *format, auto... values) {
        if(n < JSON_BYTES)
            n += snprintf(json + n, JSON_BYTES - n, format, values...);
    };

    /* A value, or null for "not available" - JSON has no NaN. */
    auto value = [&](const char *key, double v, int decimals) {
        if(isfinite(v))
            add("\"%s\":%.*f,", key, decimals, v);
        else
            add("\"%s\":null,", key);
    };

    /* The comma the last value left, before a closing bracket. */
    auto close = [&](const char *bracket) {
        if(n > 0 && n < JSON_BYTES && json[n - 1] == ',')
            n--;
        add("%s", bracket);
    };

    auto message = [&](unsigned id, const char *name, const char *from,
                       const can_stamp_t &stamp, uint32_t period_ms) {
        add("{\"id\":%u,\"name\":\"%s\",\"from\":\"%s\",\"period_ms\":%lu,"
            "\"count\":%lu,",
            id, name, from, (unsigned long)period_ms,
            (unsigned long)stamp.count);

        if(stamp.count > 0)
            add("\"age_ms\":%lld,",
                (long long)((now_us - stamp.rx_us) / 1000));
        else
            add("\"age_ms\":null,");

        add("\"fresh\":%s,\"v\":{",
            can_fresh(&stamp, now_us, period_ms) ? "true" : "false");
    };

    const can_bus_stats_t &bus = data->bus;

    add("{\"bus\":{\"state\":\"%s\",\"bitrate\":%d,\"tx_pin\":%d,"
        "\"rx_pin\":%d,\"frames\":%lu,\"frames_per_s\":%lu,"
        "\"unknown\":%lu,\"bad_length\":%lu,\"bus_errors\":%lu,"
        "\"rx_lost\":%lu,\"bus_off\":%lu,\"tec\":%lu,\"rec\":%lu,"
        "\"rx_level\":%d},",
        can_state_name(bus.state), BOARD_CAN_BITRATE, BOARD_CAN_TX_PIN,
        BOARD_CAN_RX_PIN, (unsigned long)bus.frames,
        (unsigned long)bus.frames_per_s, (unsigned long)bus.unknown_frames,
        (unsigned long)bus.bad_length, (unsigned long)bus.bus_errors,
        (unsigned long)bus.rx_lost, (unsigned long)bus.bus_off_count,
        (unsigned long)bus.tx_error_counter,
        (unsigned long)bus.rx_error_counter, bus.rx_level);

    add("\"msgs\":[");

    const can_air_data_t &air = data->air;
    message(0x100, "DIFF_PRESSURE", "air data", air.stamp, CAN_PERIOD_AIR_MS);
    value("diff_pa", air.diff_pa, 2);
    value("sensor_c", air.sensor_temp_c, 2);
    add("\"status\":%u,\"sequence\":%u,\"lost\":%lu",
        air.status, air.sequence, (unsigned long)air.sequence_gaps);
    close("}},");

    const can_oat_t &oat = data->oat;
    message(0x101, "OAT", "air data", oat.stamp, CAN_PERIOD_OAT_MS);
    value("oat_c", oat.oat_c, 2);
    value("humidity_pct", oat.humidity_pct, 2);
    close("}},");

    const can_gps_t &gps = data->gps;
    message(0x200, "GPS_POSITION", "GPS", gps.position_stamp,
            CAN_PERIOD_GPS_MS);
    value("lat_deg", gps.latitude_deg, 7);
    value("lon_deg", gps.longitude_deg, 7);
    close("}},");

    message(0x201, "GPS_VELOCITY", "GPS", gps.velocity_stamp,
            CAN_PERIOD_GPS_MS);
    value("gs_ms", gps.ground_speed_ms, 2);
    value("track_deg", gps.track_deg, 2);
    value("vs_ms", gps.vertical_speed_ms, 2);
    add("\"fix\":%u,\"fix_type\":%u", gps.velocity_fix_sequence,
        gps.fix_type);
    close("}},");

    message(0x202, "GPS_ALTITUDE", "GPS", gps.altitude_stamp,
            CAN_PERIOD_GPS_MS);
    value("alt_msl_m", gps.altitude_msl_m, 3);
    value("geoid_m", gps.geoid_separation_m, 1);
    add("\"fix\":%u", gps.altitude_fix_sequence);
    close("}},");

    message(0x203, "GPS_QUALITY", "GPS", gps.quality_stamp,
            CAN_PERIOD_GPS_SLOW_MS);
    value("sats", gps.satellites == 0xFF ? NAN : gps.satellites, 0);
    value("hdop", gps.hdop, 1);
    value("hacc_m", gps.horizontal_accuracy_m, 2);
    value("vacc_m", gps.vertical_accuracy_m, 2);
    value("sacc_ms", gps.speed_accuracy_ms, 2);
    close("}},");

    message(0x204, "GPS_TIME", "GPS", gps.time_stamp, CAN_PERIOD_GPS_SLOW_MS);
    add("\"utc_s\":%lu,\"ms\":%u,\"flags\":%u",
        (unsigned long)gps.utc_s, gps.utc_ms, gps.time_flags);
    close("}},");

    for(int e = 0; e < CAN_ENGINES; e++)
    {
        const can_engine_t &engine = data->engine[e];
        const unsigned base = 0x300 + 0x20 * e;
        const char *from = (e == 0) ? "engine 1" : "engine 2";

        /* Engine 2 only once it has been heard from. */
        if(e > 0 && engine.fast_stamp.count == 0 &&
           engine.slow_stamp.count == 0)
            continue;

        message(base + 0, "ENGINE_FAST", from, engine.fast_stamp,
                CAN_PERIOD_ENGINE_FAST_MS);
        value("rpm", engine.rpm, 0);
        value("map_kpa", engine.manifold_kpa, 2);
        value("oil_kpa", engine.oil_pressure_kpa, 1);
        value("fuel_kpa", engine.fuel_pressure_kpa, 1);
        close("}},");

        message(base + 1, "ENGINE_SLOW", from, engine.slow_stamp,
                CAN_PERIOD_ENGINE_SLOW_MS);
        value("oil_c", engine.oil_temp_c, 1);
        value("coolant_c", engine.coolant_temp_c, 1);
        value("flow_lph", engine.fuel_flow_lph, 2);
        value("volts", engine.bus_volts, 2);
        close("}},");

        message(base + 2, "EGT_1_4", from, engine.egt_stamp,
                CAN_PERIOD_ENGINE_SLOW_MS);
        for(int i = 0; i < 4; i++)
        {
            char key[8];
            snprintf(key, sizeof(key), "egt%d_c", i + 1);
            value(key, engine.egt_c[i], 1);
        }
        close("}},");

        message(base + 3, "CHT_1_4", from, engine.cht_stamp,
                CAN_PERIOD_ENGINE_SLOW_MS);
        for(int i = 0; i < 4; i++)
        {
            char key[8];
            snprintf(key, sizeof(key), "cht%d_c", i + 1);
            value(key, engine.cht_c[i], 1);
        }
        close("}},");

        message(base + 4, "CYL_5_6", from, engine.cyl56_stamp,
                CAN_PERIOD_ENGINE_SLOW_MS);
        value("egt5_c", engine.egt_c[4], 1);
        value("egt6_c", engine.egt_c[5], 1);
        value("cht5_c", engine.cht_c[4], 1);
        value("cht6_c", engine.cht_c[5], 1);
        close("}},");

        message(base + 5, "FUEL_ELECTRICAL", from, engine.fuel_stamp,
                CAN_PERIOD_FUEL_MS);
        value("fuel_left_l", engine.fuel_left_l, 1);
        value("fuel_right_l", engine.fuel_right_l, 1);
        value("amps", engine.battery_amps, 2);
        value("intake_c", engine.intake_temp_c, 1);
        close("}},");

        message(base + 6, "ENGINE_TOTALS", from, engine.totals_stamp,
                CAN_PERIOD_TOTALS_MS);
        value("engine_s", engine.engine_time_s == 0xFFFFFFFFu
                              ? NAN : (double)engine.engine_time_s, 0);
        value("used_l", engine.fuel_used_l, 1);
        value("endurance_min", engine.endurance_min, 0);
        close("}},");
    }

    close("],\"nodes\":[");

    for(int i = 0; i < CAN_NODES; i++)
    {
        const can_node_t &node = data->node[i];

        if(node.stamp.count == 0)
            continue;

        add("{\"node\":%d,\"type\":%u,\"state\":%u,\"fw\":\"%u.%u\","
            "\"uptime_s\":%lu,\"age_ms\":%lld,\"fresh\":%s},",
            i, node.device_type, node.state, node.firmware >> 8,
            node.firmware & 0xFF, (unsigned long)node.uptime_s,
            (long long)((now_us - node.stamp.rx_us) / 1000),
            can_fresh(&node.stamp, now_us, CAN_PERIOD_HEARTBEAT_MS)
                ? "true" : "false");
    }

    close("]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    const esp_err_t err = (n < JSON_BYTES)
        ? httpd_resp_send(req, json, (ssize_t)n)
        : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                              "too long");

    free(data);
    free(json);

    return err;
}

/* ---- firmware update ------------------------------------------------------ */

/* Enough of the image to read its description. */
static const size_t IMAGE_HEAD_BYTES = sizeof(esp_image_header_t) +
                                       sizeof(esp_image_segment_header_t) +
                                       sizeof(esp_app_desc_t);

/*
 * This firmware's builds: its own name, and the one it had before it
 * was renamed, so that either can replace the other over Wi-Fi.
 */
static bool project_accepted(const char *name, size_t size)
{
    static const char *const FORMER_NAMES[] = { "artificial_horizon" };

    if(strncmp(name, esp_app_get_description()->project_name, size) == 0 ||
       strncmp(name, "miniefis", size) == 0)
        return true;

    for(const char *former : FORMER_NAMES)
        if(strncmp(name, former, size) == 0)
            return true;

    return false;
}

/*
 * Turns away anything that is not a build of this firmware - the
 * bootloader, the partition table, another project - before a byte of
 * flash is erased.
 */
static const char *image_problem(const uint8_t *data, size_t length)
{
    if(length < IMAGE_HEAD_BYTES)
        return "too short to be a firmware image";

    esp_image_header_t header;
    memcpy(&header, data, sizeof(header));

    if(header.magic != ESP_IMAGE_HEADER_MAGIC)
        return "not a firmware image";

    if(header.chip_id != ESP_CHIP_ID_ESP32S3)
        return "built for another chip";

    esp_app_desc_t app;
    memcpy(&app, data + sizeof(esp_image_header_t) +
                     sizeof(esp_image_segment_header_t), sizeof(app));

    if(app.magic_word != ESP_APP_DESC_MAGIC_WORD)
        return "not an application image (bootloader? partition table?)";

    if(!project_accepted(app.project_name, sizeof(app.project_name)))
        return "a different project's firmware";

    return NULL;
}

static esp_err_t update_failed(httpd_req_t *req, const char *why)
{
    ESP_LOGE(TAG, "update failed: %s", why);

    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, why);
}

/*
 * Bytes [*received, until) of the upload, into image. Returns what went
 * wrong, or NULL. Reports progress once the screen is the update's.
 */
static const char *receive_range(httpd_req_t *req, uint8_t *image,
                                 size_t *received, size_t until,
                                 bool show_progress)
{
    int timeouts = 0;

    while(*received < until)
    {
        size_t want = until - *received;

        if(want > TRANSFER_BYTES)
            want = TRANSFER_BYTES;

        const int got = httpd_req_recv(req, (char *)image + *received, want);

        if(got == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5)
            continue;

        if(got <= 0)
            return "connection lost during upload";

        *received += (size_t)got;

        if(show_progress)
            update_screen_progress(UPDATE_SCREEN_RECEIVING,
                                   (uint8_t)((uint64_t)*received * 100 /
                                             req->content_len));
    }

    return NULL;
}

/*
 * Erases and writes the other slot, while the display says so
 * (update_screen.h).
 *
 * The upload goes to PSRAM first and the flash work is done in one go
 * afterwards: a bad or broken upload is turned away before anything is
 * erased, and the writing is not held up by the network. The display
 * is generated from internal RAM meanwhile (update_screen.h), so the
 * flash writes cannot disturb it.
 *
 * A block at a time, through the partition API rather than
 * esp_ota_write(): that erases 4 KB sectors, one command each, which
 * took 27 s for an image against 15 s by 64 KB blocks. Setting the
 * boot partition checks the whole image, hash included, before it is
 * accepted, as esp_ota_end() would.
 */
static const char *write_image(const esp_partition_t *target,
                               const uint8_t *image, size_t length)
{
    for(size_t offset = 0; offset < length; offset += WRITE_BLOCK_BYTES)
    {
        const size_t chunk = (length - offset < WRITE_BLOCK_BYTES)
                                 ? length - offset : WRITE_BLOCK_BYTES;

        if(esp_partition_erase_range(target, offset, WRITE_BLOCK_BYTES)
               != ESP_OK)
            return "flash erase failed";

        if(esp_partition_write(target, offset, image + offset, chunk)
               != ESP_OK)
            return "flash write failed";

        update_screen_progress(UPDATE_SCREEN_WRITING,
                               (uint8_t)((uint64_t)(offset + chunk) * 100 /
                                         length));

        vTaskDelay(pdMS_TO_TICKS(BLOCK_PAUSE_MS));
    }

    const esp_err_t selected = esp_ota_set_boot_partition(target);

    if(selected == ESP_ERR_OTA_VALIDATE_FAILED)
        return "image is corrupt";

    if(selected != ESP_OK)
        return "could not select the new image";

    return NULL;
}

static esp_err_t handle_update(httpd_req_t *req)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);

    if(target == NULL)
        return update_failed(req, "no OTA partition - this unit still "
                             "has the old partition table, flash once over USB");

    if(req->content_len > target->size)
        return update_failed(req, "larger than the OTA partition");

    uint8_t *image =
        (uint8_t *)heap_caps_malloc(req->content_len, MALLOC_CAP_SPIRAM);

    if(image == NULL)
        return update_failed(req, "out of memory");

    ESP_LOGI(TAG, "receiving %u bytes for %s", (unsigned)req->content_len,
             target->label);

    /*
     * The head first, with the instruments still up: a wrong file is
     * turned away without disturbing anything. Only an image that looks
     * right takes the display over - from which the only way back is a
     * restart.
     */
    size_t received = 0;
    const size_t head = (req->content_len < IMAGE_HEAD_BYTES)
                            ? req->content_len : IMAGE_HEAD_BYTES;

    const char *problem = receive_range(req, image, &received, head, false);

    if(problem == NULL)
        problem = image_problem(image, received);

    if(problem != NULL)
    {
        free(image);
        return update_failed(req, problem);
    }

    update_screen_begin();
    update_screen_progress(UPDATE_SCREEN_RECEIVING, 0);

    problem = receive_range(req, image, &received, req->content_len, true);

    if(problem == NULL)
    {
        ESP_LOGI(TAG, "received, writing it");

        const int64_t started_us = esp_timer_get_time();

        problem = write_image(target, image, req->content_len);

        ESP_LOGI(TAG, "flash written in %lld ms",
                 (long long)((esp_timer_get_time() - started_us) / 1000));
    }

    free(image);

    if(problem != NULL)
    {
        /* The running firmware is untouched, but the instruments are
         * gone with LVGL: show why on the page, and start again. */
        update_screen_failed();
        restart_soon(UPDATE_FAILED_RESTART_US);

        return update_failed(req, problem);
    }

    ESP_LOGI(TAG, "update written to %s, restarting", target->label);
    update_screen_progress(UPDATE_SCREEN_RESTARTING, 100);

    httpd_resp_sendstr(req, "Installed - restarting. The page picks up the "
                            "log again once it is back.");
    restart_soon(RESTART_DELAY_US);

    return ESP_OK;
}

static esp_err_t handle_restart(httpd_req_t *req)
{
    ESP_LOGI(TAG, "restart requested from the web page");

    httpd_resp_sendstr(req, "restarting");
    restart_soon(RESTART_DELAY_US);

    return ESP_OK;
}

/* ---- core dump ------------------------------------------------------------ */

/* For idf.py coredump-info --core <file>, against the matching ELF. */
static esp_err_t handle_coredump(httpd_req_t *req)
{
    size_t address = 0;
    size_t size = 0;

    if(esp_core_dump_image_get(&address, &size) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                                   "no core dump stored");

    char *buffer = (char *)heap_caps_malloc(TRANSFER_BYTES, MALLOC_CAP_SPIRAM);

    if(buffer == NULL)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "out of memory");

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"coredump.elf\"");

    esp_err_t err = ESP_OK;

    for(size_t offset = 0; offset < size && err == ESP_OK;
        offset += TRANSFER_BYTES)
    {
        const size_t length = (size - offset < TRANSFER_BYTES)
                                  ? size - offset : TRANSFER_BYTES;

        err = esp_flash_read(NULL, buffer, address + offset, length);

        if(err == ESP_OK)
            err = httpd_resp_send_chunk(req, buffer, (ssize_t)length);
    }

    free(buffer);

    if(err == ESP_OK)
        err = httpd_resp_send_chunk(req, NULL, 0);

    return err;
}

/* ---- network to join ------------------------------------------------------ */

static int hex_digit(char c)
{
    if(c >= '0' && c <= '9') return c - '0';
    if(c >= 'a' && c <= 'f') return c - 'a' + 10;
    if(c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* One field of an application/x-www-form-urlencoded body, decoded. */
static bool form_value(const char *body, const char *key, char *out,
                       size_t size)
{
    const size_t key_length = strlen(key);
    const char *p = body;

    while(p != NULL && *p != '\0')
    {
        if(strncmp(p, key, key_length) == 0 && p[key_length] == '=')
        {
            p += key_length + 1;
            size_t used = 0;

            while(*p != '\0' && *p != '&' && used + 1 < size)
            {
                if(*p == '+')
                {
                    out[used++] = ' ';
                    p++;
                }
                else if(*p == '%' && hex_digit(p[1]) >= 0 &&
                        hex_digit(p[2]) >= 0)
                {
                    out[used++] = (char)(hex_digit(p[1]) * 16 +
                                         hex_digit(p[2]));
                    p += 3;
                }
                else
                {
                    out[used++] = *p++;
                }
            }

            out[used] = '\0';
            return true;
        }

        p = strchr(p, '&');

        if(p != NULL)
            p++;
    }

    return false;
}

static esp_err_t handle_wifi(httpd_req_t *req)
{
    char body[256];

    if(req->content_len >= sizeof(body))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "too long");

    size_t used = 0;

    while(used < req->content_len)
    {
        const int received = httpd_req_recv(req, body + used,
                                            req->content_len - used);

        if(received <= 0)
            return ESP_FAIL;

        used += (size_t)received;
    }

    body[used] = '\0';

    char ssid[33];
    char password[65];

    if(!form_value(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0' ||
       !form_value(body, "password", password, sizeof(password)))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "network name missing");

    if(save_credentials(ssid, password) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "could not save");

    ESP_LOGI(TAG, "network set to \"%s\" from the web page", ssid);

    s_new_credentials = true;
    xTaskNotifyGive(s_task);

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req,
        "Saved - joining it now. The unit's address on that network is "
        "shown on its settings screen. <a href=\"/\">back</a>");
}

/* ---- server --------------------------------------------------------------- */

static void start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.stack_size = SERVER_STACK_BYTES;
    config.task_priority = TASK_PRIORITY;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 16;       /* the default 8 is all taken */

    if(httpd_start(&s_server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "could not start the web server");
        s_server = NULL;
        return;
    }

    static const httpd_uri_t handlers[] = {
        { "/",         HTTP_GET,  handle_page,     NULL },
        { "/status",   HTTP_GET,  handle_status,   NULL },
        { "/log",      HTTP_GET,  handle_log,      NULL },
        { "/stream",   HTTP_GET,  handle_stream,   NULL },
        { "/telemetry", HTTP_GET, handle_telemetry, NULL },
        { "/compass",  HTTP_GET,  handle_compass,  NULL },
        { "/can",      HTTP_GET,  handle_can,      NULL },
        { "/screenshot.bmp", HTTP_GET, handle_screenshot, NULL },
        { "/screen",   HTTP_GET,  handle_screen,   NULL },
        { "/coredump", HTTP_GET,  handle_coredump, NULL },
        { "/update",   HTTP_POST, handle_update,   NULL },
        { "/restart",  HTTP_POST, handle_restart,  NULL },
        { "/wifi",     HTTP_POST, handle_wifi,     NULL },
    };

    for(const httpd_uri_t &handler : handlers)
        httpd_register_uri_handler(s_server, &handler);

    /* Up and reachable again: a new image has proved itself. */
    confirm_running_image();
}

/* ---- Wi-Fi ---------------------------------------------------------------- */

static void on_event(void *argument, esp_event_base_t base, int32_t id,
                     void *data)
{
    (void)argument;

    if(base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        s_sta_started = true;
    }
    else if(base == WIFI_EVENT && id == WIFI_EVENT_STA_STOP)
    {
        s_sta_started = false;
        s_attempting = false;
    }
    else if(base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        s_attempting = false;

        status_update([](wifi_debug_status_t *status) {
            status->joined = false;
            status->ip[0] = '\0';
        });
    }
    else if(base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        static char ip[16];
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;

        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "joined, web page at http://%s", ip);

        status_update([](wifi_debug_status_t *status) {
            status->joined = true;
            status->joining = false;
            strlcpy(status->ip, ip, sizeof(status->ip));
        });
    }
    else
    {
        return;
    }

    xTaskNotifyGive(s_task);
}

static bool initialise(void)
{
    esp_err_t err = esp_netif_init();

    if(err == ESP_OK)
    {
        err = esp_event_loop_create_default();

        if(err == ESP_ERR_INVALID_STATE)    /* someone else made it */
            err = ESP_OK;
    }

    if(err == ESP_OK)
    {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        esp_netif_create_default_wifi_ap();

        const wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&config);
    }

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi did not start: %s", esp_err_to_name(err));
        return false;
    }

    /* The configuration is set afresh every time; keeping the driver's
     * own copy in NVS would only write flash. */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_netif_set_hostname(s_sta_netif, "miniefis");

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);

    return true;
}

static void configure_station(void)
{
    wifi_config_t config;
    memset(&config, 0, sizeof(config));

    strlcpy((char *)config.sta.ssid, s_ssid, sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, s_password,
            sizeof(config.sta.password));
    config.sta.threshold.authmode =
        (s_password[0] != '\0') ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    esp_wifi_set_config(WIFI_IF_STA, &config);

    status_update([](wifi_debug_status_t *status) {
        strlcpy(status->ssid, s_ssid, sizeof(status->ssid));
        status->joining = (s_ssid[0] != '\0');
        status->joined = false;
        status->ip[0] = '\0';
    });

    s_join_started_us = esp_timer_get_time();
    s_last_attempt_us = 0;
}

/* Our own network, alongside the attempts to join the configured one. */
static void open_access_point(void)
{
    static char ap_ssid[33];
    static bool ap_open;

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    snprintf(ap_ssid, sizeof(ap_ssid), "MiniEFIS-%02X%02X", mac[4], mac[5]);

    const char *password = CONFIG_WIFI_DEBUG_AP_PASSWORD;
    ap_open = strlen(password) < 8;

    wifi_config_t config;
    memset(&config, 0, sizeof(config));

    strlcpy((char *)config.ap.ssid, ap_ssid, sizeof(config.ap.ssid));
    config.ap.ssid_len = (uint8_t)strlen(ap_ssid);
    config.ap.max_connection = 2;
    config.ap.authmode = ap_open ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    if(!ap_open)
        strlcpy((char *)config.ap.password, password,
                sizeof(config.ap.password));

    /* A change of mode, so a clean restart of the driver. */
    esp_wifi_stop();
    esp_wifi_set_mode(s_ssid[0] != '\0' ? WIFI_MODE_APSTA : WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &config);

    if(s_ssid[0] != '\0')
        configure_station();

    esp_wifi_start();

    s_fallback = true;

    ESP_LOGI(TAG, "access point \"%s\"%s, web page at http://192.168.4.1",
             ap_ssid, ap_open ? " (open)" : "");

    status_update([](wifi_debug_status_t *status) {
        status->access_point = true;
        status->ap_open = ap_open;
        strlcpy(status->ap_ssid, ap_ssid, sizeof(status->ap_ssid));
    });
}

static void start(void)
{
    if(!s_initialised)
    {
        s_initialised = initialise();

        if(!s_initialised)
            return;
    }

    load_credentials();

    s_fallback = false;
    s_running = true;

    if(s_ssid[0] == '\0')
    {
        ESP_LOGI(TAG, "no network set");
        open_access_point();
    }
    else
    {
        ESP_LOGI(TAG, "joining \"%s\"", s_ssid);
        esp_wifi_set_mode(WIFI_MODE_STA);
        configure_station();
        esp_wifi_start();
    }

    start_server();
}

static void stop(void)
{
    s_running = false;

    if(s_server != NULL)
    {
        httpd_stop(s_server);
        s_server = NULL;
    }

    esp_wifi_stop();

    /*
     * Switching off is also how to start over: a network saved from the
     * web page that cannot be joined any more - or was mistyped - would
     * otherwise be tried every time. Only the switch stops it; nothing
     * else does.
     */
    forget_credentials();

    ESP_LOGI(TAG, "off");

    status_update([](wifi_debug_status_t *status) {
        const bool enabled = status->enabled;
        memset(status, 0, sizeof(*status));
        status->enabled = enabled;
    });
}

static bool access_point_in_use(void)
{
    wifi_sta_list_t stations;

    return s_fallback && esp_wifi_ap_get_sta_list(&stations) == ESP_OK &&
           stations.num > 0;
}

/* Keeps trying the network, and opens the access point if it will not
 * have us. */
static void keep_joining(void)
{
    if(s_ssid[0] == '\0')
        return;

    wifi_debug_status_t status;
    wifi_debug_get_status(&status);

    const int64_t now = esp_timer_get_time();

    /* The time allowed to join runs from when the connection is lost,
     * not from start-up. */
    if(status.joined)
        s_join_started_us = now;

    if(status.joined || s_attempting || !s_sta_started)
        return;

    if(!s_fallback && now - s_join_started_us > JOIN_TIMEOUT_US)
    {
        ESP_LOGW(TAG, "could not join \"%s\"", s_ssid);
        open_access_point();
        return;
    }

    /* Someone is on the access point: scanning would cut them off. */
    if(access_point_in_use())
        return;

    const int64_t interval = s_fallback ? FALLBACK_RETRY_US : RETRY_US;

    if(s_last_attempt_us != 0 && now - s_last_attempt_us < interval)
        return;

    s_last_attempt_us = now;

    if(esp_wifi_connect() == ESP_OK)
        s_attempting = true;
}

static void wifi_task(void *argument)
{
    (void)argument;

    while(true)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        if(s_wanted && !s_running)
            start();
        else if(!s_wanted && s_running)
            stop();

        if(!s_running)
            continue;

        if(s_new_credentials)
        {
            s_new_credentials = false;
            load_credentials();

            esp_wifi_disconnect();
            s_attempting = false;

            /* From the access point there was no station to set up. */
            if(s_fallback)
                open_access_point();
            else
                configure_station();
        }

        keep_joining();
    }
}

void wifi_debug_set_enabled(bool enabled)
{
    s_wanted = enabled;

    portENTER_CRITICAL(&s_status_lock);
    s_status.enabled = enabled;
    portEXIT_CRITICAL(&s_status_lock);

    if(s_task != NULL)
        xTaskNotifyGive(s_task);
}

void wifi_debug_init(void)
{
    const BaseType_t created = xTaskCreate(wifi_task, "wifi_debug",
                                           TASK_STACK_BYTES, NULL,
                                           TASK_PRIORITY, &s_task);

    if(created != pdPASS)
    {
        ESP_LOGE(TAG, "could not start the task");
        s_task = NULL;
    }

    if(s_task != NULL && app_settings().wifi_debug)
        wifi_debug_set_enabled(true);
    else
        confirm_running_image();    /* got this far: good enough */
}
