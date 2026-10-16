#pragma once

#include <stdbool.h>

/*
 * Wi-Fi debug: the unit's log, and firmware updates, over Wi-Fi instead
 * of the USB serial port.
 *
 * Switched on from the settings screen (kept across restarts). It joins
 * the configured network - from menuconfig, "Wi-Fi debug", or as saved
 * from the web page - and if that fails within 20 s, or none is set,
 * also opens an access point of its own, "MiniEFIS-XXXX", where the
 * page is at http://192.168.4.1. The settings screen shows the address.
 *
 * A network saved from the web page is kept across restarts, until Wi-Fi
 * debug is switched off: that forgets it, so switching off and on again
 * starts over from menuconfig's network, or the access point.
 *
 * The page shows the log live, from start-up on (web_log.h), and takes
 * a firmware image - build/miniefis.bin - to install into the
 * other OTA slot and restart into. A new image has to come up and, if
 * it was started with Wi-Fi debug on, get its web server running again;
 * one that crashes before that is rolled back by the bootloader on the
 * next restart, so a bad update cannot lock the unit out.
 *
 * There is no login: anyone on the same network, or on the access
 * point, can install firmware. It is a debug mode - leave it off.
 */

typedef struct {
    bool enabled;
    bool joining;               /* trying to join the network */
    bool joined;                /* and got an address on it */
    char ssid[33];              /* the network, "" if none set */
    char ip[16];                /* our address on it */
    bool access_point;          /* our own access point is up */
    char ap_ssid[33];
    bool ap_open;               /* without a password */
} wifi_debug_status_t;

/*
 * Call at the end of app_main, once everything else is running: getting
 * that far is what marks a newly installed image as good when Wi-Fi
 * debug is off. Starts Wi-Fi if the saved setting says so.
 */
void wifi_debug_init(void);

/* From any task; the work happens in a task of its own. */
void wifi_debug_set_enabled(bool enabled);

void wifi_debug_get_status(wifi_debug_status_t *status);
