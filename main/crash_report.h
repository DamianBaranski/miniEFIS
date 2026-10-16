#pragma once

/*
 * Why the last reset happened, and what crashed if something did.
 *
 * A crash on a unit that is not attached to a serial console leaves
 * nothing behind but a restart. To make those diagnosable, a panic
 * writes a core dump to its own flash partition (see partitions.csv),
 * and at start-up this reads its summary back: the task, the program
 * counter and the backtrace. Both go to the serial log and to the
 * diagnostics screen.
 *
 * The full dump stays in flash until the next crash replaces it, and can
 * be decoded on a PC against the matching ELF:
 *
 *   idf.py -p /dev/ttyACM0 coredump-info
 */

/* Call once, early in app_main. */
void crash_report_init(void);

/* One or two short lines for the diagnostics screen. */
const char *crash_report_text(void);
