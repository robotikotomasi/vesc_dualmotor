/*
 * shutdown.h compatibility — Simplified shutdown module
 * Overrides Src/hwconf/shutdown.h for FreeRTOS build.
 */

#ifndef SHUTDOWN_H_COMPAT
#define SHUTDOWN_H_COMPAT

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHUTDOWN_RESET()
#define SHUTDOWN_BUTTON_PRESSED     false
#define SHUTDOWN_SAVE_BACKUPDATA_TIMEOUT  180

void shutdown_init(void);
void shutdown_reset_timer(void);
bool shutdown_button_pressed(void);
float shutdown_get_inactivity_time(void);
void do_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* SHUTDOWN_H_COMPAT */
