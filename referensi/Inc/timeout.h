/**
  ******************************************************************************
  * @file    timeout.h
  * @brief   Timeout and watchdog management (VESC-style)
  *          IWDG initialization, command timeout, kill-switch
  ******************************************************************************
  */

#ifndef TIMEOUT_H
#define TIMEOUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * Timeout API
 *============================================================================*/

/* Initialize timeout system and IWDG */
void timeout_init(void);

/* Configure timeout parameters */
void timeout_configure(uint32_t timeout_ms, float brake_current);

/* Reset the command timeout counter (call when valid command received) */
void timeout_reset(void);

/* Check if timeout has occurred */
bool timeout_has_timeout(void);

/* Get the brake current to apply on timeout */
float timeout_get_brake_current(void);

/* Feed the watchdog timer (call periodically from main loop) */
void timeout_feed_WDT(void);

/* Configure IWDG for slow operations (flash erase) */
void timeout_configure_IWDG_slowest(void);

/* Restore IWDG to normal speed after slow operations */
void timeout_configure_IWDG_normal(void);

/* Get timeout counter value (ms since last reset) */
uint32_t timeout_get_counter(void);

/* Kill switch control */
void timeout_set_kill_sw(bool active);
bool timeout_get_kill_sw(void);

#ifdef __cplusplus
}
#endif

#endif /* TIMEOUT_H */
