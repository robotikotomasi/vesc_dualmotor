/**
  ******************************************************************************
  * @file    timeout.c
  * @brief   Timeout and watchdog management (VESC-style)
  *          Adapted from VESC timeout.c for STM32F103RCT6 hoverboard
  *
  * Features:
  *  - IWDG (Independent Watchdog) initialization and feeding
  *  - Command timeout detection with configurable brake current
  *  - Kill-switch support
  *  - Slow mode for flash operations
  ******************************************************************************
  */

#include "timeout.h"
#include "hw_config.h"
#include "mc_tasks.h"
#include "stm32f1xx_hal.h"

/*============================================================================
 * Private Data
 *============================================================================*/
static volatile uint32_t timeout_counter_ms = 0;
static uint32_t timeout_limit_ms = TIMEOUT_MS;
static float brake_current = TIMEOUT_BRAKE_CURRENT;
static volatile bool timeout_occurred = false;
static volatile bool kill_switch_active = false;
static IWDG_HandleTypeDef hiwdg;

/*============================================================================
 * Initialize Timeout System and IWDG
 *============================================================================*/
void timeout_init(void)
{
    timeout_counter_ms = 0;
    timeout_occurred = false;
    timeout_limit_ms = TIMEOUT_MS;
    brake_current = TIMEOUT_BRAKE_CURRENT;
    kill_switch_active = false;

    /* Initialize IWDG
     * LSI ≈ 40 kHz
     * Prescaler 4: counter clock = 10 kHz
     * Reload 140: timeout ≈ 14 ms
     * This ensures the system resets if the main loop hangs */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Reload = IWDG_RELOAD_VALUE;
    HAL_IWDG_Init(&hiwdg);
}

/*============================================================================
 * Configure Timeout Parameters
 *============================================================================*/
void timeout_configure(uint32_t ms, float brake_cur)
{
    timeout_limit_ms = ms;
    brake_current = brake_cur;
}

/*============================================================================
 * Reset Timeout Counter (call when valid command received)
 *============================================================================*/
void timeout_reset(void)
{
    timeout_counter_ms = 0;
    timeout_occurred = false;
}

/*============================================================================
 * Check Timeout Status
 *============================================================================*/
bool timeout_has_timeout(void)
{
    return timeout_occurred || kill_switch_active;
}

float timeout_get_brake_current(void)
{
    return brake_current;
}

uint32_t timeout_get_counter(void)
{
    return timeout_counter_ms;
}

/*============================================================================
 * Feed Watchdog — called from VescComm_Task at ~100 Hz (10 ms period)
 * Also checks command timeout and applies brake if needed
 *============================================================================*/
void timeout_feed_WDT(void)
{
    /* Increment timeout counter by 10 ms (matching VescComm_Task period) */
    timeout_counter_ms += 10;

    /* Check command timeout */
    if (timeout_limit_ms > 0 && timeout_counter_ms >= timeout_limit_ms) {
        if (!timeout_occurred) {
            timeout_occurred = true;
            /* Apply brake current to both motors */
            if (brake_current > 0.0f) {
                MC_SetCurrentBrake(M_RIGHT, brake_current);
                MC_SetCurrentBrake(M_LEFT, brake_current);
            } else {
                MC_StopMotor(M_RIGHT);
                MC_StopMotor(M_LEFT);
            }
        }
    }

    /* Kill switch check */
    if (kill_switch_active) {
        MC_StopMotor(M_RIGHT);
        MC_StopMotor(M_LEFT);
    }

    /* Feed IWDG — prevents hardware reset */
    HAL_IWDG_Refresh(&hiwdg);
}

/*============================================================================
 * IWDG Speed Control (for flash operations)
 *============================================================================*/
void timeout_configure_IWDG_slowest(void)
{
    /* Extend IWDG timeout for flash erase operations
     * Prescaler 256, Reload 4000 → ~26 seconds timeout */
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload = 4000;
    HAL_IWDG_Init(&hiwdg);
}

void timeout_configure_IWDG_normal(void)
{
    /* Restore normal IWDG timing */
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Reload = IWDG_RELOAD_VALUE;
    HAL_IWDG_Init(&hiwdg);
}

/*============================================================================
 * Kill Switch
 *============================================================================*/
void timeout_set_kill_sw(bool active)
{
    kill_switch_active = active;
}

bool timeout_get_kill_sw(void)
{
    return kill_switch_active;
}
