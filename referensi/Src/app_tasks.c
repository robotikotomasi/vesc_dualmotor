/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   FreeRTOS CMSIS-RTOS v2 task creation for Hoverboard ESC
  *          Creates: Motor Control, VESC Comm, Safety, LED Status tasks
  ******************************************************************************
  */

#include "app_tasks.h"
#include "mc_tasks.h"
#include "vesc_comm.h"
#include "app_control.h"
#include "hw_config.h"
#include "cmsis_os2.h"
#include "stm32f1xx_hal.h"

/*============================================================================
 * Task: Motor Control (Medium Frequency — 1 kHz)
 * The high-frequency FOC is handled by TIM1 ISR, not here.
 *============================================================================*/
static void Task_MotorControl(void *argument)
{
    (void)argument;

    for (;;) {
        MC_MediumFrequencyTask();
        osDelay(1);  /* 1 ms period */
    }
}

/*============================================================================
 * Task: Safety Monitor (Voltage, Temperature — 10 Hz)
 *============================================================================*/
static void Task_Safety(void *argument)
{
    (void)argument;

    for (;;) {
        MC_SafetyTask();
        osDelay(100);  /* 100 ms period */
    }
}

/*============================================================================
 * Task: LED Status Indicator (2 Hz blink)
 *============================================================================*/
static void Task_LED(void *argument)
{
    (void)argument;

    for (;;) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        /* Blink pattern based on motor state */
        if (motor[M_RIGHT].faultCode || motor[M_LEFT].faultCode) {
            osDelay(100);   /* Fast blink on error */
        } else if (motor[M_RIGHT].enable || motor[M_LEFT].enable) {
            osDelay(250);   /* Medium blink when motors active */
        } else {
            osDelay(500);   /* Slow blink in idle */
        }
    }
}

/*============================================================================
 * Task: Application Control (ADC/PPM/Custom — 1 kHz)
 *============================================================================*/
static void Task_AppControl(void *argument)
{
    (void)argument;

    for (;;) {
        app_control_process();
        osDelay(1);  /* 1 ms period */
    }
}

/*============================================================================
 * Create All FreeRTOS Tasks
 *============================================================================*/
void App_CreateTasks(void)
{
    /* Motor Control Task — highest priority among app tasks */
    const osThreadAttr_t mc_attr = {
        .name       = "MotorCtrl",
        .stack_size = 256 * 4,
        .priority   = osPriorityRealtime,
    };
    osThreadNew(Task_MotorControl, NULL, &mc_attr);

    /* VESC Communication Task */
    const osThreadAttr_t vesc_attr = {
        .name       = "VescComm",
        .stack_size = 384 * 4,
        .priority   = osPriorityAboveNormal,
    };
    osThreadNew(VescComm_Task, NULL, &vesc_attr);

    /* Safety Monitor Task */
    const osThreadAttr_t safety_attr = {
        .name       = "Safety",
        .stack_size = 128 * 4,
        .priority   = osPriorityNormal,
    };
    osThreadNew(Task_Safety, NULL, &safety_attr);

    /* LED Status Task */
    const osThreadAttr_t led_attr = {
        .name       = "LED",
        .stack_size = 128 * 4,
        .priority   = osPriorityLow,
    };
    osThreadNew(Task_LED, NULL, &led_attr);

    /* Application Control Task (ADC/PPM/Custom input) */
    const osThreadAttr_t app_attr = {
        .name       = "AppCtrl",
        .stack_size = 256 * 4,
        .priority   = osPriorityNormal,
    };
    osThreadNew(Task_AppControl, NULL, &app_attr);
}
