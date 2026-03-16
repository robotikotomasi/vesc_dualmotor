/**
  ******************************************************************************
  * @file    main.c
  * @brief   Entry point for Hoverboard ESC — Dual Motor BLDC Controller
  *          MCSDK FOC + VESC Protocol + FreeRTOS CMSIS-RTOS v2
  ******************************************************************************
  */

#include "stm32f1xx_hal.h"
#include "main.h"
#include "hw_config.h"
#include "mc_tasks.h"
#include "app_tasks.h"
#include "flash_helper.h"
#include "timeout.h"
#include "app_control.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* Forward declarations */
void SystemClock_Config(void);
void Error_Handler(void);

/**
  * @brief  The application entry point.
  */
int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_USART3_Init();
    MX_NVIC_Init();

    /* TIM4 as HAL tick base — must be after HAL_Init */
    MX_TIM4_Init();

    /* Activate power latch */
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);

    /* Initialize Motor Control subsystem (MCSDK FOC for both motors)
       This configures ADC DMA and starts PWM timers */
    MX_MotorControl_Init();

    /* Initialize flash configuration system (EEPROM emulation) */
    flash_helper_init();

    /* Initialize application control layer */
    app_control_init();

    /* Initialize CMSIS-RTOS v2 kernel */
    osKernelInitialize();

    /* Create application tasks */
    App_CreateTasks();

    /* Start the scheduler — never returns */
    osKernelStart();

    /* Should never reach here */
    while (1) { }
}

/**
  * @brief  System Clock Configuration
  *         HSI + PLL = 64 MHz
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief  HAL TIM PeriodElapsedCallback — TIM4 as HAL tick base
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        HAL_IncTick();
    }
}

/**
  * @brief  Error handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

