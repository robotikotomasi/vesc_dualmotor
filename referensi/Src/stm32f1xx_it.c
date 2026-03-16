/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt handlers for Hoverboard ESC
  ******************************************************************************
  */

#include "stm32f1xx_hal.h"
#include "main.h"
#include "mc_tasks.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

/*============================================================================
 * Core Fault Handlers
 *============================================================================*/
void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    /* Disable motor outputs on hard fault */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void)
{
}

/*============================================================================
 * SysTick Handler — FreeRTOS tick (called at configTICK_RATE_HZ = 1kHz)
 *============================================================================*/
void SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif
        xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif
}

/*============================================================================
 * TIM1 Update IRQ — FOC High Frequency Motor Control (16 kHz)
 *============================================================================*/
void TIM1_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);

    /* Execute motor control at PWM frequency */
    MC_HighFrequencyTask();
}

/*============================================================================
 * TIM1 Break IRQ — Overcurrent / Emergency
 *============================================================================*/
void TIM1_BRK_IRQHandler(void)
{
    /* Emergency stop — disable all motor outputs */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    HAL_TIM_IRQHandler(&htim1);
}

/*============================================================================
 * TIM4 IRQ — HAL Tick Base (1 kHz)
 *============================================================================*/
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

/*============================================================================
 * DMA1 Channel1 IRQ — ADC DMA Transfer Complete
 *============================================================================*/
void DMA1_Channel1_IRQHandler(void)
{
    /* ADC conversion complete — data is in adc_buffer
       Motor control reads it in the next HF task cycle */
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

/*============================================================================
 * ADC1/2 IRQ — ADC Injected Conversion
 *============================================================================*/
void ADC1_2_IRQHandler(void)
{
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC)) {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    }
}



/*============================================================================
 * USART3 IRQ — VESC Communication
 *============================================================================*/
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/*============================================================================
 * DMA1 Channel2 IRQ — USART3 TX
 *============================================================================*/
void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

/*============================================================================
 * DMA1 Channel3 IRQ — USART3 RX
 *============================================================================*/
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}
