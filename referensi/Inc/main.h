/**
  ******************************************************************************
  * @file    main.h
  * @brief   Header for main.c - Hoverboard ESC dual motor controller
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void Error_Handler(void);
void SystemClock_Config(void);

/* Extern peripheral handles for global access */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
