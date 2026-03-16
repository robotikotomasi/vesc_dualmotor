/*
 * stm32f4xx_conf.h compatibility → redirect to STM32F1xx HAL
 * This replaces the ChibiOS STM32F4xx peripheral includes.
 */

#ifndef STM32F4XX_CONF_COMPAT_H
#define STM32F4XX_CONF_COMPAT_H

#include "stm32f1xx_hal.h"

/* Macro commonly used with STM32F4 StdPeriph Library */
#ifndef USE_RTOS
#define USE_RTOS  0
#endif

#endif /* STM32F4XX_CONF_COMPAT_H */
