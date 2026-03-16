/**
  ******************************************************************************
  * @file    mc_stm_types.h
  * @brief   STM32F1xx HAL type bridge for MCSDK 6.4.1 compatibility
  *          Replaces G4xx LL driver includes with F1xx HAL.
  ******************************************************************************
  */
#ifndef MC_STM_TYPES_H
#define MC_STM_TYPES_H

#include "stm32f1xx_hal.h"

/* Number of motors */
#define NBR_OF_MOTORS  2

/* Speed unit definitions */
#define U_RPM   60
#define U_01HZ  10
#define SPEED_UNIT  U_01HZ

#define RPM_2_SPEED_UNIT(rpm)   ((int16_t)(((rpm) * SPEED_UNIT) / U_RPM))
#define SPEED_UNIT_2_RPM(speed) ((int16_t)(((speed) * U_RPM) / SPEED_UNIT))

#endif /* MC_STM_TYPES_H */
