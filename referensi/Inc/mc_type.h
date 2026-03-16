/**
  ******************************************************************************
  * @file    mc_type.h
  * @brief   Motor Control SDK global types — adapted for STM32F1xx hoverboard
  *          Based on MCSDK v6.4.1 mc_type.h (no CORDIC)
  ******************************************************************************
  */
#ifndef MC_TYPE_H
#define MC_TYPE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_stm_types.h"

typedef int8_t  char_t;

/* Motor identification */
#define M1      ((uint8_t)0x0)
#define M2      ((uint8_t)0x1)
#define M_NONE  ((uint8_t)0xFF)

/* Fault codes */
#define MC_NO_ERROR   ((uint16_t)0x0000)
#define MC_NO_FAULTS  ((uint16_t)0x0000)
#define MC_DURATION   ((uint16_t)0x0001)
#define MC_OVER_VOLT  ((uint16_t)0x0002)
#define MC_UNDER_VOLT ((uint16_t)0x0004)
#define MC_OVER_TEMP  ((uint16_t)0x0008)
#define MC_START_UP   ((uint16_t)0x0010)
#define MC_SPEED_FDBK ((uint16_t)0x0020)
#define MC_OVER_CURR  ((uint16_t)0x0040)
#define MC_SW_ERROR   ((uint16_t)0x0080)
#define MC_DP_FAULT   ((uint16_t)0x0400)

/* Q-D components (rotor reference frame) */
typedef struct {
    int16_t q;
    int16_t d;
} qd_t;

/* Alpha-Beta components (stator reference frame) */
typedef struct {
    int16_t alpha;
    int16_t beta;
} alphabeta_t;

/* A-B components (phase currents) */
typedef struct {
    int16_t a;
    int16_t b;
} ab_t;

/* Sensor type */
typedef enum {
    REAL_SENSOR,
    VIRTUAL_SENSOR
} SensorType_t;

/* Motor control mode */
typedef enum {
    MCM_OBSERVING_MODE = 0,
    MCM_OPEN_LOOP_VOLTAGE_MODE,
    MCM_OPEN_LOOP_CURRENT_MODE,
    MCM_SPEED_MODE,
    MCM_TORQUE_MODE,
    MCM_POSITION_MODE,
    MCM_MODE_NUM,
    MCM_DUTY_MODE
} MC_ControlMode_t;

/* Current reference source */
typedef enum {
    INTERNAL,
    EXTERNAL
} CurrRefSource_t;

/* FOC Variables */
typedef struct {
    ab_t          Iab;
    alphabeta_t   Ialphabeta;
    qd_t          Iqd;
    qd_t          Iqdref;
    int16_t       UserIdref;
    qd_t          Vqd;
    alphabeta_t   Valphabeta;
    int16_t       hTeref;
    int16_t       hElAngle;
    uint16_t      hCodeError;
    CurrRefSource_t bDriveInput;
} FOCVars_t;

/* Low side output configuration */
typedef enum {
    LS_DISABLED  = 0x0U,
    LS_PWM_TIMER = 0x1U,
    ES_GPIO      = 0x2U
} LowSideOutputsFunction_t;

/* SVM Sectors */
#define SECTOR_1  0u
#define SECTOR_2  1u
#define SECTOR_3  2u
#define SECTOR_4  3u
#define SECTOR_5  4u
#define SECTOR_6  5u

/* sqrt(3) in Q15 */
#define SQRT3FACTOR ((int32_t)0xDDB4)

/* Speed-torque control mode */
typedef enum {
    STC_TORQUE_MODE,
    STC_SPEED_MODE
} STC_Modality_t;

/* Polarization offsets */
typedef struct {
    int32_t phaseAOffset;
    int32_t phaseBOffset;
    int32_t phaseCOffset;
} PolarizationOffsets_t;

/* Macros */
#define MC_NULL ((void *)(0x0))

#ifdef __cplusplus
}
#endif

#endif /* MC_TYPE_H */
