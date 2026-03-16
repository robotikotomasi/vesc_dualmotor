/**
  ******************************************************************************
  * @file    mc_tasks.h
  * @brief   Motor control — FOC dual motor hoverboard ESC
  *          MCSDK 6.4.1 architecture, Hall sensor feedback, no CORDIC
  ******************************************************************************
  */

#ifndef MC_TASKS_H
#define MC_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "mc_type.h"
#include "hw_config.h"

/* ---- Motor IDs ---- */
#define M_RIGHT  0
#define M_LEFT   1

/* ---- Motor state machine ---- */
typedef enum {
    MC_STATE_IDLE = 0,
    MC_STATE_OFFSET_CALIB,
    MC_STATE_ALIGNMENT,
    MC_STATE_RUN,
    MC_STATE_STOP,
    MC_STATE_FAULT_NOW,
    MC_STATE_FAULT_OVER
} MC_State_t;

/* ---- Control mode ---- */
typedef enum {
    MC_CTRL_DUTY = 0,
    MC_CTRL_CURRENT,
    MC_CTRL_CURRENT_BRAKE,
    MC_CTRL_SPEED,
    MC_CTRL_POSITION,
    MC_CTRL_HANDBRAKE,
    MC_CTRL_OPEN_LOOP,
    MC_CTRL_BLDC
} MC_CtrlMode_t;

/* ---- Motor Type ---- */
typedef enum {
    MC_MOTOR_FOC  = 0,
    MC_MOTOR_BLDC = 1
} MC_MotorType_t;

/* ---- Motor Detection State ---- */
typedef enum {
    DETECT_IDLE = 0,
    DETECT_R,
    DETECT_L,
    DETECT_FLUX,
    DETECT_HALL,
    DETECT_DONE
} MC_DetectState_t;

/* ---- Motor Detection Result ---- */
typedef struct {
    float resistance;
    float inductance;
    float flux_linkage;
    uint8_t hall_table[8];
    bool  valid;
} MC_DetectResult_t;

/* ---- PID controller ---- */
typedef struct {
    int16_t  hKpGain;
    int16_t  hKiGain;
    int16_t  hKdGain;
    uint16_t hKpDivisorPOW2;
    uint16_t hKiDivisorPOW2;
    uint16_t hKdDivisorPOW2;
    int32_t  wIntegralTerm;
    int32_t  wUpperIntegralLimit;
    int32_t  wLowerIntegralLimit;
    int16_t  hUpperOutputLimit;
    int16_t  hLowerOutputLimit;
    int32_t  wPrevProcessVarError;
} PID_Handle_t;

/* ---- Hall sensor state ---- */
typedef struct {
    uint8_t  state;                 /* Raw 3-bit Hall state */
    uint8_t  prev_state;
    uint16_t elAngle;               /* Electrical angle (Q16: 0-65535 = 0-360°) */
    uint16_t elAngleBase;           /* Base angle at last Hall transition */
    int16_t  elSpeedDpp;            /* Electrical speed in dpp (digit per period) */
    int32_t  measuredSpeed;         /* Speed in 0.1 Hz */
    int32_t  avgSpeed;              /* Filtered speed */
    uint32_t lastCapture;           /* Timer capture at last Hall change */
    uint32_t period;                /* Period between Hall transitions */
    int8_t   direction;             /* +1 or -1 */
    bool     speedValid;
    int32_t  tachometer;           /* Signed tacho (+1/-1 per transition) */
    int32_t  tachometerAbs;        /* Absolute tacho */
} Hall_Handle_t;

/* ---- Flux Weakening ---- */
typedef struct {
    PID_Handle_t pid;
    int16_t      IdFW;
    int32_t      vqdAmplFilt;
    int16_t      IdRefOffset;
} FW_Handle_t;

/* ---- Motor data structure ---- */
typedef struct {
    MC_State_t   state;
    MC_CtrlMode_t ctrlMode;
    bool         enable;
    uint16_t     faultCode;

    /* FOC variables */
    ab_t          Iab;
    alphabeta_t   Ialphabeta;
    qd_t          Iqd;
    qd_t          Iqdref;
    qd_t          Vqd;
    alphabeta_t   Valphabeta;
    int16_t       hElAngle;
    uint8_t       sector;

    /* PID controllers */
    PID_Handle_t  pidIq;
    PID_Handle_t  pidId;
    PID_Handle_t  pidSpeed;

    /* Hall sensor */
    Hall_Handle_t hall;

    /* Flux Weakening */
    FW_Handle_t   fw;

    /* Setpoints */
    float    dutyCycleRef;          /* -1.0 to 1.0 */
    float    currentRef;            /* Amps */
    float    speedRef;              /* ERPM */
    float    positionRef;           /* Degrees */
    float    handbrakeRef;          /* Amps */

    /* Ramp Generator (VESC-style) */
    float    rampCurrentOut;
    float    rampSpeedOut;

    /* BEMF Decoupling (VESC-style) */
    int16_t  bemfDq;
    int16_t  bemfQq;

    /* Measurements (float for VESC interface) */
    float    currentMotor;          /* Amps RMS */
    float    currentInput;          /* DC link Amps */
    float    dutyNow;               /* Current duty cycle */
    float    speedNow;              /* Current ERPM */
    float    idNow;
    float    iqNow;
    float    vdNow;
    float    vqNow;

    /* Energy Tracking (VESC-style) */
    float    ampHours;
    float    ampHoursCharged;
    float    wattHours;
    float    wattHoursCharged;

    /* Limiting Factors */
    float    tempCurrentScale;
    float    batCurrentScale;

    /* ADC offsets */
    int32_t  phaseAOffset;
    int32_t  phaseBOffset;
    uint16_t offsetCalibCnt;

    /* Open loop */
    uint16_t openLoopAngle;
    int16_t  openLoopVoltage;

    /* Position control speed limit (ERPM) — set per command, defaults to POSITION_MAX_ERPM */
    float    positionMaxErpm;

    /* BLDC 6-step trapezoidal */
    MC_MotorType_t motorType;
    float    bldcDuty;
    uint8_t  bldcStep;

    /* Motor detection */
    MC_DetectState_t detectState;
    MC_DetectResult_t detectResult;
    uint16_t detectCounter;
    float    detectAccum;
    MC_CtrlMode_t detectPrevMode;   /* Saved mode before detection */
    bool     detectPrevEnable;      /* Saved enable state before detection */
} MotorControl_t;

/* ---- ADC buffer for DMA (dual simultaneous conversion) ---- */
typedef struct {
    uint32_t data[5];   /* 5 pairs of ADC1+ADC2 readings (32-bit packed) */
} ADC_Buffer_t;

/* ---- Global motor data ---- */
extern volatile MotorControl_t motor[NBR_OF_MOTORS];
extern volatile int16_t  batVoltageRaw;
extern volatile int16_t  boardTempRaw;
extern volatile float    batVoltage;
extern volatile float    boardTemp;
extern volatile ADC_Buffer_t adc_buffer;
extern volatile uint16_t halfPwmPeriod;

/* ---- Function prototypes ---- */
void MX_MotorControl_Init(void);
void MC_HighFrequencyTask(void);
void MC_MediumFrequencyTask(void);
void MC_SafetyTask(void);

/* Motor control API */
void MC_SetDuty(uint8_t motor_id, float duty);
void MC_SetCurrent(uint8_t motor_id, float current_a);
void MC_SetCurrentBrake(uint8_t motor_id, float current_a);
void MC_SetSpeed(uint8_t motor_id, float erpm);
void MC_SetHandbrake(uint8_t motor_id, float current_a);
void MC_SetPosition(uint8_t motor_id, float degrees);
void MC_EnableMotor(uint8_t motor_id, bool enable);
void MC_StopMotor(uint8_t motor_id);
void MC_FaultAck(uint8_t motor_id);

/* Telemetry/config helpers shared with the VESC protocol layer */
float MC_GetRotorPositionDeg(uint8_t motor_id);
float MC_GetElectricalRpm(uint8_t motor_id);
float MC_GetMotorCurrentLimit(void);
float MC_GetInputCurrentLimit(void);
float MC_GetBrakeCurrentLimit(void);
float MC_GetMotorResistance(void);
float MC_GetMotorInductance(void);
float MC_GetMotorFluxLinkage(void);

/* FOC math */
void MC_Clarke(int16_t ia, int16_t ib, alphabeta_t *out);
void MC_Park(alphabeta_t ialphabeta, int16_t angle, qd_t *out);
void MC_RevPark(qd_t vqd, int16_t angle, alphabeta_t *out);
void MC_SVPWM(alphabeta_t valphabeta, uint16_t half_period,
              uint16_t *dutyA, uint16_t *dutyB, uint16_t *dutyC, uint8_t *sector);
int16_t MC_PI_Controller(PID_Handle_t *pid, int16_t error);
void MC_CircleLimitation(qd_t *vqd, uint16_t maxModule);

/* Trig functions (no CORDIC — lookup table) */
int16_t MC_Sin(int16_t angle);
int16_t MC_Cos(int16_t angle);

/* Tachometer API (VESC-style) */
int32_t MC_GetTachometer(uint8_t motor_id);
int32_t MC_GetTachometerAbs(uint8_t motor_id);
void    MC_ResetTachometer(uint8_t motor_id);

/* Energy API (VESC-style) */
float   MC_GetAmpHours(uint8_t motor_id);
float   MC_GetAmpHoursCharged(uint8_t motor_id);
float   MC_GetWattHours(uint8_t motor_id);
float   MC_GetWattHoursCharged(uint8_t motor_id);
void    MC_ResetEnergy(uint8_t motor_id);

/* Motor detection (VESC-style) */
void    MC_DetectMotorRL(uint8_t motor_id);
void    MC_DetectHallFOC(uint8_t motor_id);
bool    MC_IsDetectDone(uint8_t motor_id);
MC_DetectResult_t MC_GetDetectResult(uint8_t motor_id);

/* Motor type selection */
void    MC_SetMotorType(uint8_t motor_id, MC_MotorType_t type);
MC_MotorType_t MC_GetMotorType(uint8_t motor_id);

/* BLDC mode */
void    MC_SetBldcDuty(uint8_t motor_id, float duty);

#ifdef __cplusplus
}
#endif

#endif /* MC_TASKS_H */
