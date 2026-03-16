/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @brief   FOC motor control for dual motor hoverboard ESC
  *          MCSDK 6.4.1 architecture — Hall sensor feedback, no CORDIC
  *          Clarke/Park/SVPWM transforms, PI controllers, dual TIM1+TIM8
  ******************************************************************************
  */

#include "mc_tasks.h"
#include "hw_config.h"
#include "flash_helper.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static void MC_DetectStep(volatile MotorControl_t *m, uint8_t motor_id);

static float MC_NormalizeDegrees(float degrees)
{
    while (degrees >= 180.0f) {
        degrees -= 360.0f;
    }
    while (degrees < -180.0f) {
        degrees += 360.0f;
    }
    return degrees;
}

static float MC_ClampFloat(float value, float low, float high)
{
    if (value > high) {
        return high;
    }
    if (value < low) {
        return low;
    }
    return value;
}

float MC_GetRotorPositionDeg(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) {
        return 0.0f;
    }

    return ((float)(uint16_t)motor[motor_id].hElAngle * 360.0f / 65536.0f) / (float)POLE_PAIR_NUM;
}

float MC_GetElectricalRpm(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) {
        return 0.0f;
    }

    return motor[motor_id].speedNow;
}

float MC_GetMotorCurrentLimit(void)
{
    return MAX_MOTOR_CURRENT_A;
}

float MC_GetInputCurrentLimit(void)
{
    return MAX_INPUT_CURRENT_A;
}

float MC_GetBrakeCurrentLimit(void)
{
    return MAX_BRAKE_CURRENT_A;
}

float MC_GetMotorResistance(void)
{
    return FOC_MOTOR_R_OHM;
}

float MC_GetMotorInductance(void)
{
    return FOC_MOTOR_L_H;
}

float MC_GetMotorFluxLinkage(void)
{
    return FOC_MOTOR_FLUX_LINKAGE;
}

/* ---- Global data ---- */
volatile MotorControl_t motor[NBR_OF_MOTORS];
volatile int16_t  batVoltageRaw = 0;
volatile int16_t  boardTempRaw  = 0;
volatile float    batVoltage    = 0.0f;
volatile float    boardTemp     = 0.0f;
volatile ADC_Buffer_t adc_buffer;
volatile uint16_t halfPwmPeriod;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

/* ---- Sin lookup table (256 entries, Q15, full cycle) ---- */
static const int16_t sinTable[256] = {
        0,   804,  1608,  2410,  3212,  4011,  4808,  5602,
     6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285,
    32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683,
    27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
    23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868,
    18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279,
    12539, 11793, 11039, 10278,  9512,  8739,  7962,  7179,
     6393,  5602,  4808,  4011,  3212,  2410,  1608,   804,
        0,  -804, -1608, -2410, -3212, -4011, -4808, -5602,
    -6393, -7179, -7962, -8739, -9512,-10278,-11039,-11793,
   -12539,-13279,-14010,-14732,-15446,-16151,-16846,-17530,
   -18204,-18868,-19519,-20159,-20787,-21403,-22005,-22594,
   -23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,
   -27245,-27683,-28105,-28510,-28898,-29268,-29621,-29956,
   -30273,-30571,-30852,-31113,-31356,-31580,-31785,-31971,
   -32137,-32285,-32412,-32521,-32609,-32678,-32728,-32757,
   -32768,-32757,-32728,-32678,-32609,-32521,-32412,-32285,
   -32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,
   -30273,-29956,-29621,-29268,-28898,-28510,-28105,-27683,
   -27245,-26790,-26319,-25832,-25329,-24811,-24279,-23731,
   -23170,-22594,-22005,-21403,-20787,-20159,-19519,-18868,
   -18204,-17530,-16846,-16151,-15446,-14732,-14010,-13279,
   -12539,-11793,-11039,-10278, -9512, -8739, -7962, -7179,
    -6393, -5602, -4808, -4011, -3212, -2410, -1608,  -804
};

/* Hall sensor → electrical angle mapping (Q16, 0-65535 = 0-360°)
   Index = 3-bit Hall state. 0 and 7 are invalid.
   Angle includes HALL_PHASE_SHIFT offset for FOC alignment.
   Default for hoverboard motor, 120° placement, 90° phase shift. */
static const uint16_t hallAngleTable[8] = {
    0,          /* state 0: invalid */
    27307,      /* state 1 (001):  60° + 90° = 150° */
    60075,      /* state 2 (010): 240° + 90° = 330° */
    16384,      /* state 3 (011):   0° + 90° =  90° */
    49152,      /* state 4 (100): 180° + 90° = 270° */
    38229,      /* state 5 (101): 120° + 90° = 210° */
     5461,      /* state 6 (110): 300° + 90° =  30° */
    0           /* state 7: invalid */
};

/* Hall transition valid direction table: next_state[prev_state] for forward */
static const uint8_t hallNextFwd[8] = { 0, 3, 6, 2, 5, 1, 4, 0 };

/* Speed filter coefficient (IIR low-pass, shift = 4 → α ≈ 1/16) */
#define SPEED_FILTER_SHIFT  4

/* Offset calibration samples */
#define OFFSET_CALIB_COUNT  2000

/*============================================================================
 * Trigonometric Functions (no CORDIC — lookup table with interpolation)
 *============================================================================*/
int16_t MC_Sin(int16_t angle)
{
    /* angle is in Q16: -32768..32767 maps to -π..+π (or 0..65535 for 0..2π unsigned) */
    uint16_t ua = (uint16_t)angle;
    uint8_t  idx = (uint8_t)(ua >> 8);
    uint8_t  frac = (uint8_t)(ua & 0xFF);

    int16_t s0 = sinTable[idx];
    int16_t s1 = sinTable[(uint8_t)(idx + 1)];

    return (int16_t)(s0 + (((int32_t)(s1 - s0) * frac) >> 8));
}

int16_t MC_Cos(int16_t angle)
{
    return MC_Sin((int16_t)(angle + 16384));  /* cos(x) = sin(x + 90°) */
}

/*============================================================================
 * Clarke Transform: (Ia, Ib) → (Iα, Iβ)
 *============================================================================*/
void MC_Clarke(int16_t ia, int16_t ib, alphabeta_t *out)
{
    out->alpha = ia;
    out->beta  = (int16_t)(((int32_t)ia + 2 * (int32_t)ib) * 18919 >> 15);
    /* 18919 = 1/√3 * 32768 */
}

/*============================================================================
 * Park Transform: (Iα, Iβ, θ) → (Id, Iq)
 *============================================================================*/
void MC_Park(alphabeta_t iab, int16_t angle, qd_t *out)
{
    int16_t sin_val = MC_Sin(angle);
    int16_t cos_val = MC_Cos(angle);

    out->d = (int16_t)(((int32_t)iab.alpha * cos_val + (int32_t)iab.beta * sin_val) >> 15);
    out->q = (int16_t)((-(int32_t)iab.alpha * sin_val + (int32_t)iab.beta * cos_val) >> 15);
}

/*============================================================================
 * Reverse Park Transform: (Vd, Vq, θ) → (Vα, Vβ)
 *============================================================================*/
void MC_RevPark(qd_t vqd, int16_t angle, alphabeta_t *out)
{
    int16_t sin_val = MC_Sin(angle);
    int16_t cos_val = MC_Cos(angle);

    out->alpha = (int16_t)(((int32_t)vqd.d * cos_val - (int32_t)vqd.q * sin_val) >> 15);
    out->beta  = (int16_t)(((int32_t)vqd.d * sin_val + (int32_t)vqd.q * cos_val) >> 15);
}

/*============================================================================
 * PI Controller (MCSDK compatible)
 *============================================================================*/
int16_t MC_PI_Controller(PID_Handle_t *pid, int16_t error)
{
    /* Proportional term */
    int32_t wProportional = (int32_t)pid->hKpGain * (int32_t)error;

    /* Integral term */
    int32_t wIntegralTerm = pid->wIntegralTerm + (int32_t)pid->hKiGain * (int32_t)error;

    /* Integral clamping */
    if (wIntegralTerm > pid->wUpperIntegralLimit) {
        wIntegralTerm = pid->wUpperIntegralLimit;
    } else if (wIntegralTerm < pid->wLowerIntegralLimit) {
        wIntegralTerm = pid->wLowerIntegralLimit;
    }
    pid->wIntegralTerm = wIntegralTerm;

    /* Derivative term */
    int32_t wDerivative = (int32_t)pid->hKdGain * (int32_t)(error - pid->wPrevProcessVarError);
    pid->wPrevProcessVarError = error;

    /* Output */
    int32_t wOutput = (wProportional >> pid->hKpDivisorPOW2)
                    + (wIntegralTerm >> pid->hKiDivisorPOW2)
                    + (wDerivative >> pid->hKdDivisorPOW2);

    /* Output saturation */
    if (wOutput > (int32_t)pid->hUpperOutputLimit) {
        wOutput = pid->hUpperOutputLimit;
    } else if (wOutput < (int32_t)pid->hLowerOutputLimit) {
        wOutput = pid->hLowerOutputLimit;
    }

    return (int16_t)wOutput;
}

/*============================================================================
 * Circle Limitation: Vqd must satisfy √(Vq² + Vd²) ≤ maxModule
 *============================================================================*/
void MC_CircleLimitation(qd_t *vqd, uint16_t maxModule)
{
    uint32_t sq = (uint32_t)((int32_t)vqd->q * vqd->q)
                + (uint32_t)((int32_t)vqd->d * vqd->d);
    uint32_t maxSq = (uint32_t)maxModule * maxModule;

    if (sq > maxSq) {
        /* Scale down proportionally */
        /* Approximate: find scale = maxModule / sqrt(sq) using Newton's method */
        uint32_t root = 1;
        for (int i = 0; i < 8; i++) {
            root = (root + sq / root) / 2;
        }
        if (root == 0) root = 1;
        vqd->q = (int16_t)((int32_t)vqd->q * maxModule / (int32_t)root);
        vqd->d = (int16_t)((int32_t)vqd->d * maxModule / (int32_t)root);
    }
}

/*============================================================================
 * Space Vector PWM: (Vα, Vβ) → (dutyA, dutyB, dutyC)
 * MCSDK 6.4.1 SVPWM algorithm (no CORDIC)
 *============================================================================*/
void MC_SVPWM(alphabeta_t vab, uint16_t half_period,
              uint16_t *dutyA, uint16_t *dutyB, uint16_t *dutyC, uint8_t *sector)
{
    int32_t wUAlpha = (int32_t)vab.alpha * SQRT3FACTOR;
    int32_t wUBeta  = -(int32_t)vab.beta * (int32_t)32768;

    int32_t wX = wUBeta;
    int32_t wY = (wUBeta + wUAlpha);
    int32_t wZ = (wUBeta - wUAlpha);

    /* Sector determination */
    uint8_t sec;
    int32_t tA, tB, tC;

    if (wY < 0) {
        if (wZ < 0) {
            sec = SECTOR_5;
            tA = (int32_t)half_period + ((-wY + wX) >> 17);
            tB = (int32_t)half_period + (( wY + wZ) >> 17);
            tC = (int32_t)half_period + ((-wX - wZ) >> 17);
        } else {
            if (wX <= 0) {
                sec = SECTOR_4;
                tA = (int32_t)half_period + ((-wX + wZ) >> 17);
                tB = (int32_t)half_period + (( wY + wZ) >> 17);
                tC = (int32_t)half_period + ((-wX - wY) >> 17);
            } else {
                sec = SECTOR_3;
                tA = (int32_t)half_period + (( wY + wZ) >> 17);
                tB = (int32_t)half_period + ((-wX - wY) >> 17);
                tC = (int32_t)half_period + (( wX - wZ) >> 17);
            }
        }
    } else {
        if (wZ >= 0) {
            sec = SECTOR_2;
            tA = (int32_t)half_period + (( wY - wZ) >> 17);
            tB = (int32_t)half_period + ((-wX - wY) >> 17);
            tC = (int32_t)half_period + (( wX + wZ) >> 17);
        } else {
            if (wX >= 0) {
                sec = SECTOR_1;
                tA = (int32_t)half_period + ((-wY + wX) >> 17);
                tB = (int32_t)half_period + (( wY + wZ) >> 17);
                tC = (int32_t)half_period + ((-wX - wZ) >> 17);
            } else {
                sec = SECTOR_6;
                tA = (int32_t)half_period + ((-wX + wZ) >> 17);
                tB = (int32_t)half_period + (( wY + wZ) >> 17);
                tC = (int32_t)half_period + ((-wX - wY) >> 17);
            }
        }
    }

    /* Clamp duties */
    uint16_t hp = half_period;
    *dutyA = (uint16_t)CLAMP(tA, 0, hp);
    *dutyB = (uint16_t)CLAMP(tB, 0, hp);
    *dutyC = (uint16_t)CLAMP(tC, 0, hp);
    *sector = sec;
}

/*============================================================================
 * Hall Sensor Reading
 *============================================================================*/
static uint8_t MC_ReadHallRight(void)
{
    uint8_t h = 0;
    if (HAL_GPIO_ReadPin(RIGHT_HALL_U_PORT, RIGHT_HALL_U_PIN)) h |= 1;
    if (HAL_GPIO_ReadPin(RIGHT_HALL_V_PORT, RIGHT_HALL_V_PIN)) h |= 2;
    if (HAL_GPIO_ReadPin(RIGHT_HALL_W_PORT, RIGHT_HALL_W_PIN)) h |= 4;
    return h;
}

static uint8_t MC_ReadHallLeft(void)
{
    uint8_t h = 0;
    if (HAL_GPIO_ReadPin(LEFT_HALL_U_PORT, LEFT_HALL_U_PIN)) h |= 1;
    if (HAL_GPIO_ReadPin(LEFT_HALL_V_PORT, LEFT_HALL_V_PIN)) h |= 2;
    if (HAL_GPIO_ReadPin(LEFT_HALL_W_PORT, LEFT_HALL_W_PIN)) h |= 4;
    return h;
}

/*============================================================================
 * Update Hall angle with interpolation
 *============================================================================*/
static void MC_UpdateHall(volatile MotorControl_t *m, uint8_t newState)
{
    Hall_Handle_t *h = (Hall_Handle_t *)&m->hall;

    if (newState == 0 || newState == 7) return;

    if (newState != h->state) {
        h->prev_state = h->state;
        h->state = newState;
        h->elAngleBase = hallAngleTable[newState];

        /* Tachometer: count Hall transitions (VESC-style) */
        if (hallNextFwd[h->prev_state] == newState) {
            h->direction = 1;
            h->tachometer++;
        } else {
            h->direction = -1;
            h->tachometer--;
        }
        h->tachometerAbs++;

        /* Speed measurement: measure period between transitions */
        uint32_t now = DWT->CYCCNT;  /* Use DWT cycle counter */
        if (h->lastCapture != 0) {
            uint32_t delta = now - h->lastCapture;
            if (delta > 0 && delta < (SYSCLK_FREQ / 2)) {
                h->period = delta;
                /* Speed in 0.1 Hz:
                   6 transitions per electrical revolution
                   speed_01Hz = SYSCLK_FREQ / (period * 6) * 10 / POLE_PAIR_NUM */
                int32_t spd = (int32_t)((uint64_t)SYSCLK_FREQ * 10 /
                              ((uint64_t)delta * 6 * POLE_PAIR_NUM));
                h->measuredSpeed = spd * h->direction;
                h->speedValid = true;
            }
        }
        h->lastCapture = now;
    }

    /* Interpolate angle between Hall transitions */
    if (h->period > 0 && h->speedValid) {
        uint32_t elapsed = DWT->CYCCNT - h->lastCapture;
        if (elapsed > h->period * 2) elapsed = h->period * 2;
        /* Each Hall transition = 60° electrical = 10923 in Q16 */
        int32_t angleDelta = (int32_t)((uint64_t)elapsed * 10923 / h->period);
        h->elAngle = h->elAngleBase + (int16_t)(angleDelta * h->direction);
    } else {
        h->elAngle = hallAngleTable[h->state & 0x07];
    }

    /* Low-pass filter speed */
    h->avgSpeed += (h->measuredSpeed - h->avgSpeed) >> SPEED_FILTER_SHIFT;
}

/*============================================================================
 * Read Phase Currents from ADC buffer
 *============================================================================*/
static void MC_ReadCurrentsRight(volatile MotorControl_t *m)
{
    /* ADC1 dual-mode data register: upper 16 bits = ADC2, lower = ADC1
       Rank 1: ADC1=CH11(Right DC), ADC2=CH10(Left DC)
       Rank 2: ADC1=CH0(Left PhA),  ADC2=CH13(Left PhB)
       Rank 3: ADC1=CH14(Right PhA),ADC2=CH15(Right PhB)  */
    int16_t phA = (int16_t)(adc_buffer.data[2] & 0xFFFF) - (int16_t)(m->phaseAOffset);
    int16_t phB = (int16_t)(adc_buffer.data[2] >> 16) - (int16_t)(m->phaseBOffset);

    m->Iab.a = phA;
    m->Iab.b = phB;
}

static void MC_ReadCurrentsLeft(volatile MotorControl_t *m)
{
    int16_t phA = (int16_t)(adc_buffer.data[1] & 0xFFFF) - (int16_t)(m->phaseAOffset);
    int16_t phB = (int16_t)(adc_buffer.data[1] >> 16) - (int16_t)(m->phaseBOffset);

    m->Iab.a = phA;
    m->Iab.b = phB;
}

/*============================================================================
 * Write PWM to Timer
 *============================================================================*/
static void MC_WritePWM_Right(uint16_t a, uint16_t b, uint16_t c, bool enable)
{
    if (enable) {
        TIM1->CCR1 = a;
        TIM1->CCR2 = b;
        TIM1->CCR3 = c;
        TIM1->BDTR |= TIM_BDTR_MOE;
    } else {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
}

static void MC_WritePWM_Left(uint16_t a, uint16_t b, uint16_t c, bool enable)
{
    if (enable) {
        TIM8->CCR1 = a;
        TIM8->CCR2 = b;
        TIM8->CCR3 = c;
        TIM8->BDTR |= TIM_BDTR_MOE;
    } else {
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
        TIM8->BDTR &= ~TIM_BDTR_MOE;
    }
}

/*============================================================================
 * Initialize PID Controller
 *============================================================================*/
static void MC_PID_Init(PID_Handle_t *pid, int16_t kp, int16_t ki, int16_t kd,
                        uint16_t kpPow2, uint16_t kiPow2, uint16_t kdPow2,
                        int16_t outMax)
{
    pid->hKpGain = kp;
    pid->hKiGain = ki;
    pid->hKdGain = kd;
    pid->hKpDivisorPOW2 = kpPow2;
    pid->hKiDivisorPOW2 = kiPow2;
    pid->hKdDivisorPOW2 = kdPow2;
    pid->wIntegralTerm = 0;
    pid->wUpperIntegralLimit = (int32_t)outMax << kiPow2;
    pid->wLowerIntegralLimit = -pid->wUpperIntegralLimit;
    pid->hUpperOutputLimit = outMax;
    pid->hLowerOutputLimit = -outMax;
    pid->wPrevProcessVarError = 0;
}

/*============================================================================
 * BLDC Trapezoidal 6-Step Commutation (VESC mcpwm-style)
 * Uses Hall sensor state to determine commutation step.
 * duty is -1.0 to +1.0
 *============================================================================*/
static void MC_BLDC_Run(volatile MotorControl_t *m, uint8_t motor_id)
{
    uint16_t hp = halfPwmPeriod;

    /* Read Hall sensors */
    uint8_t hallState;
    if (motor_id == M_RIGHT) {
        hallState = MC_ReadHallRight();
    } else {
        hallState = MC_ReadHallLeft();
    }
    MC_UpdateHall(m, hallState);
    m->hElAngle = (int16_t)m->hall.elAngle;

    /* Determine duty based on control mode */
    float duty = m->bldcDuty;
    switch (m->ctrlMode) {
    case MC_CTRL_BLDC:
        duty = m->bldcDuty;
        break;
    case MC_CTRL_DUTY:
        duty = m->dutyCycleRef;
        break;
    case MC_CTRL_SPEED: {
        float speedMeas = (float)m->hall.avgSpeed / 10.0f;
        float speedRef  = m->speedRef / (float)(POLE_PAIR_NUM * 60) * 10.0f;
        float speedErr  = speedRef - speedMeas;
        int16_t iqOut = MC_PI_Controller((PID_Handle_t *)&m->pidSpeed, (int16_t)speedErr);
        duty = (float)iqOut / (float)IQMAX;
        break;
    }
    default:
        duty = m->dutyCycleRef;
        break;
    }

    duty = MC_ClampFloat(duty, -BLDC_DUTY_MAX, BLDC_DUTY_MAX);

    /* 6-step commutation based on Hall state */
    uint16_t pwm_val = (uint16_t)(ABS(duty) * (float)hp);
    uint16_t dA = 0, dB = 0, dC = 0;

    /* Forward direction commutation table (120° spacing) */
    uint8_t step = hallState;
    if (duty < 0.0f) {
        /* Reverse: swap high/low phases */
        switch (step) {
        case 1: dA = 0;       dB = 0;       dC = pwm_val; m->bldcStep = 5; break;
        case 2: dA = 0;       dB = pwm_val; dC = 0;       m->bldcStep = 3; break;
        case 3: dA = 0;       dB = pwm_val; dC = 0;       m->bldcStep = 4; break;
        case 4: dA = pwm_val; dB = 0;       dC = 0;       m->bldcStep = 1; break;
        case 5: dA = pwm_val; dB = 0;       dC = 0;       m->bldcStep = 0; break;
        case 6: dA = 0;       dB = 0;       dC = pwm_val; m->bldcStep = 2; break;
        default: break;
        }
    } else {
        switch (step) {
        case 1: dA = pwm_val; dB = 0;       dC = 0;       m->bldcStep = 0; break;
        case 2: dA = 0;       dB = 0;       dC = pwm_val; m->bldcStep = 2; break;
        case 3: dA = 0;       dB = 0;       dC = pwm_val; m->bldcStep = 1; break;
        case 4: dA = 0;       dB = pwm_val; dC = 0;       m->bldcStep = 4; break;
        case 5: dA = 0;       dB = pwm_val; dC = 0;       m->bldcStep = 5; break;
        case 6: dA = pwm_val; dB = 0;       dC = 0;       m->bldcStep = 3; break;
        default: break;
        }
    }

    if (motor_id == M_RIGHT) {
        MC_WritePWM_Right(dA, dB, dC, m->enable);
    } else {
        MC_WritePWM_Left(dA, dB, dC, m->enable);
    }

    /* Update measurements */
    float cur_scale = (float)ADC_REFERENCE_VOLTAGE / 4096.0f / RSHUNT / AMPLIFICATION_GAIN;
    if (motor_id == M_RIGHT) {
        MC_ReadCurrentsRight(m);
    } else {
        MC_ReadCurrentsLeft(m);
    }
    m->currentMotor = (float)m->Iab.a * cur_scale / 32768.0f * NOMINAL_CURRENT / 1000.0f;
    m->currentInput = m->currentMotor * ABS(duty);
    m->dutyNow = duty;
    m->speedNow = (float)m->hall.avgSpeed * POLE_PAIR_NUM * 6.0f;
}

/*============================================================================
 * FOC Run for One Motor
 *============================================================================*/
static void MC_FOC_Run(volatile MotorControl_t *m, uint8_t motor_id)
{
    uint16_t hp = halfPwmPeriod;

    /* 1. Read phase currents */
    if (motor_id == M_RIGHT) {
        MC_ReadCurrentsRight(m);
    } else {
        MC_ReadCurrentsLeft(m);
    }

    /* 2. Read Hall sensors → electrical angle */
    uint8_t hallState;
    if (motor_id == M_RIGHT) {
        hallState = MC_ReadHallRight();
    } else {
        hallState = MC_ReadHallLeft();
    }
    MC_UpdateHall(m, hallState);
    m->hElAngle = (int16_t)m->hall.elAngle;

    /* 3. Clarke: (Ia, Ib) → (Iα, Iβ) */
    MC_Clarke(m->Iab.a, m->Iab.b, (alphabeta_t *)&m->Ialphabeta);

    /* 4. Park: (Iα, Iβ, θ) → (Id, Iq) */
    MC_Park(m->Ialphabeta, m->hElAngle, (qd_t *)&m->Iqd);

    /* 5. Determine current references based on control mode */
    switch (m->ctrlMode) {
    case MC_CTRL_DUTY: {
        /* Open-loop duty → Iq reference proportional to duty */
        int16_t iqRef = (int16_t)(m->dutyCycleRef * (float)INT16_MAX);
        m->Iqdref.q = iqRef;
        m->Iqdref.d = 0;
        break;
    }
    case MC_CTRL_CURRENT:
        m->Iqdref.q = (int16_t)(m->rampCurrentOut * 1000.0f / NOMINAL_CURRENT * INT16_MAX);
        m->Iqdref.d = 0;
        break;
    case MC_CTRL_CURRENT_BRAKE: {
        /* Regenerative braking: apply Iq opposing the direction of rotation.
           If spinning forward (direction > 0), apply negative Iq.
           If spinning backward (direction < 0), apply positive Iq. */
        float brakeMag = MC_ClampFloat(m->handbrakeRef,
                                       0.0f, MC_GetBrakeCurrentLimit());
        float brakeIq = (m->hall.direction >= 0) ? -brakeMag : brakeMag;
        m->Iqdref.q = (int16_t)(brakeIq * 1000.0f / NOMINAL_CURRENT * INT16_MAX);
        m->Iqdref.d = 0;
        break;
    }
    case MC_CTRL_SPEED: {
        /* Speed PI → Iq reference (ramp applied) */
        int16_t speedMeas = (int16_t)(m->hall.avgSpeed / 10); /* 0.1Hz to 1Hz */
        int16_t speedRef  = (int16_t)(m->rampSpeedOut / (float)(POLE_PAIR_NUM * 60) * 10.0f);
        int16_t speedErr  = speedRef - speedMeas;
        m->Iqdref.q = MC_PI_Controller((PID_Handle_t *)&m->pidSpeed, speedErr);
        m->Iqdref.d = 0;
        break;
    }
    case MC_CTRL_POSITION: {
        float positionNow = MC_GetRotorPositionDeg(motor_id);
        float positionErr = MC_NormalizeDegrees(m->positionRef - positionNow);
        float speedLimit  = (m->positionMaxErpm > 0.0f) ? m->positionMaxErpm
                                                         : (float)POSITION_MAX_ERPM;
        float speedCmd = MC_ClampFloat(positionErr * POSITION_KP_ERPM_PER_DEG,
                                       -speedLimit,
                                       speedLimit);
        int16_t speedMeas = (int16_t)(m->hall.avgSpeed / 10);
        int16_t speedRef = (int16_t)(speedCmd / (float)(POLE_PAIR_NUM * 60) * 10.0f);
        int16_t speedErr = speedRef - speedMeas;
        m->Iqdref.q = MC_PI_Controller((PID_Handle_t *)&m->pidSpeed, speedErr);
        m->Iqdref.d = 0;
        break;
    }
    case MC_CTRL_HANDBRAKE:
        m->Iqdref.q = 0;
        m->Iqdref.d = (int16_t)(m->handbrakeRef * 1000.0f / NOMINAL_CURRENT * INT16_MAX);
        break;
    case MC_CTRL_OPEN_LOOP:
        m->openLoopAngle += 100;  /* Increment angle for open-loop rotation */
        m->hElAngle = (int16_t)m->openLoopAngle;
        m->Iqdref.q = m->openLoopVoltage;
        m->Iqdref.d = 0;
        break;
    default:
        m->Iqdref.q = 0;
        m->Iqdref.d = 0;
        break;
    }

    /* 5b. Apply thermal/battery derating to current references */
    float scale = 1.0f;
    if (m->tempCurrentScale < scale) scale = m->tempCurrentScale;
    if (m->batCurrentScale < scale)  scale = m->batCurrentScale;
    if (scale < 1.0f) {
        m->Iqdref.q = (int16_t)((float)m->Iqdref.q * scale);
        m->Iqdref.d = (int16_t)((float)m->Iqdref.d * scale);
    }

    /* 6. PI current controllers: Id, Iq */
    int16_t iqErr = m->Iqdref.q - m->Iqd.q;
    int16_t idErr = m->Iqdref.d - m->Iqd.d;

    m->Vqd.q = MC_PI_Controller((PID_Handle_t *)&m->pidIq, iqErr);
    m->Vqd.d = MC_PI_Controller((PID_Handle_t *)&m->pidId, idErr);

    /* 6b. BEMF Decoupling (VESC-style cross-coupling compensation) */
    m->Vqd.q = (int16_t)CLAMP((int32_t)m->Vqd.q + (int32_t)m->bemfQq,
                               -hp, hp);
    m->Vqd.d = (int16_t)CLAMP((int32_t)m->Vqd.d + (int32_t)m->bemfDq,
                               -hp, hp);

    /* 6c. Flux Weakening PI (VESC/MCSDK-style)
       When |Vqd| exceeds FW_VOLTAGE_REF% of halfPwmPeriod,
       PI generates negative Id to reduce back-EMF */
    {
        int32_t vqd_ampl = (int32_t)m->Vqd.q * m->Vqd.q
                         + (int32_t)m->Vqd.d * m->Vqd.d;
        /* Approximate sqrt via Newton's method (fast) */
        uint32_t root = (vqd_ampl > 0) ? (uint32_t)vqd_ampl : 1;
        for (int i = 0; i < 8; i++) {
            root = (root + (uint32_t)vqd_ampl / root) / 2;
        }
        /* Low-pass filter the amplitude */
        m->fw.vqdAmplFilt += ((int32_t)root - m->fw.vqdAmplFilt) >> FW_VQD_LP_SHIFT;

        int16_t fwRef = (int16_t)((int32_t)hp * FW_VOLTAGE_REF / 1000);
        int16_t fwErr = fwRef - (int16_t)m->fw.vqdAmplFilt;
        /* When fwErr < 0 → voltage is too high → FW PI outputs negative Id */
        m->fw.IdFW = MC_PI_Controller((PID_Handle_t *)&m->fw.pid, fwErr);
        /* Clamp to negative only (demagnetizing current) */
        if (m->fw.IdFW > 0) m->fw.IdFW = 0;
        /* Apply FW Id contribution */
        m->Iqdref.d = (int16_t)CLAMP((int32_t)m->Iqdref.d + (int32_t)m->fw.IdFW,
                                      (int32_t)FW_MAX_DEMAGCURRENT, (int32_t)hp);
    }

    /* 7. Circle limitation */
    qd_t vqd_lim = { .q = m->Vqd.q, .d = m->Vqd.d };
    MC_CircleLimitation(&vqd_lim, (uint16_t)(hp * 95 / 100));
    m->Vqd.q = vqd_lim.q;
    m->Vqd.d = vqd_lim.d;

    /* 8. Reverse Park: (Vd, Vq, θ) → (Vα, Vβ) */
    MC_RevPark(vqd_lim, m->hElAngle, (alphabeta_t *)&m->Valphabeta);

    /* 9. SVPWM → duty cycles */
    uint16_t dA, dB, dC;
    MC_SVPWM(m->Valphabeta, hp, &dA, &dB, &dC, (uint8_t *)&m->sector);

    /* 10. Write PWM */
    if (motor_id == M_RIGHT) {
        MC_WritePWM_Right(dA, dB, dC, m->enable);
    } else {
        MC_WritePWM_Left(dA, dB, dC, m->enable);
    }

    /* Update measurements for VESC interface */
    float cur_scale = (float)ADC_REFERENCE_VOLTAGE / 4096.0f / RSHUNT / AMPLIFICATION_GAIN;
    m->currentMotor = (float)m->Iqd.q * cur_scale / 32768.0f * NOMINAL_CURRENT / 1000.0f;
    m->currentInput = (float)(int16_t)(adc_buffer.data[0] & 0xFFFF) * cur_scale;
    m->dutyNow = MC_ClampFloat((float)m->Vqd.q / (float)hp, -1.0f, 1.0f);
    m->speedNow = (float)m->hall.avgSpeed * POLE_PAIR_NUM * 6.0f;

    /* Update d/q current and voltage measurements for VESC telemetry.
     * Iqd values are in Q15 fixed-point (32768 = full scale = NOMINAL_CURRENT).
     * Vqd values are normalised to the half-period (hp) of the PWM carrier. */
    m->idNow = (float)m->Iqd.d / 32768.0f * NOMINAL_CURRENT / 1000.0f;
    m->iqNow = (float)m->Iqd.q / 32768.0f * NOMINAL_CURRENT / 1000.0f;
    m->vdNow = (float)m->Vqd.d / (float)hp * batVoltage;
    m->vqNow = (float)m->Vqd.q / (float)hp * batVoltage;
}

/*============================================================================
 * Offset Calibration
 *============================================================================*/
static void MC_OffsetCalib(volatile MotorControl_t *m, uint8_t motor_id)
{
    if (m->offsetCalibCnt < OFFSET_CALIB_COUNT) {
        if (motor_id == M_RIGHT) {
            m->phaseAOffset += (int16_t)(adc_buffer.data[2] & 0xFFFF);
            m->phaseBOffset += (int16_t)(adc_buffer.data[2] >> 16);
        } else {
            m->phaseAOffset += (int16_t)(adc_buffer.data[1] & 0xFFFF);
            m->phaseBOffset += (int16_t)(adc_buffer.data[1] >> 16);
        }
        m->offsetCalibCnt++;
    } else if (m->offsetCalibCnt == OFFSET_CALIB_COUNT) {
        m->phaseAOffset /= OFFSET_CALIB_COUNT;
        m->phaseBOffset /= OFFSET_CALIB_COUNT;
        m->state = MC_STATE_IDLE;
        m->offsetCalibCnt++;
    }
}

/*============================================================================
 * High Frequency Task — TIM1_UP ISR (16 kHz)
 *============================================================================*/
void MC_HighFrequencyTask(void)
{
    halfPwmPeriod = TIM1->ARR;

    for (uint8_t m = 0; m < NBR_OF_MOTORS; m++) {
        volatile MotorControl_t *mc = &motor[m];

        switch (mc->state) {
        case MC_STATE_OFFSET_CALIB:
            MC_OffsetCalib(mc, m);
            if (m == M_RIGHT) MC_WritePWM_Right(0, 0, 0, false);
            else              MC_WritePWM_Left(0, 0, 0, false);
            break;

        case MC_STATE_RUN:
            if (mc->motorType == MC_MOTOR_BLDC) {
                MC_BLDC_Run(mc, m);
            } else {
                MC_FOC_Run(mc, m);
            }
            break;

        case MC_STATE_IDLE:
        case MC_STATE_STOP:
        case MC_STATE_FAULT_NOW:
        case MC_STATE_FAULT_OVER:
        default:
            if (m == M_RIGHT) MC_WritePWM_Right(0, 0, 0, false);
            else              MC_WritePWM_Left(0, 0, 0, false);
            break;
        }
    }
}

/*============================================================================
 * Medium Frequency Task — FreeRTOS 1 kHz
 *============================================================================*/
void MC_MediumFrequencyTask(void)
{
    /* Battery voltage from ADC rank 4 */
    uint16_t vbat_adc = (uint16_t)(adc_buffer.data[3] & 0xFFFF);
    batVoltageRaw = (int16_t)vbat_adc;
    batVoltage = (float)vbat_adc * ADC_REFERENCE_VOLTAGE / 4096.0f / VBUS_PARTITIONING_FACTOR;

    /* Board temperature from internal sensor (ADC rank 5) */
    uint16_t temp_adc = (uint16_t)(adc_buffer.data[4] & 0xFFFF);
    boardTempRaw = (int16_t)temp_adc;
    boardTemp = ((V0_V - (float)temp_adc * ADC_REFERENCE_VOLTAGE / 4096.0f) / dV_dT) + (float)T0_C;

    /* ---- Per-motor 1 kHz updates ---- */
    for (uint8_t mi = 0; mi < NBR_OF_MOTORS; mi++) {
        volatile MotorControl_t *m = &motor[mi];

        /* Speed ramp (VESC-style: smoothly approach target speed) */
        if (m->ctrlMode == MC_CTRL_SPEED) {
            float rampStep = SPEED_RAMP_ERPMS_S / (float)SPEED_LOOP_FREQUENCY_HZ;
            if (m->rampSpeedOut < m->speedRef) {
                m->rampSpeedOut += rampStep;
                if (m->rampSpeedOut > m->speedRef) m->rampSpeedOut = m->speedRef;
            } else if (m->rampSpeedOut > m->speedRef) {
                m->rampSpeedOut -= rampStep;
                if (m->rampSpeedOut < m->speedRef) m->rampSpeedOut = m->speedRef;
            }
        } else {
            m->rampSpeedOut = m->speedRef;
        }

        /* Current ramp (VESC-style: smoothly approach target current) */
        if (m->ctrlMode == MC_CTRL_CURRENT) {
            float rampStep = CURRENT_RAMP_A_S / (float)SPEED_LOOP_FREQUENCY_HZ;
            if (m->rampCurrentOut < m->currentRef) {
                m->rampCurrentOut += rampStep;
                if (m->rampCurrentOut > m->currentRef) m->rampCurrentOut = m->currentRef;
            } else if (m->rampCurrentOut > m->currentRef) {
                m->rampCurrentOut -= rampStep;
                if (m->rampCurrentOut < m->currentRef) m->rampCurrentOut = m->currentRef;
            }
        } else {
            m->rampCurrentOut = m->currentRef;
        }

        /* BEMF decoupling computation (VESC-style)
           Vd_corr = -ω_el * L * Iq
           Vq_corr = +ω_el * L * Id + ω_el * Ψ
           Speed in 0.1 Hz → ω_el ≈ avgSpeed / 10 * 2π */
        {
            int32_t elSpeed = m->hall.avgSpeed;  /* in 0.1 Hz */
            int32_t iq = m->Iqd.q;
            int32_t id = m->Iqd.d;
            /* Scaled computation to avoid float in ISR */
            m->bemfDq = (int16_t)((-elSpeed * BEMF_L_SCALE * iq) >> 24);
            m->bemfQq = (int16_t)(( elSpeed * BEMF_L_SCALE * id
                                   + elSpeed * BEMF_PSI_SCALE) >> 20);
        }

        /* Energy tracking (VESC-style) — 1 kHz integration */
        {
            float cur = m->currentInput;
            float pwr = cur * batVoltage;
            if (cur > 0.0f) {
                m->ampHours += cur * AH_PER_MA_PER_MS * 1000.0f;
                m->wattHours += pwr * WH_PER_MW_PER_MS * 1000.0f;
            } else {
                m->ampHoursCharged += (-cur) * AH_PER_MA_PER_MS * 1000.0f;
                m->wattHoursCharged += (-pwr) * WH_PER_MW_PER_MS * 1000.0f;
            }
        }

        /* Thermal derating (VESC-style: linearly reduce current as temp rises) */
        if (boardTemp > TEMP_CUR_END_C) {
            m->tempCurrentScale = 0.0f;
        } else if (boardTemp > TEMP_CUR_START_C) {
            m->tempCurrentScale = 1.0f - (boardTemp - TEMP_CUR_START_C)
                                        / (TEMP_CUR_END_C - TEMP_CUR_START_C);
        } else {
            m->tempCurrentScale = 1.0f;
        }

        /* Battery voltage derating (VESC-style: taper current near cut-off) */
        if (batVoltage < BAT_CUT_END_V) {
            m->batCurrentScale = 0.0f;
        } else if (batVoltage < BAT_CUT_START_V) {
            m->batCurrentScale = (batVoltage - BAT_CUT_END_V)
                               / (BAT_CUT_START_V - BAT_CUT_END_V);
        } else {
            m->batCurrentScale = 1.0f;
        }

        /* Power limiting (VESC-style: scale back batCurrentScale when
           instantaneous input power exceeds configured power_max) */
        {
            float powerNow = (m->currentInput < 0.0f ? -m->currentInput : m->currentInput)
                             * batVoltage;
            mc_configuration *conf = flash_helper_get_config();
            if (conf->power_max > 0.0f && powerNow > conf->power_max) {
                float powerScale = conf->power_max / powerNow;
                if (powerScale < m->batCurrentScale) {
                    m->batCurrentScale = powerScale;
                }
            }
        }

        /* Update speed average (IIR filter) */
        m->hall.avgSpeed += (m->hall.measuredSpeed - m->hall.avgSpeed) / 4;

        /* Motor detection state machine (runs at 1 kHz) */
        if (m->detectState != DETECT_IDLE && m->detectState != DETECT_DONE) {
            MC_DetectStep(m, mi);
        }
    }
}

/*============================================================================
 * Safety Task — Voltage / Temperature protection (10 Hz)
 *============================================================================*/
void MC_SafetyTask(void)
{
    for (uint8_t m = 0; m < NBR_OF_MOTORS; m++) {
        volatile MotorControl_t *mc = &motor[m];

        uint16_t newFaults = MC_NO_FAULTS;

        /* Over-voltage protection */
        if (batVoltage > (float)OV_VOLTAGE_THRESHOLD_V) {
            newFaults |= MC_OVER_VOLT;
        }
        /* Under-voltage protection */
        if (batVoltage > 5.0f && batVoltage < (float)UD_VOLTAGE_THRESHOLD_V) {
            newFaults |= MC_UNDER_VOLT;
        }
        /* Over-temperature MOSFET protection */
        if (boardTemp > (float)OV_TEMPERATURE_THRESHOLD_C) {
            newFaults |= MC_OVER_TEMP;
        }
        /* Over-current protection */
        if (ABS(mc->currentMotor) > MC_GetMotorCurrentLimit()) {
            newFaults |= MC_OVER_CURR;
        }
        /* Over-RPM protection (VESC-style) */
        if (ABS(mc->speedNow) > MAX_ERPM_COMMAND * 1.2f) {
            newFaults |= MC_OVER_CURR;  /* Use existing fault bit */
        }
        /* Over-duty protection (VESC-style) */
        if (ABS(mc->dutyNow) > 0.99f) {
            /* Soft-limit: don't fault, just clamp */
            mc->dutyNow = MC_ClampFloat(mc->dutyNow, -0.95f, 0.95f);
        }
        /* Over-regenerative current protection */
        if (mc->currentInput < -MC_GetBrakeCurrentLimit()) {
            newFaults |= MC_OVER_CURR;
        }

        if (newFaults != MC_NO_FAULTS) {
            mc->faultCode |= newFaults;
            mc->enable = false;
            mc->state = MC_STATE_FAULT_NOW;
            if (m == M_RIGHT) MC_WritePWM_Right(0, 0, 0, false);
            else              MC_WritePWM_Left(0, 0, 0, false);
        }
    }
}

/*============================================================================
 * Motor Control Initialization
 *============================================================================*/
void MX_MotorControl_Init(void)
{
    memset((void *)motor, 0, sizeof(motor));

    /* Enable DWT cycle counter for Hall timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    halfPwmPeriod = (uint16_t)(SYSCLK_FREQ / 2 / PWM_FREQUENCY);

    for (uint8_t m = 0; m < NBR_OF_MOTORS; m++) {
        motor[m].state     = MC_STATE_OFFSET_CALIB;
        motor[m].ctrlMode  = MC_CTRL_DUTY;
        motor[m].enable    = false;
        motor[m].faultCode = MC_NO_FAULTS;

        /* Initialize current PIDs (torque/flux) */
        MC_PID_Init((PID_Handle_t *)&motor[m].pidIq,
                    PID_TORQUE_KP_DEFAULT, PID_TORQUE_KI_DEFAULT, PID_TORQUE_KD_DEFAULT,
                    TF_KPDIV_POW2, TF_KIDIV_POW2, TF_KDDIV_POW2,
                    (int16_t)halfPwmPeriod);

        MC_PID_Init((PID_Handle_t *)&motor[m].pidId,
                    PID_FLUX_KP_DEFAULT, PID_FLUX_KI_DEFAULT, PID_FLUX_KD_DEFAULT,
                    TF_KPDIV_POW2, TF_KIDIV_POW2, TF_KDDIV_POW2,
                    (int16_t)halfPwmPeriod);

        /* Initialize speed PID */
        MC_PID_Init((PID_Handle_t *)&motor[m].pidSpeed,
                    PID_SPEED_KP_DEFAULT, PID_SPEED_KI_DEFAULT, PID_SPEED_KD_DEFAULT,
                    SP_KPDIV_POW2, SP_KIDIV_POW2, SP_KDDIV_POW2,
                    (int16_t)IQMAX);

        motor[m].hall.direction = 1;
        motor[m].hall.tachometer = 0;
        motor[m].hall.tachometerAbs = 0;

        /* Initialize flux weakening PI */
        MC_PID_Init((PID_Handle_t *)&motor[m].fw.pid,
                    FW_KP_GAIN, FW_KI_GAIN, 0,
                    FW_KPDIV_POW2, FW_KIDIV_POW2, 0,
                    0);  /* upper limit 0 (FW only produces negative Id) */
        motor[m].fw.pid.hLowerOutputLimit = FW_MAX_DEMAGCURRENT;
        motor[m].fw.IdFW = 0;
        motor[m].fw.vqdAmplFilt = 0;

        /* Initialize derating */
        motor[m].tempCurrentScale = 1.0f;
        motor[m].batCurrentScale = 1.0f;

        /* Default motor type */
        motor[m].motorType = MC_MOTOR_FOC;

        /* Position max speed — 0 means use POSITION_MAX_ERPM default */
        motor[m].positionMaxErpm = 0.0f;

        /* Detection init */
        motor[m].detectState = DETECT_IDLE;
        motor[m].detectResult.valid = false;

        /* Detection defaults (from hw_config) */
        motor[m].detectResult.resistance = FOC_MOTOR_R_OHM;
        motor[m].detectResult.inductance = FOC_MOTOR_L_H;
        motor[m].detectResult.flux_linkage = FOC_MOTOR_FLUX_LINKAGE;
    }

    /* ADC DMA setup: dual simultaneous mode, TIM8_TRGO trigger */
    __HAL_RCC_DMA1_CLK_ENABLE();
    DMA1_Channel1->CCR   = 0;
    DMA1_Channel1->CNDTR = 5;
    DMA1_Channel1->CPAR  = (uint32_t)&(ADC1->DR);
    DMA1_Channel1->CMAR  = (uint32_t)&adc_buffer;
    DMA1_Channel1->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1
                         | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE;
    DMA1_Channel1->CCR  |= DMA_CCR_EN;

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* Start PWM timers */
    startTimers();
}

/*============================================================================
 * Motor Control API
 *============================================================================*/
void MC_SetDuty(uint8_t motor_id, float duty)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    volatile MotorControl_t *m = &motor[motor_id];
    m->ctrlMode = MC_CTRL_DUTY;
    m->dutyCycleRef = MC_ClampFloat(duty, -1.0f, 1.0f);
    m->enable = true;
    if (m->state == MC_STATE_IDLE) m->state = MC_STATE_RUN;
}

void MC_SetCurrent(uint8_t motor_id, float current_a)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    volatile MotorControl_t *m = &motor[motor_id];
    m->ctrlMode = MC_CTRL_CURRENT;
    m->currentRef = MC_ClampFloat(current_a, -MC_GetMotorCurrentLimit(), MC_GetMotorCurrentLimit());
    m->enable = true;
    if (m->state == MC_STATE_IDLE) m->state = MC_STATE_RUN;
}

void MC_SetCurrentBrake(uint8_t motor_id, float current_a)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    volatile MotorControl_t *m = &motor[motor_id];
    m->ctrlMode = MC_CTRL_CURRENT_BRAKE;
    m->handbrakeRef = MC_ClampFloat((current_a < 0.0f) ? -current_a : current_a,
                                    0.0f,
                                    MC_GetBrakeCurrentLimit());
    m->enable = true;
    if (m->state == MC_STATE_IDLE) m->state = MC_STATE_RUN;
}

void MC_SetSpeed(uint8_t motor_id, float erpm)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    volatile MotorControl_t *m = &motor[motor_id];
    m->ctrlMode = MC_CTRL_SPEED;
    m->speedRef = MC_ClampFloat(erpm, -MAX_ERPM_COMMAND, MAX_ERPM_COMMAND);
    m->enable = true;
    if (m->state == MC_STATE_IDLE) m->state = MC_STATE_RUN;
}

void MC_SetHandbrake(uint8_t motor_id, float current_a)
{
    MC_SetCurrentBrake(motor_id, current_a);
}

void MC_SetPosition(uint8_t motor_id, float degrees)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].ctrlMode = MC_CTRL_POSITION;
    motor[motor_id].positionRef = MC_NormalizeDegrees(degrees);
    motor[motor_id].enable = true;
    if (motor[motor_id].state == MC_STATE_IDLE) motor[motor_id].state = MC_STATE_RUN;
}

void MC_EnableMotor(uint8_t motor_id, bool enable)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].enable = enable;
    if (enable && motor[motor_id].state == MC_STATE_IDLE) {
        motor[motor_id].state = MC_STATE_RUN;
    }
    if (!enable) {
        motor[motor_id].dutyCycleRef = 0.0f;
        motor[motor_id].currentRef   = 0.0f;
        motor[motor_id].speedRef     = 0.0f;
    }
}

void MC_StopMotor(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].enable = false;
    motor[motor_id].state  = MC_STATE_IDLE;
    motor[motor_id].dutyCycleRef = 0.0f;
    motor[motor_id].currentRef   = 0.0f;
    motor[motor_id].speedRef     = 0.0f;
}

void MC_FaultAck(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    if (motor[motor_id].state == MC_STATE_FAULT_NOW ||
        motor[motor_id].state == MC_STATE_FAULT_OVER) {
        motor[motor_id].faultCode = MC_NO_FAULTS;
        motor[motor_id].state = MC_STATE_IDLE;
    }
}

/*============================================================================
 * Motor Detection Step Machine (VESC-style motor parameter detection)
 * Runs at 1 kHz from MC_MediumFrequencyTask
 *============================================================================*/
static void MC_DetectStep(volatile MotorControl_t *m, uint8_t motor_id)
{
    switch (m->detectState) {
    case DETECT_R: {
        /* Resistance detection: apply known current, measure voltage drop
           R = V_applied / I_measured
           Use duty cycle to apply voltage and measure resulting current */
        if (m->detectCounter < DETECT_SETTLE_MS) {
            /* Apply test duty */
            m->ctrlMode = MC_CTRL_DUTY;
            m->dutyCycleRef = DETECT_DUTY;
            m->enable = true;
            if (m->state == MC_STATE_IDLE) m->state = MC_STATE_RUN;
            m->detectCounter++;
        } else if (m->detectCounter < DETECT_SETTLE_MS + DETECT_SAMPLES) {
            /* Accumulate current measurements */
            float iMeas = (float)m->Iqd.q / 32768.0f * NOMINAL_CURRENT / 1000.0f;
            m->detectAccum += iMeas;
            m->detectCounter++;
        } else {
            /* Calculate resistance */
            float avgI = m->detectAccum / (float)DETECT_SAMPLES;
            float vApplied = DETECT_DUTY * batVoltage;
            if (avgI > 0.1f) {
                m->detectResult.resistance = vApplied / avgI;
            } else {
                m->detectResult.resistance = FOC_MOTOR_R_OHM;
            }
            /* Move to inductance detection */
            m->detectAccum = 0.0f;
            m->detectCounter = 0;
            m->detectState = DETECT_L;
            m->dutyCycleRef = 0.0f;
        }
        break;
    }

    case DETECT_L: {
        /* Inductance detection: measure current rise time with step voltage
           L = V * dt / di
           Apply short voltage pulse and measure di/dt */
        if (m->detectCounter < DETECT_SETTLE_MS) {
            m->dutyCycleRef = 0.0f;
            m->detectCounter++;
        } else if (m->detectCounter == DETECT_SETTLE_MS) {
            /* Apply step voltage */
            m->dutyCycleRef = DETECT_DUTY * 2.0f;
            m->detectAccum = (float)m->Iqd.q;  /* Save initial current */
            m->detectCounter++;
        } else if (m->detectCounter < DETECT_SETTLE_MS + 5) {
            /* Wait a few cycles for current to rise */
            m->detectCounter++;
        } else {
            float iFinal = (float)m->Iqd.q;
            float di = (iFinal - m->detectAccum) / 32768.0f * NOMINAL_CURRENT / 1000.0f;
            float dt = 5.0f / (float)PWM_FREQUENCY;
            float vApplied = DETECT_DUTY * 2.0f * batVoltage;
            if (di > 0.01f) {
                m->detectResult.inductance = vApplied * dt / di;
            } else {
                m->detectResult.inductance = FOC_MOTOR_L_H;
            }
            /* Move to flux detection */
            m->detectAccum = 0.0f;
            m->detectCounter = 0;
            m->detectState = DETECT_FLUX;
            m->dutyCycleRef = 0.0f;
        }
        break;
    }

    case DETECT_FLUX: {
        /* Flux linkage detection: spin motor and measure BEMF
           Ψ = V_bemf / (ω_el)
           Spin motor at known duty and measure speed */
        if (m->detectCounter < DETECT_SETTLE_MS * 5) {
            m->dutyCycleRef = DETECT_DUTY * 3.0f;
            m->detectCounter++;
        } else if (m->detectCounter < DETECT_SETTLE_MS * 5 + DETECT_SAMPLES) {
            m->detectAccum += (float)m->hall.avgSpeed;
            m->detectCounter++;
        } else {
            float avgSpeed_01Hz = m->detectAccum / (float)DETECT_SAMPLES;
            float omega_el = avgSpeed_01Hz / 10.0f * 2.0f * (float)M_PI;
            float vApplied = DETECT_DUTY * 3.0f * batVoltage;
            float iR = m->currentMotor * m->detectResult.resistance;
            if (omega_el > 1.0f) {
                m->detectResult.flux_linkage = (vApplied - iR) / omega_el;
            } else {
                m->detectResult.flux_linkage = FOC_MOTOR_FLUX_LINKAGE;
            }
            /* Stop motor and mark detection done */
            m->dutyCycleRef = 0.0f;
            m->ctrlMode = m->detectPrevMode;
            m->enable = m->detectPrevEnable;
            if (!m->enable) m->state = MC_STATE_IDLE;
            m->detectResult.valid = true;
            m->detectState = DETECT_DONE;
        }
        break;
    }

    case DETECT_HALL: {
        /* Hall sensor table detection: spin motor slowly, record
           Hall state at each electrical angle sector */
        if (m->detectCounter < DETECT_SETTLE_MS * 10) {
            m->ctrlMode = MC_CTRL_OPEN_LOOP;
            m->openLoopVoltage = (int16_t)(DETECT_DUTY * INT16_MAX);
            m->enable = true;
            if (m->state == MC_STATE_IDLE) m->state = MC_STATE_RUN;

            /* Record Hall state at current angle */
            uint8_t hallState;
            if (motor_id == M_RIGHT) hallState = MC_ReadHallRight();
            else                     hallState = MC_ReadHallLeft();

            uint8_t sector = (uint8_t)(m->openLoopAngle >> 13);  /* 0-7 */
            if (sector < 8) {
                m->detectResult.hall_table[sector] = hallState;
            }
            m->detectCounter++;
        } else {
            m->ctrlMode = m->detectPrevMode;
            m->enable = m->detectPrevEnable;
            if (!m->enable) m->state = MC_STATE_IDLE;
            m->detectResult.valid = true;
            m->detectState = DETECT_DONE;
        }
        break;
    }

    default:
        break;
    }
}

/*============================================================================
 * Tachometer API (VESC-style)
 *============================================================================*/
int32_t MC_GetTachometer(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return 0;
    return motor[motor_id].hall.tachometer;
}

int32_t MC_GetTachometerAbs(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return 0;
    return motor[motor_id].hall.tachometerAbs;
}

void MC_ResetTachometer(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].hall.tachometer = 0;
    motor[motor_id].hall.tachometerAbs = 0;
}

/*============================================================================
 * Energy API (VESC-style)
 *============================================================================*/
float MC_GetAmpHours(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return 0.0f;
    return motor[motor_id].ampHours;
}

float MC_GetAmpHoursCharged(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return 0.0f;
    return motor[motor_id].ampHoursCharged;
}

float MC_GetWattHours(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return 0.0f;
    return motor[motor_id].wattHours;
}

float MC_GetWattHoursCharged(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return 0.0f;
    return motor[motor_id].wattHoursCharged;
}

void MC_ResetEnergy(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].ampHours = 0.0f;
    motor[motor_id].ampHoursCharged = 0.0f;
    motor[motor_id].wattHours = 0.0f;
    motor[motor_id].wattHoursCharged = 0.0f;
}

/*============================================================================
 * Motor Detection API (VESC-style)
 *============================================================================*/
void MC_DetectMotorRL(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].detectPrevMode = motor[motor_id].ctrlMode;
    motor[motor_id].detectPrevEnable = motor[motor_id].enable;
    motor[motor_id].detectState = DETECT_R;
    motor[motor_id].detectCounter = 0;
    motor[motor_id].detectAccum = 0.0f;
    motor[motor_id].detectResult.valid = false;
}

void MC_DetectHallFOC(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].detectPrevMode = motor[motor_id].ctrlMode;
    motor[motor_id].detectPrevEnable = motor[motor_id].enable;
    motor[motor_id].detectState = DETECT_HALL;
    motor[motor_id].detectCounter = 0;
    motor[motor_id].detectAccum = 0.0f;
    motor[motor_id].detectResult.valid = false;
}

bool MC_IsDetectDone(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return true;
    return motor[motor_id].detectState == DETECT_DONE;
}

MC_DetectResult_t MC_GetDetectResult(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) {
        MC_DetectResult_t empty = {0};
        return empty;
    }
    return motor[motor_id].detectResult;
}

/*============================================================================
 * Motor Type Selection API
 *============================================================================*/
void MC_SetMotorType(uint8_t motor_id, MC_MotorType_t type)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    motor[motor_id].motorType = type;
}

MC_MotorType_t MC_GetMotorType(uint8_t motor_id)
{
    if (motor_id >= NBR_OF_MOTORS) return MC_MOTOR_FOC;
    return motor[motor_id].motorType;
}

void MC_SetBldcDuty(uint8_t motor_id, float duty)
{
    if (motor_id >= NBR_OF_MOTORS) return;
    volatile MotorControl_t *m = &motor[motor_id];
    m->ctrlMode = MC_CTRL_BLDC;
    m->motorType = MC_MOTOR_BLDC;
    m->bldcDuty = MC_ClampFloat(duty, -BLDC_DUTY_MAX, BLDC_DUTY_MAX);
    m->enable = true;
    if (m->state == MC_STATE_IDLE) m->state = MC_STATE_RUN;
}
