/**
  ******************************************************************************
  * @file    app_control.c
  * @brief   Application layer for motor control input (VESC-style)
  *          ADC throttle, PPM input, cruise control, traction control
  *
  * Features:
  *  - ADC throttle input with deadzone, filtering, safety checks
  *  - PPM/PWM receiver input decoding
  *  - Cruise control (speed hold)
  *  - Simple traction control (wheel slip detection)
  *  - Throttle curve (linear/exponential)
  ******************************************************************************
  */

#include "app_control.h"
#include "hw_config.h"
#include "mc_tasks.h"
#include "timeout.h"
#include "flash_helper.h"
#include <math.h>

/*============================================================================
 * Private Data
 *============================================================================*/
static AppMode_t currentMode = APP_NONE;
static ADC_Throttle_t adcThrottle;
static PPM_Input_t ppmInput;
static CruiseControl_t cruise;
static bool tractionEnabled = false;
static DiffDrive_t diffDrive;
static TrajectoryTarget_t trajectory;

/*============================================================================
 * ADC Throttle Processing
 *============================================================================*/
static float apply_throttle_curve(float input, float curve)
{
    /* curve: 0.0 = linear, 1.0 = max exponential
       output = input * (1 - curve) + input^3 * curve */
    if (curve < 0.01f) return input;
    float sign = (input < 0.0f) ? -1.0f : 1.0f;
    float abs_in = (input < 0.0f) ? -input : input;
    float linear = abs_in;
    float cubic = abs_in * abs_in * abs_in;
    return sign * (linear * (1.0f - curve) + cubic * curve);
}

static void process_adc_throttle(void)
{
    mc_configuration *conf = flash_helper_get_config();

    /* Read ADC — for hoverboard, ADC throttle can use one of the existing
       ADC channels. If ADC_THROTTLE_CHANNEL is configured (PA3 = ADC_IN3),
       read from the ADC peripheral directly.  When no dedicated ADC is
       available, the raw value stays at zero and the validity check below
       will prevent erroneous motor commands. */
    if (adcThrottle.raw == 0) {
        /* Attempt a single conversion on the throttle ADC channel.
           On boards without a throttle input, this safely returns 0. */
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = ADC_THROTTLE_CHANNEL;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
        extern ADC_HandleTypeDef hadc1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK) {
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
                adcThrottle.raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
            }
            HAL_ADC_Stop(&hadc1);
        }
    }
    uint16_t raw = adcThrottle.raw;

    /* Check validity */
    if (raw < 100 || raw > 4000) {
        adcThrottle.invalidCount++;
        if (adcThrottle.invalidCount > 100) {
            adcThrottle.valid = false;
            return;
        }
    } else {
        adcThrottle.invalidCount = 0;
        adcThrottle.valid = true;
    }

    /* Normalize to -1.0 to 1.0 */
    float normalized;
    uint16_t center = conf->adc_center;
    uint16_t deadzone = conf->adc_deadzone;

    if (raw > center + deadzone) {
        normalized = (float)(raw - center - deadzone) /
                     (float)(conf->adc_max - center - deadzone);
    } else if (raw < center - deadzone) {
        normalized = -(float)(center - deadzone - raw) /
                      (float)(center - deadzone - conf->adc_min);
    } else {
        normalized = 0.0f;
    }

    /* Clamp */
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    /* Apply throttle curve */
    normalized = apply_throttle_curve(normalized, conf->throttle_curve);

    adcThrottle.normalized = normalized;

    /* Low-pass filter (α = 0.1) */
    adcThrottle.filtered += (normalized - adcThrottle.filtered) * 0.1f;
}

/*============================================================================
 * PPM Input Processing
 *============================================================================*/
static void process_ppm_input(void)
{
    /* PPM pulse width is captured by timer input capture.
       Typical: 1000µs = min, 1500µs = center, 2000µs = max
       The actual capture is done in the timer ISR; here we process it. */

    uint32_t pw = ppmInput.pulseWidth;

    /* Check validity: pulse should be between 800-2200 µs */
    if (pw < 800 || pw > 2200) {
        ppmInput.valid = false;
        return;
    }
    ppmInput.valid = true;

    /* Normalize: 1000-2000 → -1.0 to 1.0 */
    float normalized = ((float)pw - 1500.0f) / 500.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    /* Deadzone around center */
    if (normalized > -0.05f && normalized < 0.05f) {
        normalized = 0.0f;
    }

    ppmInput.normalized = normalized;

    /* Low-pass filter */
    ppmInput.filtered += (normalized - ppmInput.filtered) * 0.1f;
}

/*============================================================================
 * Traction Control (simple wheel slip detection)
 *============================================================================*/
static void process_traction_control(void)
{
    if (!tractionEnabled) return;

    /* Compare speed of both wheels.
       If one wheel spins much faster than the other, reduce its torque. */
    float speedR = motor[M_RIGHT].speedNow;
    float speedL = motor[M_LEFT].speedNow;
    float diff = (speedR > 0 ? speedR : -speedR) -
                 (speedL > 0 ? speedL : -speedL);

    /* If speed difference exceeds 30% of average, limit the faster wheel */
    float avg = ((speedR > 0 ? speedR : -speedR) +
                 (speedL > 0 ? speedL : -speedL)) / 2.0f;
    if (avg > 100.0f && diff > avg * 0.3f) {
        /* Reduce duty of the faster wheel */
        if ((speedR > 0 ? speedR : -speedR) > (speedL > 0 ? speedL : -speedL)) {
            motor[M_RIGHT].dutyCycleRef *= 0.9f;
        } else {
            motor[M_LEFT].dutyCycleRef *= 0.9f;
        }
    }
}

/* Trajectory completion threshold: both motors must be within this many
 * degrees of their target before the trajectory is marked done. */
#define TRAJECTORY_POSITION_TOLERANCE_DEG  2.0f

/*============================================================================
 * Differential Drive Processing
 * Converts (linear, angular) velocity to independent left/right wheel ERPM.
 * Standard robotics convention (right-hand rule viewed from above):
 *   positive angular = counter-clockwise → left wheel faster
 *   left_erpm  = linear + angular
 *   right_erpm = linear - angular
 *============================================================================*/
static void process_diff_drive(void)
{
    float left  = diffDrive.linearSpeed + diffDrive.angularSpeed;
    float right = diffDrive.linearSpeed - diffDrive.angularSpeed;

    /* Clamp to configured RPM limit */
    mc_configuration *conf = flash_helper_get_config();
    float limit = conf->rpm_max;
    if (left  >  limit) left  =  limit;
    if (left  < -limit) left  = -limit;
    if (right >  limit) right =  limit;
    if (right < -limit) right = -limit;

    MC_SetSpeed(M_LEFT,  left);
    MC_SetSpeed(M_RIGHT, right);
}

/*============================================================================
 * Trajectory Control Processing
 * Sets position target for each motor; the motor control's position PID
 * drives each motor toward the target position independently.
 *============================================================================*/
static void process_trajectory(void)
{
    if (!trajectory.active) return;

    /* Drive each motor toward its position target */
    MC_SetPosition(M_LEFT,  trajectory.leftPosDeg);
    MC_SetPosition(M_RIGHT, trajectory.rightPosDeg);

    /* Mark done once both motors are within 2° of target */
    float leftNow  = MC_GetRotorPositionDeg(M_LEFT);
    float rightNow = MC_GetRotorPositionDeg(M_RIGHT);
    float leftErr  = trajectory.leftPosDeg  - leftNow;
    float rightErr = trajectory.rightPosDeg - rightNow;
    if (leftErr  < 0.0f) leftErr  = -leftErr;
    if (rightErr < 0.0f) rightErr = -rightErr;
    if (leftErr < TRAJECTORY_POSITION_TOLERANCE_DEG &&
        rightErr < TRAJECTORY_POSITION_TOLERANCE_DEG) {
        trajectory.active = false;
    }
}


/*============================================================================
 * Cruise Control
 *============================================================================*/
static void process_cruise_control(void)
{
    if (!cruise.enabled) return;

    /* Maintain target speed on both motors */
    MC_SetSpeed(M_RIGHT, cruise.targetSpeed);
    MC_SetSpeed(M_LEFT, cruise.targetSpeed);
}

/*============================================================================
 * Public API
 *============================================================================*/

void app_control_init(void)
{
    currentMode = APP_NONE;
    adcThrottle.raw = 0;
    adcThrottle.normalized = 0.0f;
    adcThrottle.filtered = 0.0f;
    adcThrottle.valid = false;
    adcThrottle.invalidCount = 0;
    ppmInput.pulseWidth = 0;
    ppmInput.normalized = 0.0f;
    ppmInput.filtered = 0.0f;
    ppmInput.valid = false;
    ppmInput.lastUpdate = 0;
    cruise.enabled = false;
    cruise.targetSpeed = 0.0f;
    cruise.targetDuty = 0.0f;
    tractionEnabled = false;
    diffDrive.linearSpeed  = 0.0f;
    diffDrive.angularSpeed = 0.0f;
    trajectory.leftPosDeg   = 0.0f;
    trajectory.rightPosDeg  = 0.0f;
    trajectory.maxSpeedErpm = 0.0f;
    trajectory.active       = false;
}

void app_control_set_mode(AppMode_t mode)
{
    currentMode = mode;
}

AppMode_t app_control_get_mode(void)
{
    return currentMode;
}

void app_control_process(void)
{
    /* Check timeout first */
    if (timeout_has_timeout()) {
        return;  /* Timeout handler already stopped motors */
    }

    /* Process cruise control if active */
    if (cruise.enabled) {
        process_cruise_control();
        return;
    }

    switch (currentMode) {
    case APP_ADC:
        process_adc_throttle();
        if (adcThrottle.valid) {
            float val = adcThrottle.filtered;
            /* Apply to both motors (differential drive: same value = straight) */
            MC_SetDuty(M_RIGHT, val);
            MC_SetDuty(M_LEFT, val);
        } else {
            MC_StopMotor(M_RIGHT);
            MC_StopMotor(M_LEFT);
        }
        break;

    case APP_PPM:
        process_ppm_input();
        if (ppmInput.valid) {
            float val = ppmInput.filtered;
            MC_SetDuty(M_RIGHT, val);
            MC_SetDuty(M_LEFT, val);
        } else {
            MC_StopMotor(M_RIGHT);
            MC_StopMotor(M_LEFT);
        }
        break;

    case APP_DIFF_DRIVE:
        process_diff_drive();
        break;

    case APP_UART:
        /* UART control is handled by vesc_comm module directly */
        break;

    case APP_CUSTOM:
        /* Custom control logic — user can override this */
        break;

    case APP_NONE:
    default:
        break;
    }

    /* Traction control (always runs if enabled) */
    process_traction_control();

    /* Trajectory control: when active, overrides speed/duty outputs from all
     * other modes by issuing position commands to each motor independently. */
    process_trajectory();
}

float app_control_get_adc_value(void)
{
    return adcThrottle.filtered;
}

bool app_control_get_adc_valid(void)
{
    return adcThrottle.valid;
}

float app_control_get_ppm_value(void)
{
    return ppmInput.filtered;
}

bool app_control_get_ppm_valid(void)
{
    return ppmInput.valid;
}

void app_control_cruise_enable(bool enable)
{
    cruise.enabled = enable;
    if (enable) {
        /* Capture current speed as cruise target */
        cruise.targetSpeed = motor[M_RIGHT].speedNow;
    }
}

bool app_control_cruise_active(void)
{
    return cruise.enabled;
}

void app_control_traction_enable(bool enable)
{
    tractionEnabled = enable;
}

void app_control_diff_drive_set(float linear_erpm, float angular_erpm)
{
    diffDrive.linearSpeed  = linear_erpm;
    diffDrive.angularSpeed = angular_erpm;
}

void app_control_trajectory_set(float left_pos_deg, float right_pos_deg,
                                 float max_speed_erpm)
{
    mc_configuration *conf = flash_helper_get_config();
    float limit = conf->rpm_max;
    float maxSpd = (max_speed_erpm > 0.0f && max_speed_erpm < limit)
                   ? max_speed_erpm : limit;

    trajectory.leftPosDeg   = left_pos_deg;
    trajectory.rightPosDeg  = right_pos_deg;
    trajectory.maxSpeedErpm = maxSpd;
    trajectory.active       = true;

    /* Apply per-motor speed limit so the position PID respects this value */
    motor[M_LEFT].positionMaxErpm  = maxSpd;
    motor[M_RIGHT].positionMaxErpm = maxSpd;
}

bool app_control_trajectory_done(void)
{
    return !trajectory.active;
}
