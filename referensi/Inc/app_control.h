/**
  ******************************************************************************
  * @file    app_control.h
  * @brief   Application layer for motor control input (VESC-style)
  *          ADC throttle, PPM input, cruise control, traction control
  ******************************************************************************
  */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * Application Control Mode
 *============================================================================*/
typedef enum {
    APP_NONE   = 0,    /* No application (UART only) */
    APP_ADC,           /* ADC throttle */
    APP_PPM,           /* PPM/PWM receiver */
    APP_UART,          /* UART control */
    APP_DIFF_DRIVE,    /* Differential drive robot (linear + angular velocity) */
    APP_CUSTOM         /* Custom control logic */
} AppMode_t;

/*============================================================================
 * ADC Throttle Configuration
 *============================================================================*/
typedef struct {
    uint16_t raw;          /* Raw ADC value (0-4095) */
    float    normalized;   /* Normalized value (-1.0 to 1.0) */
    float    filtered;     /* Low-pass filtered value */
    bool     valid;        /* True if ADC reading is within expected range */
    uint32_t invalidCount; /* Count of consecutive invalid readings */
} ADC_Throttle_t;

/*============================================================================
 * PPM Input Configuration
 *============================================================================*/
typedef struct {
    uint32_t pulseWidth;   /* Pulse width in µs (typically 1000-2000) */
    float    normalized;   /* Normalized value (-1.0 to 1.0) */
    float    filtered;     /* Low-pass filtered value */
    bool     valid;        /* True if PPM signal is present */
    uint32_t lastUpdate;   /* Tick of last valid pulse */
} PPM_Input_t;

/*============================================================================
 * Differential Drive (skid-steer robot)
 *============================================================================*/
typedef struct {
    float linearSpeed;     /* Forward/backward ERPM (+forward, -backward) */
    float angularSpeed;    /* Rotation ERPM (+counter-clockwise, -clockwise) */
} DiffDrive_t;

/*============================================================================
 * Trajectory Control (position-based, per-motor)
 *============================================================================*/
typedef struct {
    float leftPosDeg;      /* Target left motor position (degrees) */
    float rightPosDeg;     /* Target right motor position (degrees) */
    float maxSpeedErpm;    /* Maximum speed during trajectory (ERPM) */
    bool  active;          /* True when a trajectory is being executed */
} TrajectoryTarget_t;

/*============================================================================
 * Cruise Control
 *============================================================================*/
typedef struct {
    bool     enabled;
    float    targetSpeed;  /* ERPM */
    float    targetDuty;   /* For duty mode cruise */
} CruiseControl_t;

/*============================================================================
 * Application Control API
 *============================================================================*/

/* Initialize application control */
void app_control_init(void);

/* Set application mode */
void app_control_set_mode(AppMode_t mode);
AppMode_t app_control_get_mode(void);

/* Process application control (call at 1 kHz) */
void app_control_process(void);

/* ADC throttle */
float app_control_get_adc_value(void);
bool  app_control_get_adc_valid(void);

/* PPM input */
float app_control_get_ppm_value(void);
bool  app_control_get_ppm_valid(void);

/* Cruise control */
void  app_control_cruise_enable(bool enable);
bool  app_control_cruise_active(void);

/* Traction control */
void  app_control_traction_enable(bool enable);

/* Differential drive robot mode */
void  app_control_diff_drive_set(float linear_erpm, float angular_erpm);

/* Trajectory control (position target per motor) */
void  app_control_trajectory_set(float left_pos_deg, float right_pos_deg,
                                  float max_speed_erpm);
bool  app_control_trajectory_done(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */
