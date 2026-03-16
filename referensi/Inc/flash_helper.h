/**
  ******************************************************************************
  * @file    flash_helper.h
  * @brief   Flash EEPROM emulation for persistent configuration storage
  *          Adapted from VESC flash_helper for STM32F103RCT6 hoverboard board
  ******************************************************************************
  */

#ifndef FLASH_HELPER_H
#define FLASH_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "hw_config.h"

/*============================================================================
 * Motor Configuration Structure (stored in flash)
 * All parameters user-configurable via VESC protocol
 *============================================================================*/
typedef struct {
    uint32_t magic;             /* FLASH_CONFIG_MAGIC for validation */

    /* Motor parameters */
    float    motor_r;           /* Resistance per phase (Ohm) */
    float    motor_l;           /* Inductance (Henry) */
    float    motor_flux;        /* Flux linkage (Wb) */
    uint8_t  pole_pairs;        /* Pole pairs */
    uint8_t  motor_type;        /* 0=FOC, 1=BLDC */
    uint8_t  sensor_mode;       /* 0=hall */

    /* Current limits */
    float    current_max;       /* Max motor current (A) */
    float    current_min;       /* Min motor current (A) */
    float    current_in_max;    /* Max input current (A) */
    float    current_brake_max; /* Max brake current (A) */

    /* Voltage limits */
    float    v_in_max;          /* Max input voltage (V) */
    float    v_in_min;          /* Min input voltage (V) */

    /* Battery */
    float    bat_cut_start;     /* Battery cut-off start (V) */
    float    bat_cut_end;       /* Battery cut-off end (V) */
    uint8_t  bat_cells;         /* Battery cell count */
    float    bat_ah;            /* Battery capacity (Ah) */

    /* RPM/duty limits */
    float    rpm_max;           /* Max ERPM */
    float    duty_max;          /* Max duty cycle (0-1) */

    /* PID - Torque/Flux */
    int16_t  pid_iq_kp;
    int16_t  pid_iq_ki;
    int16_t  pid_iq_kd;

    /* PID - Speed */
    int16_t  pid_speed_kp;
    int16_t  pid_speed_ki;
    int16_t  pid_speed_kd;

    /* Position control */
    float    position_kp;       /* Position P gain */
    float    position_max_erpm; /* Max speed in position mode */

    /* Ramp */
    float    speed_ramp;        /* ERPM/s acceleration */
    float    current_ramp;      /* A/s current ramp */

    /* Thermal */
    float    temp_start;        /* Temp derating start (°C) */
    float    temp_end;          /* Temp derating end (°C) */

    /* Sensor */
    uint8_t  hall_table[8];     /* Hall sensor angle table */

    /* Observer */
    float    observer_gain;     /* FOC observer gain */

    /* Input */
    uint8_t  input_mode;        /* 0=UART, 1=ADC, 2=PPM */
    uint16_t adc_min;
    uint16_t adc_max;
    uint16_t adc_center;
    uint16_t adc_deadzone;
    float    throttle_curve;    /* 0=linear, 1=max expo */

    /* Timeout */
    uint16_t timeout_ms;        /* Command timeout (ms) */
    float    timeout_brake;     /* Brake current on timeout (A) */

    /* Misc */
    uint16_t pwm_freq;          /* PWM frequency (Hz) */
    float    fw_voltage_ref;    /* Flux weakening voltage reference (%) */

    /* Power limit */
    float    power_max;         /* Max motor input power per motor (W), 0 = disabled */

    uint32_t crc;               /* CRC32 for data integrity */
} mc_configuration;

/*============================================================================
 * Flash Helper API
 *============================================================================*/

/* Initialize flash helper and load config from flash */
void flash_helper_init(void);

/* Load configuration from flash into RAM */
bool flash_helper_load_config(mc_configuration *conf);

/* Save configuration from RAM to flash */
bool flash_helper_save_config(const mc_configuration *conf);

/* Set all configuration to defaults */
void flash_helper_set_defaults(mc_configuration *conf);

/* Get pointer to current active configuration */
mc_configuration *flash_helper_get_config(void);

/* Erase configuration (reset to defaults) */
bool flash_helper_erase_config(void);

/* Write raw data to flash (for firmware update) */
bool flash_helper_write_new_app_data(uint32_t offset, const uint8_t *data, uint32_t len);

/* Erase new app area (for firmware update) */
bool flash_helper_erase_new_app(void);

/* Jump to bootloader */
void flash_helper_jump_to_bootloader(void);

/* CRC32 calculation */
uint32_t flash_helper_crc32(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_HELPER_H */
