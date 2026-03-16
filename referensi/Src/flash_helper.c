/**
  ******************************************************************************
  * @file    flash_helper.c
  * @brief   Flash EEPROM emulation for persistent configuration storage
  *          Adapted from VESC flash_helper for STM32F103RCT6 hoverboard board
  *
  * STM32F103RC has 256KB flash with 1KB pages.
  * Last page (0x0803F800) is used for configuration storage.
  ******************************************************************************
  */

#include "flash_helper.h"
#include "mc_tasks.h"
#include <string.h>

/*============================================================================
 * Private Data
 *============================================================================*/
static mc_configuration activeConfig;
static bool configLoaded = false;

/*============================================================================
 * CRC32 (same polynomial as VESC)
 *============================================================================*/
uint32_t flash_helper_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

/*============================================================================
 * Set Defaults
 *============================================================================*/
void flash_helper_set_defaults(mc_configuration *conf)
{
    memset(conf, 0, sizeof(mc_configuration));
    conf->magic = FLASH_CONFIG_MAGIC;

    /* Motor parameters */
    conf->motor_r      = FOC_MOTOR_R_OHM;
    conf->motor_l      = FOC_MOTOR_L_H;
    conf->motor_flux   = FOC_MOTOR_FLUX_LINKAGE;
    conf->pole_pairs   = POLE_PAIR_NUM;
    conf->motor_type   = 0;  /* FOC */
    conf->sensor_mode  = 0;  /* Hall */

    /* Current limits */
    conf->current_max       = MAX_MOTOR_CURRENT_A;
    conf->current_min       = -MAX_MOTOR_CURRENT_A;
    conf->current_in_max    = MAX_INPUT_CURRENT_A;
    conf->current_brake_max = MAX_BRAKE_CURRENT_A;

    /* Voltage limits */
    conf->v_in_max    = (float)OV_VOLTAGE_THRESHOLD_V;
    conf->v_in_min    = (float)UD_VOLTAGE_THRESHOLD_V;

    /* Battery */
    conf->bat_cut_start = BAT_CUT_START_V;
    conf->bat_cut_end   = BAT_CUT_END_V;
    conf->bat_cells     = SI_BATTERY_CELLS;
    conf->bat_ah        = SI_BATTERY_AH;

    /* RPM/duty limits */
    conf->rpm_max  = MAX_ERPM_COMMAND;
    conf->duty_max = 0.95f;

    /* PID - Torque/Flux */
    conf->pid_iq_kp = PID_TORQUE_KP_DEFAULT;
    conf->pid_iq_ki = PID_TORQUE_KI_DEFAULT;
    conf->pid_iq_kd = PID_TORQUE_KD_DEFAULT;

    /* PID - Speed */
    conf->pid_speed_kp = PID_SPEED_KP_DEFAULT;
    conf->pid_speed_ki = PID_SPEED_KI_DEFAULT;
    conf->pid_speed_kd = PID_SPEED_KD_DEFAULT;

    /* Position */
    conf->position_kp       = POSITION_KP_ERPM_PER_DEG;
    conf->position_max_erpm = POSITION_MAX_ERPM;

    /* Ramp */
    conf->speed_ramp   = SPEED_RAMP_ERPMS_S;
    conf->current_ramp = CURRENT_RAMP_A_S;

    /* Thermal */
    conf->temp_start = TEMP_CUR_START_C;
    conf->temp_end   = TEMP_CUR_END_C;

    /* Hall table (default 120° spacing) */
    conf->hall_table[0] = 0;
    conf->hall_table[1] = 0;    /* state 1 */
    conf->hall_table[2] = 2;    /* state 2 */
    conf->hall_table[3] = 1;    /* state 3 */
    conf->hall_table[4] = 4;    /* state 4 */
    conf->hall_table[5] = 5;    /* state 5 */
    conf->hall_table[6] = 3;    /* state 6 */
    conf->hall_table[7] = 0;

    /* Observer */
    conf->observer_gain = FOC_OBSERVER_GAIN;

    /* Input */
    conf->input_mode   = 0;  /* UART */
    conf->adc_min      = ADC_INPUT_MIN;
    conf->adc_max      = ADC_INPUT_MAX;
    conf->adc_center   = ADC_INPUT_CENTER;
    conf->adc_deadzone = ADC_INPUT_DEADZONE;
    conf->throttle_curve = 0.0f;  /* Linear */

    /* Timeout */
    conf->timeout_ms    = TIMEOUT_MS;
    conf->timeout_brake = TIMEOUT_BRAKE_CURRENT;

    /* PWM */
    conf->pwm_freq = PWM_FREQUENCY;
    conf->fw_voltage_ref = (float)FW_VOLTAGE_REF / 10.0f;

    /* Power limit — 2000 W per motor (practical hoverboard limit) */
    conf->power_max = 2000.0f;

    /* CRC */
    conf->crc = flash_helper_crc32((const uint8_t *)conf,
                                    sizeof(mc_configuration) - sizeof(uint32_t));
}

/*============================================================================
 * Flash Operations (STM32F103 HAL Flash)
 *============================================================================*/

/* Erase the configuration flash page */
static bool flash_erase_config_page(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_CONFIG_PAGE_ADDR;
    erase.NbPages     = 1;

    uint32_t pageError = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &pageError);

    HAL_FLASH_Lock();
    return (status == HAL_OK);
}

/* Write data to flash (halfword-aligned, STM32F103 writes 16-bit) */
static bool flash_write_data(uint32_t addr, const uint8_t *data, uint32_t len)
{
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = HAL_OK;
    /* Write 16-bit halfwords */
    for (uint32_t i = 0; i < len && status == HAL_OK; i += 2) {
        uint16_t halfword;
        if (i + 1 < len) {
            halfword = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
        } else {
            halfword = (uint16_t)data[i] | 0xFF00;  /* Pad last byte */
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, halfword);
    }

    HAL_FLASH_Lock();
    return (status == HAL_OK);
}

/*============================================================================
 * Public API
 *============================================================================*/

void flash_helper_init(void)
{
    if (!flash_helper_load_config(&activeConfig)) {
        flash_helper_set_defaults(&activeConfig);
    }
    configLoaded = true;
}

bool flash_helper_load_config(mc_configuration *conf)
{
    /* Read from flash */
    const mc_configuration *flashConf = (const mc_configuration *)FLASH_CONFIG_PAGE_ADDR;

    /* Check magic */
    if (flashConf->magic != FLASH_CONFIG_MAGIC) {
        return false;
    }

    /* Copy to RAM */
    memcpy(conf, flashConf, sizeof(mc_configuration));

    /* Verify CRC */
    uint32_t calcCrc = flash_helper_crc32((const uint8_t *)conf,
                                           sizeof(mc_configuration) - sizeof(uint32_t));
    if (calcCrc != conf->crc) {
        return false;
    }

    return true;
}

bool flash_helper_save_config(const mc_configuration *conf)
{
    /* Calculate CRC */
    mc_configuration toSave;
    memcpy(&toSave, conf, sizeof(mc_configuration));
    toSave.magic = FLASH_CONFIG_MAGIC;
    toSave.crc = flash_helper_crc32((const uint8_t *)&toSave,
                                     sizeof(mc_configuration) - sizeof(uint32_t));

    /* Disable motor outputs during flash write */
    MC_StopMotor(M_RIGHT);
    MC_StopMotor(M_LEFT);

    /* Erase page */
    if (!flash_erase_config_page()) {
        return false;
    }

    /* Write config */
    if (!flash_write_data(FLASH_CONFIG_PAGE_ADDR,
                          (const uint8_t *)&toSave,
                          sizeof(mc_configuration))) {
        return false;
    }

    /* Verify by reading back */
    mc_configuration verify;
    if (!flash_helper_load_config(&verify)) {
        return false;
    }

    /* Update active config */
    memcpy(&activeConfig, &toSave, sizeof(mc_configuration));

    return true;
}

mc_configuration *flash_helper_get_config(void)
{
    if (!configLoaded) {
        flash_helper_init();
    }
    return &activeConfig;
}

bool flash_helper_erase_config(void)
{
    if (!flash_erase_config_page()) {
        return false;
    }
    flash_helper_set_defaults(&activeConfig);
    return true;
}

bool flash_helper_erase_new_app(void)
{
    /* For STM32F103RC, new app area would be after main application
       This is a placeholder for bootloader support */
    return true;
}

bool flash_helper_write_new_app_data(uint32_t offset, const uint8_t *data, uint32_t len)
{
    /* Placeholder for OTA firmware update support.
       The actual bootloader would handle writing to the application area. */
    (void)offset;
    (void)data;
    (void)len;
    return false;  /* Not implemented without bootloader */
}

void flash_helper_jump_to_bootloader(void)
{
    /* Disable all interrupts */
    __disable_irq();

    /* Stop motors */
    MC_StopMotor(M_RIGHT);
    MC_StopMotor(M_LEFT);

    /* Jump to system memory bootloader (STM32F103 built-in) */
    typedef void (*pFunction)(void);
    uint32_t jumpAddr = *(__IO uint32_t *)(SYSTEM_MEMORY_ADDR + 4);
    pFunction jumpToBootloader = (pFunction)jumpAddr;

    /* Set MSP to bootloader stack pointer */
    __set_MSP(*(__IO uint32_t *)SYSTEM_MEMORY_ADDR);

    /* Jump */
    jumpToBootloader();
}
