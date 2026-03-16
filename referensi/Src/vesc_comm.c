/**
  ******************************************************************************
  * @file    vesc_comm.c
  * @brief   Full VESC UART communication protocol for hoverboard ESC
  *          Comprehensive command support for VESC Tool compatibility.
  *          Dual motor control, FOC telemetry, configuration, detection stubs.
  ******************************************************************************
  */

#include "vesc_comm.h"
#include "mc_tasks.h"
#include "hw_config.h"
#include "flash_helper.h"
#include "timeout.h"
#include "terminal.h"
#include "app_control.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

static float vesc_duty_to_voltage(float duty)
{
    return duty * batVoltage;
}

static void vesc_append_string_response(uint8_t command, const char *message)
{
    uint8_t resp[128];
    int32_t ri = 0;

    resp[ri++] = command;
    while (*message != '\0' && ri < (int32_t)(sizeof(resp) - 1)) {
        resp[ri++] = (uint8_t)*message++;
    }

    VescComm_SendPacket(resp, (unsigned int)ri);
}

static void vesc_handle_terminal_command(const uint8_t *data, unsigned int len)
{
    char command[96];
    unsigned int copy_len = len - 1;

    if (copy_len >= sizeof(command)) {
        copy_len = sizeof(command) - 1;
    }

    memcpy(command, data + 1, copy_len);
    command[copy_len] = '\0';

    /* Route to terminal module for processing */
    terminal_process_string(command);
}

extern UART_HandleTypeDef huart3;

/* ---- CRC16 Table ---- */
static const uint16_t crc16_tab[] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x6006,0x7027,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x4864,0x5845,0x6826,0x7807,0x08e0,0x18c1,0x28a2,0x38c3,
    0xc92c,0xd90d,0xe96e,0xf94f,0x89a8,0x9989,0xa9ea,0xb9cb,
    0x5a15,0x4a34,0x7a57,0x6a76,0x1a91,0x0ab0,0x3ad3,0x2af2,
    0xdb1d,0xcb3c,0xfb5f,0xeb7e,0x9b99,0x8bb8,0xbbdb,0xabfa,
    0x6c46,0x7c67,0x4c04,0x5c25,0x2cc2,0x3ce3,0x0c80,0x1ca1,
    0xed4e,0xfd6f,0xcd0c,0xdd2d,0xad6a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78
};

static uint16_t crc16(const unsigned char *buf, unsigned int len) {
    uint16_t cksum = 0;
    for (unsigned int i = 0; i < len; i++) {
        cksum = crc16_tab[((cksum >> 8) ^ buf[i]) & 0xFF] ^ (cksum << 8);
    }
    return cksum;
}

/* ---- State ---- */
static VescPacketState_t vesc_pkt;

#define VESC_RX_BUF_SIZE  256
static uint8_t vesc_rx_dma_buf[VESC_RX_BUF_SIZE];
static uint16_t vesc_rx_read_pos = 0;
static volatile uint32_t vesc_timeout_cnt = 0;

/* Telemetry is now tracked per-motor in mc_tasks.c MotorControl_t.
   For VESC protocol, we report right motor values (primary motor). */

/* ---- Buffer helpers (big-endian) ---- */
static void buf_append_int16(uint8_t *buf, int16_t val, int32_t *idx) {
    buf[(*idx)++] = (uint8_t)(val >> 8);
    buf[(*idx)++] = (uint8_t)val;
}

static void buf_append_uint16(uint8_t *buf, uint16_t val, int32_t *idx) {
    buf[(*idx)++] = (uint8_t)(val >> 8);
    buf[(*idx)++] = (uint8_t)val;
}

static void buf_append_int32(uint8_t *buf, int32_t val, int32_t *idx) {
    buf[(*idx)++] = (uint8_t)(val >> 24);
    buf[(*idx)++] = (uint8_t)(val >> 16);
    buf[(*idx)++] = (uint8_t)(val >> 8);
    buf[(*idx)++] = (uint8_t)val;
}

static void buf_append_uint32(uint8_t *buf, uint32_t val, int32_t *idx) {
    buf[(*idx)++] = (uint8_t)(val >> 24);
    buf[(*idx)++] = (uint8_t)(val >> 16);
    buf[(*idx)++] = (uint8_t)(val >> 8);
    buf[(*idx)++] = (uint8_t)val;
}

static void buf_append_float16(uint8_t *buf, float val, float scale, int32_t *idx) {
    buf_append_int16(buf, (int16_t)(val * scale), idx);
}

static void buf_append_float32(uint8_t *buf, float val, float scale, int32_t *idx) {
    buf_append_int32(buf, (int32_t)(val * scale), idx);
}

static int16_t buf_get_int16(const uint8_t *buf, int32_t *idx) {
    int16_t val = ((int16_t)buf[*idx] << 8) | (int16_t)buf[*idx + 1];
    *idx += 2;
    return val;
}

static int32_t buf_get_int32(const uint8_t *buf, int32_t *idx) {
    int32_t val = ((int32_t)buf[*idx] << 24) | ((int32_t)buf[*idx + 1] << 16)
                | ((int32_t)buf[*idx + 2] << 8) | (int32_t)buf[*idx + 3];
    *idx += 4;
    return val;
}

static float buf_get_float16(const uint8_t *buf, float scale, int32_t *idx) {
    return (float)buf_get_int16(buf, idx) / scale;
}

static float buf_get_float32(const uint8_t *buf, float scale, int32_t *idx) {
    return (float)buf_get_int32(buf, idx) / scale;
}

/* ---- Packet Send ---- */
void VescComm_SendPacket(unsigned char *data, unsigned int len)
{
    if (len == 0 || len > 512) return;

    int b_ind = 0;
    if (len <= 255) {
        vesc_pkt.tx_buffer[b_ind++] = 2;
        vesc_pkt.tx_buffer[b_ind++] = (uint8_t)len;
    } else {
        vesc_pkt.tx_buffer[b_ind++] = 3;
        vesc_pkt.tx_buffer[b_ind++] = (uint8_t)(len >> 8);
        vesc_pkt.tx_buffer[b_ind++] = (uint8_t)(len & 0xFF);
    }

    memcpy(vesc_pkt.tx_buffer + b_ind, data, len);
    b_ind += len;

    uint16_t crc_val = crc16(data, len);
    vesc_pkt.tx_buffer[b_ind++] = (uint8_t)(crc_val >> 8);
    vesc_pkt.tx_buffer[b_ind++] = (uint8_t)(crc_val & 0xFF);
    vesc_pkt.tx_buffer[b_ind++] = 3;

    VescComm_UartSend(vesc_pkt.tx_buffer, b_ind);
}

void VescComm_UartSend(unsigned char *data, unsigned int len)
{
    HAL_UART_Transmit_DMA(&huart3, data, (uint16_t)len);
}

/* ---- Map internal faults to VESC fault codes ---- */
static uint8_t mapFaultCode(uint16_t internalFault)
{
    if (internalFault & MC_OVER_VOLT)  return VESC_FAULT_CODE_OVER_VOLTAGE;
    if (internalFault & MC_UNDER_VOLT) return VESC_FAULT_CODE_UNDER_VOLTAGE;
    if (internalFault & MC_OVER_TEMP)  return VESC_FAULT_CODE_OVER_TEMP_FET;
    if (internalFault & MC_OVER_CURR)  return VESC_FAULT_CODE_ABS_OVER_CURRENT;
    return VESC_FAULT_CODE_NONE;
}

/* ---- Process Decoded VESC Command ---- */
static void VescComm_ProcessCommand(unsigned char *data, unsigned int len)
{
    if (len < 1) return;

    VESC_CommPacketId cmd = (VESC_CommPacketId)data[0];
    int32_t idx = 1;

    vesc_timeout_cnt = 0;

    switch (cmd) {

    /* ---- Firmware Version ---- */
    case COMM_FW_VERSION: {
        uint8_t resp[32];
        int32_t ri = 0;
        resp[ri++] = COMM_FW_VERSION;
        resp[ri++] = FW_VERSION_MAJOR;
        resp[ri++] = FW_VERSION_MINOR;
        /* HW name (null-terminated) */
        const char *hw = HW_NAME;
        while (*hw) resp[ri++] = (uint8_t)*hw++;
        resp[ri++] = 0;
        /* UUID (12 bytes) */
        memset(resp + ri, 0, 12);
        ri += 12;
        /* Pairing done */
        resp[ri++] = 0;
        /* FW test version */
        resp[ri++] = 0;
        /* HW type */
        resp[ri++] = 0;  /* 0 = VESC */
        /* Custom config */
        resp[ri++] = 0;
        resp[ri++] = 0;
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Get Values (Full Telemetry) ---- */
    case COMM_GET_VALUES: {
        uint8_t resp[80];
        int32_t ri = 0;
        resp[ri++] = COMM_GET_VALUES;

        volatile MotorControl_t *m = &motor[M_RIGHT];

        buf_append_float16(resp, boardTemp, 10.0f, &ri);             /* temp_mos */
        buf_append_float16(resp, boardTemp, 10.0f, &ri);             /* temp_motor */
        buf_append_float32(resp, m->currentMotor, 100.0f, &ri);     /* current_motor */
        buf_append_float32(resp, m->currentInput, 100.0f, &ri);     /* current_in */
        buf_append_float32(resp, m->idNow, 100.0f, &ri);  /* id */
        buf_append_float32(resp, m->iqNow, 100.0f, &ri);  /* iq */
        buf_append_float16(resp, m->dutyNow, 1000.0f, &ri);         /* duty_now */
        buf_append_float32(resp, m->speedNow, 1.0f, &ri);           /* rpm (ERPM) */
        buf_append_float16(resp, batVoltage, 10.0f, &ri);           /* v_in */
        buf_append_float32(resp, m->ampHours, 10000.0f, &ri);         /* amp_hours */
        buf_append_float32(resp, m->ampHoursCharged, 10000.0f, &ri); /* amp_hours_charged */
        buf_append_float32(resp, m->wattHours, 10000.0f, &ri);        /* watt_hours */
        buf_append_float32(resp, m->wattHoursCharged, 10000.0f, &ri);/* watt_hours_charged */
        buf_append_int32(resp, m->hall.tachometer, &ri);                     /* tachometer */
        buf_append_int32(resp, m->hall.tachometerAbs, &ri);                 /* tachometer_abs */
        resp[ri++] = mapFaultCode(m->faultCode);                     /* fault_code */
        buf_append_float32(resp, 0.0f, 1.0f, &ri);                  /* pid_pos_now */
        resp[ri++] = 0;                                              /* controller_id */
        buf_append_float16(resp, boardTemp, 10.0f, &ri);            /* temp_mos_1 */
        buf_append_float16(resp, boardTemp, 10.0f, &ri);            /* temp_mos_2 */
        buf_append_float16(resp, boardTemp, 10.0f, &ri);            /* temp_mos_3 */
        buf_append_float32(resp, m->vdNow, 1000.0f, &ri);  /* vd */
        buf_append_float32(resp, m->vqNow, 1000.0f, &ri);  /* vq */
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Get Values Selective ---- */
    case COMM_GET_VALUES_SELECTIVE: {
        if (len < 5) break;
        uint32_t mask = (uint32_t)buf_get_int32(data, &idx);
        uint8_t resp[80];
        int32_t ri = 0;
        resp[ri++] = COMM_GET_VALUES_SELECTIVE;
        buf_append_uint32(resp, mask, &ri);

        volatile MotorControl_t *m = &motor[M_RIGHT];

        if (mask & ((uint32_t)1 << 0))  buf_append_float16(resp, boardTemp, 10.0f, &ri);
        if (mask & ((uint32_t)1 << 1))  buf_append_float16(resp, boardTemp, 10.0f, &ri);
        if (mask & ((uint32_t)1 << 2))  buf_append_float32(resp, m->currentMotor, 100.0f, &ri);
        if (mask & ((uint32_t)1 << 3))  buf_append_float32(resp, m->currentInput, 100.0f, &ri);
        if (mask & ((uint32_t)1 << 4))  buf_append_float32(resp, 0.0f, 100.0f, &ri);
        if (mask & ((uint32_t)1 << 5))  buf_append_float32(resp, 0.0f, 100.0f, &ri);
        if (mask & ((uint32_t)1 << 6))  buf_append_float16(resp, m->dutyNow, 1000.0f, &ri);
        if (mask & ((uint32_t)1 << 7))  buf_append_float32(resp, m->speedNow, 1.0f, &ri);
        if (mask & ((uint32_t)1 << 8))  buf_append_float16(resp, batVoltage, 10.0f, &ri);
        if (mask & ((uint32_t)1 << 9))  buf_append_float32(resp, m->ampHours, 10000.0f, &ri);
        if (mask & ((uint32_t)1 << 10)) buf_append_float32(resp, m->ampHoursCharged, 10000.0f, &ri);
        if (mask & ((uint32_t)1 << 11)) buf_append_float32(resp, m->wattHours, 10000.0f, &ri);
        if (mask & ((uint32_t)1 << 12)) buf_append_float32(resp, m->wattHoursCharged, 10000.0f, &ri);
        if (mask & ((uint32_t)1 << 13)) buf_append_int32(resp, m->hall.tachometer, &ri);
        if (mask & ((uint32_t)1 << 14)) buf_append_int32(resp, m->hall.tachometerAbs, &ri);
        if (mask & ((uint32_t)1 << 15)) resp[ri++] = mapFaultCode(m->faultCode);
        if (mask & ((uint32_t)1 << 16)) buf_append_float32(resp, 0.0f, 1.0f, &ri);
        if (mask & ((uint32_t)1 << 17)) resp[ri++] = 0;

        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Get Values Setup ---- */
    case COMM_GET_VALUES_SETUP: {
        uint8_t resp[40];
        int32_t ri = 0;
        resp[ri++] = COMM_GET_VALUES_SETUP;

        volatile MotorControl_t *m = &motor[M_RIGHT];
        buf_append_float16(resp, boardTemp, 10.0f, &ri);
        buf_append_float16(resp, boardTemp, 10.0f, &ri);
        buf_append_float32(resp, m->currentMotor, 100.0f, &ri);
        buf_append_float32(resp, m->currentInput, 100.0f, &ri);
        buf_append_float16(resp, m->dutyNow, 1000.0f, &ri);
        buf_append_float32(resp, m->speedNow, 1.0f, &ri);
        buf_append_float16(resp, batVoltage, 10.0f, &ri);
        buf_append_float32(resp, m->wattHours, 10000.0f, &ri);
        buf_append_float32(resp, m->wattHoursCharged, 10000.0f, &ri);
        buf_append_int32(resp, m->hall.tachometer, &ri);
        buf_append_int32(resp, m->hall.tachometerAbs, &ri);
        buf_append_float32(resp, 0.0f, 1.0f, &ri);
        resp[ri++] = mapFaultCode(m->faultCode);
        resp[ri++] = 0;
        resp[ri++] = 1;
        buf_append_float32(resp, 0.0f, 100.0f, &ri);
        buf_append_float32(resp, 0.0f, 100.0f, &ri);
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Set Duty Cycle ---- */
    case COMM_SET_DUTY: {
        if (len < 5) break;
        int32_t duty_i = buf_get_int32(data, &idx);
        float duty = (float)duty_i / 100000.0f;
        MC_SetDuty(M_RIGHT, duty);
        MC_SetDuty(M_LEFT, duty);
        timeout_reset();
        break;
    }

    /* ---- Set Current ---- */
    case COMM_SET_CURRENT: {
        if (len < 5) break;
        int32_t cur_i = buf_get_int32(data, &idx);
        float current = (float)cur_i / 1000.0f;
        MC_SetCurrent(M_RIGHT, current);
        MC_SetCurrent(M_LEFT, current);
        timeout_reset();
        break;
    }

    /* ---- Set Current Brake ---- */
    case COMM_SET_CURRENT_BRAKE: {
        if (len < 5) break;
        int32_t cur_i = buf_get_int32(data, &idx);
        float current = (float)cur_i / 1000.0f;
        MC_SetCurrentBrake(M_RIGHT, current);
        MC_SetCurrentBrake(M_LEFT, current);
        timeout_reset();
        break;
    }

    /* ---- Set RPM ---- */
    case COMM_SET_RPM: {
        if (len < 5) break;
        int32_t erpm = buf_get_int32(data, &idx);
        MC_SetSpeed(M_RIGHT, (float)erpm);
        MC_SetSpeed(M_LEFT, (float)erpm);
        timeout_reset();
        break;
    }

    /* ---- Set Position ---- */
    case COMM_SET_POS: {
        if (len < 5) break;
        int32_t pos_i = buf_get_int32(data, &idx);
        float pos = (float)pos_i / 1000000.0f;
        MC_SetPosition(M_RIGHT, pos);
        MC_SetPosition(M_LEFT, pos);
        timeout_reset();
        break;
    }

    /* ---- Set Handbrake ---- */
    case COMM_SET_HANDBRAKE: {
        if (len < 5) break;
        int32_t cur_i = buf_get_int32(data, &idx);
        float current = (float)cur_i / 1000.0f;
        MC_SetHandbrake(M_RIGHT, current);
        MC_SetHandbrake(M_LEFT, current);
        timeout_reset();
        break;
    }

    /* ---- Set Current Relative ---- */
    case COMM_SET_CURRENT_REL: {
        if (len < 5) break;
        int32_t rel_i = buf_get_int32(data, &idx);
        float rel = (float)rel_i / 100000.0f;
        float current = rel * (float)NOMINAL_CURRENT / 1000.0f;
        MC_SetCurrent(M_RIGHT, current);
        MC_SetCurrent(M_LEFT, current);
        timeout_reset();
        break;
    }

    /* ---- Alive ---- */
    case COMM_ALIVE:
        vesc_timeout_cnt = 0;
        timeout_reset();
        break;

    /* ---- Firmware Update / Bootloader ---- */
    case COMM_JUMP_TO_BOOTLOADER:
        MC_StopMotor(M_RIGHT);
        MC_StopMotor(M_LEFT);
        flash_helper_jump_to_bootloader();
        break;

    case COMM_ERASE_NEW_APP: {
        bool ok = flash_helper_erase_new_app();
        uint8_t resp[3];
        int32_t ri = 0;
        resp[ri++] = COMM_ERASE_NEW_APP;
        resp[ri++] = ok ? 1 : 0;
        VescComm_SendPacket(resp, ri);
        break;
    }

    case COMM_WRITE_NEW_APP_DATA: {
        if (len >= 5) {
            uint32_t offset = (uint32_t)buf_get_int32(data, &idx);
            bool ok = flash_helper_write_new_app_data(offset,
                                                      data + (uint32_t)idx,
                                                      len - 5u);
            uint8_t resp[3];
            int32_t ri = 0;
            resp[ri++] = COMM_WRITE_NEW_APP_DATA;
            resp[ri++] = ok ? 1 : 0;
            VescComm_SendPacket(resp, ri);
        }
        break;
    }

    /* ---- Reboot ---- */
    case COMM_REBOOT:
        MC_StopMotor(M_RIGHT);
        MC_StopMotor(M_LEFT);
        NVIC_SystemReset();
        break;

    /* ---- Get MCCONF ---- */
    case COMM_GET_MCCONF:
    case COMM_GET_MCCONF_DEFAULT: {
        mc_configuration *conf;
        mc_configuration defaults;
        if (cmd == COMM_GET_MCCONF_DEFAULT) {
            flash_helper_set_defaults(&defaults);
            conf = &defaults;
        } else {
            conf = flash_helper_get_config();
        }

        uint8_t resp[128];
        int32_t ri = 0;
        resp[ri++] = cmd;
        /* Signature */
        buf_append_uint32(resp, FLASH_CONFIG_MAGIC, &ri);
        /* Motor type: 0=BLDC, 2=FOC */
        resp[ri++] = (conf->motor_type == 0) ? 2 : 0;
        /* Sensor mode: 2=HALL */
        resp[ri++] = 2;
        /* Current limits */
        buf_append_float32(resp, conf->current_max, 1000.0f, &ri);
        buf_append_float32(resp, conf->current_min, 1000.0f, &ri);
        buf_append_float32(resp, conf->current_in_max, 1000.0f, &ri);
        buf_append_float32(resp, -conf->current_in_max, 1000.0f, &ri);
        /* Voltage limits */
        buf_append_float32(resp, conf->v_in_min, 1000.0f, &ri);
        buf_append_float32(resp, conf->v_in_max, 1000.0f, &ri);
        /* FOC params */
        buf_append_float32(resp, conf->motor_l, 1e6f, &ri);
        buf_append_float32(resp, conf->motor_r, 1e6f, &ri);
        buf_append_float32(resp, conf->motor_flux, 1e6f, &ri);
        buf_append_float32(resp, conf->observer_gain, 1.0f, &ri);
        /* Pole pairs */
        resp[ri++] = conf->pole_pairs;
        /* Hall table */
        for (int i = 0; i < 8; i++) resp[ri++] = conf->hall_table[i];
        /* PWM frequency */
        buf_append_float32(resp, (float)conf->pwm_freq, 1.0f, &ri);
        /* Battery */
        buf_append_float32(resp, conf->bat_cut_start, 1000.0f, &ri);
        buf_append_float32(resp, conf->bat_cut_end, 1000.0f, &ri);
        /* Thermal */
        buf_append_float32(resp, conf->temp_start, 10.0f, &ri);
        buf_append_float32(resp, conf->temp_end, 10.0f, &ri);
        /* PID */
        buf_append_float32(resp, (float)conf->pid_speed_kp, 1000.0f, &ri);
        buf_append_float32(resp, (float)conf->pid_speed_ki, 1000.0f, &ri);
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Set MCCONF ---- */
    case COMM_SET_MCCONF:
    case COMM_SET_MCCONF_TEMP: {
        mc_configuration *conf = flash_helper_get_config();

        /* Parse basic parameters from the packet */
        if (len >= 6) {
            uint32_t sig = (uint32_t)buf_get_int32(data, &idx);
            (void)sig;
        }
        if (len >= 7) {
            uint8_t mtype = data[idx++];
            conf->motor_type = (mtype == 2) ? 0 : 1;
        }
        if (len >= 8) {
            uint8_t smode = data[idx++];
            conf->sensor_mode = smode;
        }
        if (len >= 12) conf->current_max = buf_get_float32(data, 1000.0f, &idx);
        if (len >= 16) conf->current_min = buf_get_float32(data, 1000.0f, &idx);
        if (len >= 20) conf->current_in_max = buf_get_float32(data, 1000.0f, &idx);

        /* Save to flash if COMM_SET_MCCONF */
        if (cmd == COMM_SET_MCCONF) {
            flash_helper_save_config(conf);
        }
        break;
    }

    /* ---- Get APPCONF ---- */
    case COMM_GET_APPCONF:
    case COMM_GET_APPCONF_DEFAULT: {
        uint8_t resp[32];
        int32_t ri = 0;
        resp[ri++] = cmd;
        buf_append_uint32(resp, 0x87654321, &ri);  /* Signature */
        resp[ri++] = 3;   /* app_to_use: UART */
        resp[ri++] = 0;   /* controller_id */
        buf_append_uint32(resp, VESC_USART_BAUD, &ri);
        resp[ri++] = 0;   /* send_can_status */
        buf_append_uint16(resp, 50, &ri);  /* send_can_status_rate_hz */
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Set APPCONF ---- */
    case COMM_SET_APPCONF:
        break;

    /* ---- Terminal Command ---- */
    case COMM_TERMINAL_CMD:
    case COMM_TERMINAL_CMD_SYNC: {
        vesc_handle_terminal_command(data, len);
        break;
    }

    /* ---- Detect Motor Param ---- */
    case COMM_DETECT_MOTOR_PARAM:
    case COMM_DETECT_MOTOR_R_L: {
        /* Start motor R/L detection (VESC-style) */
        MC_DetectMotorRL(M_RIGHT);
        /* Wait briefly for detection to start, then report current values */
        uint8_t resp[32];
        int32_t ri = 0;
        resp[ri++] = cmd;
        MC_DetectResult_t result = MC_GetDetectResult(M_RIGHT);
        buf_append_float32(resp, result.inductance, 1e6f, &ri);
        buf_append_float32(resp, result.resistance, 1e6f, &ri);
        buf_append_float32(resp, result.flux_linkage, 1e6f, &ri);
        VescComm_SendPacket(resp, ri);
        break;
    }

    case COMM_DETECT_MOTOR_FLUX_LINKAGE:
    case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP: {
        /* Report flux linkage (detection runs asynchronously) */
        uint8_t resp[16];
        int32_t ri = 0;
        resp[ri++] = cmd;
        MC_DetectResult_t result = MC_GetDetectResult(M_RIGHT);
        buf_append_float32(resp, result.flux_linkage, 1e6f, &ri);
        VescComm_SendPacket(resp, ri);
        break;
    }

    case COMM_DETECT_ENCODER:
    case COMM_DETECT_HALL_FOC: {
        /* Start Hall sensor detection */
        MC_DetectHallFOC(M_RIGHT);
        uint8_t resp[32];
        int32_t ri = 0;
        resp[ri++] = cmd;
        MC_DetectResult_t result = MC_GetDetectResult(M_RIGHT);
        buf_append_float32(resp, result.inductance, 1e6f, &ri);
        buf_append_float32(resp, result.resistance, 1e6f, &ri);
        buf_append_float32(resp, result.flux_linkage, 1e6f, &ri);
        /* Add Hall table */
        for (int i = 0; i < 8; i++) resp[ri++] = result.hall_table[i];
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Detect Apply All FOC ---- */
    case COMM_DETECT_APPLY_ALL_FOC: {
        uint8_t resp[4];
        int32_t ri = 0;
        resp[ri++] = cmd;
        buf_append_int16(resp, 0, &ri);  /* result: success */
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Rotor Position ---- */
    case COMM_ROTOR_POSITION: {
        uint8_t resp[8];
        int32_t ri = 0;
        resp[ri++] = COMM_ROTOR_POSITION;
        buf_append_float32(resp, MC_GetRotorPositionDeg(M_RIGHT), 1e5f, &ri);
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Get Decoded PPM/ADC/CHUK ---- */
    case COMM_GET_DECODED_PPM: {
        uint8_t resp[16];
        int32_t ri = 0;
        resp[ri++] = cmd;
        buf_append_float32(resp, app_control_get_ppm_value(), 1000000.0f, &ri);
        buf_append_float32(resp, (float)app_control_get_ppm_valid(), 1000000.0f, &ri);
        VescComm_SendPacket(resp, ri);
        break;
    }
    case COMM_GET_DECODED_ADC: {
        uint8_t resp[16];
        int32_t ri = 0;
        resp[ri++] = cmd;
        buf_append_float32(resp, app_control_get_adc_value(), 1000000.0f, &ri);
        buf_append_float32(resp, 0.0f, 1000000.0f, &ri);  /* adc voltage: not separately measured */
        VescComm_SendPacket(resp, ri);
        break;
    }
    case COMM_GET_DECODED_CHUK: {
        uint8_t resp[16];
        int32_t ri = 0;
        resp[ri++] = cmd;
        buf_append_float32(resp, 0.0f, 1000000.0f, &ri);
        buf_append_float32(resp, 0.0f, 1000000.0f, &ri);
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Set Battery Cut ---- */
    case COMM_SET_BATTERY_CUT: {
        if (len >= 9) {
            mc_configuration *conf = flash_helper_get_config();
            conf->bat_cut_start = buf_get_float32(data, 1000.0f, &idx);
            conf->bat_cut_end = buf_get_float32(data, 1000.0f, &idx);
        }
        break;
    }

    /* ---- Forward CAN ---- */
    case COMM_FORWARD_CAN: {
        /* No CAN on this hardware — discard */
        break;
    }

    /* ---- Ping CAN ---- */
    case COMM_PING_CAN: {
        uint8_t resp[2];
        int32_t ri = 0;
        resp[ri++] = COMM_PING_CAN;
        resp[ri++] = 0;
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- App Disable Output ---- */
    case COMM_APP_DISABLE_OUTPUT: {
        if (len >= 5) {
            int32_t time_ms = buf_get_int32(data, &idx);
            if (time_ms > 0) {
                MC_StopMotor(M_RIGHT);
                MC_StopMotor(M_LEFT);
            }
        }
        break;
    }

    /* ---- Get IMU Data ---- */
    case COMM_GET_IMU_DATA: {
        uint8_t resp[64];
        int32_t ri = 0;
        resp[ri++] = COMM_GET_IMU_DATA;
        resp[ri++] = 0;  /* mask */
        /* All zeros for IMU data (no IMU on hoverboard) */
        for (int i = 0; i < 36; i++) resp[ri++] = 0;
        VescComm_SendPacket(resp, ri);
        break;
    }

    /* ---- Set Chuck Data ---- */
    case COMM_SET_CHUCK_DATA:
        break;

    /* ---- Custom App Data ---- */
    case COMM_CUSTOM_APP_DATA:
        break;

    /* ---- Sample/Print ---- */
    case COMM_SAMPLE_PRINT:
        break;

    default:
        break;
    }
}

/* ---- Process received byte through VESC packet framing ---- */
void VescComm_ProcessByte(uint8_t rx_data)
{
    unsigned int data_len = vesc_pkt.rx_write_ptr - vesc_pkt.rx_read_ptr;

    if (data_len >= 520) {
        vesc_pkt.rx_write_ptr = 0;
        vesc_pkt.rx_read_ptr = 0;
        vesc_pkt.bytes_left = 0;
        vesc_pkt.rx_buffer[vesc_pkt.rx_write_ptr++] = rx_data;
        return;
    }

    if (vesc_pkt.rx_write_ptr >= 520) {
        memmove(vesc_pkt.rx_buffer,
                vesc_pkt.rx_buffer + vesc_pkt.rx_read_ptr,
                data_len);
        vesc_pkt.rx_read_ptr = 0;
        vesc_pkt.rx_write_ptr = data_len;
    }

    vesc_pkt.rx_buffer[vesc_pkt.rx_write_ptr++] = rx_data;
    data_len++;

    if (vesc_pkt.bytes_left > 1) {
        vesc_pkt.bytes_left--;
        return;
    }

    for (;;) {
        unsigned char *buf = vesc_pkt.rx_buffer + vesc_pkt.rx_read_ptr;
        if (data_len == 0) break;

        if (buf[0] != 2 && buf[0] != 3) {
            vesc_pkt.rx_read_ptr++;
            data_len--;
            continue;
        }

        unsigned int hdr_len = buf[0];
        if (data_len < hdr_len) {
            vesc_pkt.bytes_left = hdr_len - data_len;
            break;
        }

        unsigned int pkt_len = 0;
        if (buf[0] == 2) {
            pkt_len = buf[1];
        } else {
            pkt_len = ((unsigned int)buf[1] << 8) | buf[2];
        }

        if (pkt_len == 0 || pkt_len > 512) {
            vesc_pkt.rx_read_ptr++;
            data_len--;
            continue;
        }

        unsigned int total = hdr_len + pkt_len + 3;
        if (data_len < total) {
            vesc_pkt.bytes_left = total - data_len;
            break;
        }

        if (buf[hdr_len + pkt_len + 2] != 3) {
            vesc_pkt.rx_read_ptr++;
            data_len--;
            continue;
        }

        uint16_t crc_calc = crc16(buf + hdr_len, pkt_len);
        uint16_t crc_rx = ((uint16_t)buf[hdr_len + pkt_len] << 8) | buf[hdr_len + pkt_len + 1];

        if (crc_calc == crc_rx) {
            VescComm_ProcessCommand(buf + hdr_len, pkt_len);
            vesc_pkt.rx_read_ptr += total;
            data_len -= total;
        } else {
            vesc_pkt.rx_read_ptr++;
            data_len--;
        }
    }

    if (data_len == 0) {
        vesc_pkt.rx_read_ptr = 0;
        vesc_pkt.rx_write_ptr = 0;
    }
}

/* ---- Terminal write callback (sends via VESC COMM_PRINT) ---- */
static void vesc_terminal_write(const char *str)
{
    vesc_append_string_response(COMM_PRINT, str);
}

/* ---- Initialize ---- */
void VescComm_Init(void)
{
    memset(&vesc_pkt, 0, sizeof(vesc_pkt));
    vesc_rx_read_pos = 0;

    /* Initialize terminal and set write callback */
    terminal_init();
    terminal_set_write_callback(vesc_terminal_write);

    /* Initialize flash config */
    flash_helper_init();

    /* Initialize timeout system */
    timeout_init();

    HAL_UART_Receive_DMA(&huart3, vesc_rx_dma_buf, VESC_RX_BUF_SIZE);
}

/* ---- FreeRTOS Task ---- */
void VescComm_Task(void *argument)
{
    (void)argument;
    VescComm_Init();

    for (;;) {
        /* Poll DMA buffer for new UART bytes */
        uint16_t dma_pos = VESC_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        while (vesc_rx_read_pos != dma_pos) {
            VescComm_ProcessByte(vesc_rx_dma_buf[vesc_rx_read_pos]);
            vesc_rx_read_pos = (vesc_rx_read_pos + 1) % VESC_RX_BUF_SIZE;
        }

        /* Feed watchdog and check command timeout */
        timeout_feed_WDT();

        /* Energy tracking is now handled in MC_MediumFrequencyTask */

        osDelay(10);
    }
}
