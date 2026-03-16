/**
  ******************************************************************************
  * @file    vesc_comm.h
  * @brief   Full VESC UART communication protocol for hoverboard ESC
  *          Supports all major VESC Tool commands for dual motor control
  ******************************************************************************
  */

#ifndef VESC_COMM_H
#define VESC_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* VESC Command IDs — comprehensive set for VESC Tool compatibility */
typedef enum {
    COMM_FW_VERSION              = 0,
    COMM_JUMP_TO_BOOTLOADER      = 1,
    COMM_ERASE_NEW_APP           = 2,
    COMM_WRITE_NEW_APP_DATA      = 3,
    COMM_GET_VALUES              = 4,
    COMM_SET_DUTY                = 5,
    COMM_SET_CURRENT             = 6,
    COMM_SET_CURRENT_BRAKE       = 7,
    COMM_SET_RPM                 = 8,
    COMM_SET_POS                 = 9,
    COMM_SET_HANDBRAKE           = 10,
    COMM_SET_DETECT              = 11,
    COMM_SET_SERVO_POS           = 12,
    COMM_SET_MCCONF              = 13,
    COMM_GET_MCCONF              = 14,
    COMM_GET_MCCONF_DEFAULT      = 15,
    COMM_SET_APPCONF             = 16,
    COMM_GET_APPCONF             = 17,
    COMM_GET_APPCONF_DEFAULT     = 18,
    COMM_SAMPLE_PRINT            = 19,
    COMM_TERMINAL_CMD            = 20,
    COMM_PRINT                   = 21,
    COMM_ROTOR_POSITION          = 22,
    COMM_EXPERIMENT_SAMPLE       = 23,
    COMM_DETECT_MOTOR_PARAM      = 24,
    COMM_DETECT_MOTOR_R_L        = 25,
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 26,
    COMM_DETECT_ENCODER          = 27,
    COMM_REBOOT                  = 28,
    COMM_ALIVE                   = 29,
    COMM_GET_DECODED_PPM         = 30,
    COMM_GET_DECODED_ADC         = 31,
    COMM_GET_DECODED_CHUK        = 32,
    COMM_FORWARD_CAN             = 33,
    COMM_SET_CHUCK_DATA          = 34,
    COMM_CUSTOM_APP_DATA         = 35,
    COMM_NRF_START_PAIRING       = 36,
    COMM_GPD_SET_FSW             = 37,
    COMM_GPD_BUFFER_NOTIFY       = 38,
    COMM_GPD_BUFFER_SIZE_LEFT    = 39,
    COMM_GPD_FILL_BUFFER         = 40,
    COMM_GPD_OUTPUT_SAMPLE       = 41,
    COMM_GPD_SET_MODE            = 42,
    COMM_GPD_FILL_BUFFER_INT8    = 43,
    COMM_GPD_FILL_BUFFER_INT16   = 44,
    COMM_GPD_SET_BUFFER_INT_SCALE = 45,
    COMM_GET_VALUES_SETUP        = 46,
    COMM_SET_MCCONF_TEMP         = 47,
    COMM_SET_MCCONF_TEMP_SETUP   = 48,
    COMM_GET_VALUES_SELECTIVE     = 49,
    COMM_GET_VALUES_SETUP_SELECTIVE = 50,
    COMM_EXT_NRF_PRESENT         = 51,
    COMM_EXT_NRF_ESB_SET_CH_ADDR = 52,
    COMM_EXT_NRF_ESB_SEND_DATA   = 53,
    COMM_EXT_NRF_SET_ENABLED     = 54,
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 55,
    COMM_DETECT_APPLY_ALL_FOC    = 56,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN = 57,
    COMM_ERASE_NEW_APP_ALL_CAN   = 58,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN = 59,
    COMM_PING_CAN                = 60,
    COMM_APP_DISABLE_OUTPUT      = 61,
    COMM_TERMINAL_CMD_SYNC       = 62,
    COMM_GET_IMU_DATA            = 63,
    COMM_BM_CONNECT              = 64,
    COMM_BM_ERASE_FLASH_ALL      = 65,
    COMM_BM_WRITE_FLASH          = 66,
    COMM_BM_REBOOT               = 67,
    COMM_BM_DISCONNECT           = 68,
    COMM_BM_MAP_PINS_DEFAULT     = 69,
    COMM_BM_MAP_PINS_NRF5X       = 70,
    COMM_ERASE_BOOTLOADER        = 71,
    COMM_ERASE_BOOTLOADER_ALL_CAN = 72,
    COMM_PLOT_INIT               = 73,
    COMM_PLOT_DATA               = 74,
    COMM_PLOT_ADD_GRAPH           = 75,
    COMM_PLOT_SET_GRAPH           = 76,
    COMM_GET_DECODED_BALANCE      = 77,
    COMM_BM_MEM_READ             = 78,
    COMM_WRITE_NEW_APP_DATA_LZO  = 79,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO = 80,
    COMM_BM_WRITE_FLASH_LZO     = 81,
    COMM_SET_CURRENT_REL         = 82,
    COMM_CAN_FWD_FRAME           = 83,
    COMM_SET_BATTERY_CUT         = 84,
    COMM_SET_BLE_NAME            = 85,
    COMM_SET_BLE_PIN             = 86,
    COMM_SET_CAN_MODE            = 87,
    COMM_GET_IMU_CALIBRATION     = 88,
    COMM_GET_MCCONF_TEMP         = 89,
    COMM_DETECT_HALL_FOC         = 90,
} VESC_CommPacketId;

/* VESC fault codes */
typedef enum {
    VESC_FAULT_CODE_NONE = 0,
    VESC_FAULT_CODE_OVER_VOLTAGE,
    VESC_FAULT_CODE_UNDER_VOLTAGE,
    VESC_FAULT_CODE_DRV,
    VESC_FAULT_CODE_ABS_OVER_CURRENT,
    VESC_FAULT_CODE_OVER_TEMP_FET,
    VESC_FAULT_CODE_OVER_TEMP_MOTOR,
    VESC_FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
    VESC_FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
    VESC_FAULT_CODE_MCU_UNDER_VOLTAGE,
    VESC_FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
    VESC_FAULT_CODE_ENCODER_SPI,
    VESC_FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
    VESC_FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
    VESC_FAULT_CODE_FLASH_CORRUPTION,
    VESC_FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
    VESC_FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
    VESC_FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
    VESC_FAULT_CODE_UNBALANCED_CURRENTS,
    VESC_FAULT_CODE_BRK,
    VESC_FAULT_CODE_RESOLVER_LOT,
    VESC_FAULT_CODE_RESOLVER_DOS,
    VESC_FAULT_CODE_RESOLVER_LOS,
    VESC_FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
    VESC_FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
    VESC_FAULT_CODE_ENCODER_NO_MAGNET,
    VESC_FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,
    VESC_FAULT_CODE_PHASE_FILTER,
    VESC_FAULT_CODE_ENCODER_FAULT,
    VESC_FAULT_CODE_LV_OUTPUT_FAULT,
} VESC_FaultCode;

/* Packet state for protocol decoding */
typedef struct {
    unsigned int rx_read_ptr;
    unsigned int rx_write_ptr;
    int          bytes_left;
    unsigned char rx_buffer[520];
    unsigned char tx_buffer[520];
} VescPacketState_t;

/* Firmware version */
#define FW_VERSION_MAJOR  6
#define FW_VERSION_MINOR  0
#define HW_NAME           "HoverFOC"

/* API */
void VescComm_Init(void);
void VescComm_ProcessByte(uint8_t byte);
void VescComm_SendPacket(unsigned char *data, unsigned int len);
void VescComm_Task(void *argument);
void VescComm_UartSend(unsigned char *data, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif /* VESC_COMM_H */
