/*
 * hw_hoverboard.h — Hardware configuration bridge for Hoverboard ESC
 * This is the HW_HEADER included by hw.h.
 * Maps hw_config.h definitions to the format expected by VESC code.
 */

#ifndef HW_HOVERBOARD_H
#define HW_HOVERBOARD_H

#include "hw_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Hardware Name & Type
 * ======================================================================== */

#define HW_NAME             "HoverboardESC"
#define HW_TYPE_VESC_NAME   HW_NAME

/* ========================================================================
 * Hardware Feature Flags
 * ======================================================================== */

#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
/* HW_HAS_DUAL_MOTORS is defined via build flag */
/* #define HW_HAS_NO_CAN */   /* CAN is available on PB8/PB9 */
#define HW_USE_INTERNAL_RC

/* ========================================================================
 * Voltage / Current Sensing
 * ======================================================================== */

#define V_REG                   3.3f
#define VIN_R1                  39000.0f
#define VIN_R2                  2200.0f
#define CURRENT_AMP_GAIN        AMPLIFICATION_GAIN
#define CURRENT_SHUNT_RES       RSHUNT

/* Factor for converting ADC counts to current in amps */
#define FAC_CURRENT             ((V_REG / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

/* Factor for converting ADC counts to voltage */
#define FAC_VOLTAGE             ((V_REG / 4095.0f) * ((VIN_R1 + VIN_R2) / VIN_R2))

/* ========================================================================
 * ADC Channel Indices — VESC standard layout
 * These index into the ADC_Value[] array filled by DMA
 * ======================================================================== */

/* ADC value array (filled by DMA) */
extern volatile uint16_t ADC_Value[];

/* ADC sample count */
#define HW_ADC_CHANNELS         16
#define HW_ADC_NBR_CONV         16

/* Motor 1 (Right) current sensors */
#define ADC_IND_CURR1           0
#define ADC_IND_CURR2           1
#define ADC_IND_CURR3           2

/* Motor 2 (Left) current sensors */
#define ADC_IND_CURR4           3
#define ADC_IND_CURR5           4
#define ADC_IND_CURR6           5

/* Voltage input */
#define ADC_IND_VIN_SENS        6

/* Temperature */
#define ADC_IND_TEMP_MOS        7
#define ADC_IND_TEMP_MOTOR      8

/* Extension / external */
#define ADC_IND_EXT             9
#define ADC_IND_EXT2            10

/* Phase voltage feedback (for FOC) */
#define ADC_V_L1                ADC_Value[11]
#define ADC_V_L2                ADC_Value[12]
#define ADC_V_L3                ADC_Value[13]
#define ADC_V_L4                ADC_Value[14]
#define ADC_V_L5                ADC_Value[15]
#define ADC_V_L6                ADC_Value[11]

/* ADC voltage conversion macro */
#define ADC_VOLTS(ch)           ((float)ADC_Value[ch] / 4095.0f * V_REG)

/* ========================================================================
 * NTC Temperature Sensing
 * ======================================================================== */

#define NTC_RES(adc_val)        (10000.0f * (float)(adc_val) / (4095.0f - (float)(adc_val)))
#define NTC_BETA                3380.0f
#define NTC_TEMP_MOS()          (1.0f / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0f) / NTC_BETA) + (1.0f / 298.15f)) - 273.15f)
#define NTC_RES_MOTOR(adc_val)  (10000.0f * (float)(adc_val) / (4095.0f - (float)(adc_val)))
#define NTC_TEMP_MOTOR(beta)    (1.0f / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0f) / beta) + (1.0f / 298.15f)) - 273.15f)

/* ========================================================================
 * LEDs
 * ======================================================================== */

#define LED_GREEN               0
#define LED_RED                 1

#define LED_GREEN_ON()          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_GREEN_OFF()         HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_RED_ON()            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_RED_OFF()           HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)

/* ========================================================================
 * Hall Sensors
 * ======================================================================== */

#define READ_HALL1()            HAL_GPIO_ReadPin(RIGHT_HALL_U_PORT, RIGHT_HALL_U_PIN)
#define READ_HALL2()            HAL_GPIO_ReadPin(RIGHT_HALL_V_PORT, RIGHT_HALL_V_PIN)
#define READ_HALL3()            HAL_GPIO_ReadPin(RIGHT_HALL_W_PORT, RIGHT_HALL_W_PIN)

#define READ_HALL1_2()          HAL_GPIO_ReadPin(LEFT_HALL_U_PORT, LEFT_HALL_U_PIN)
#define READ_HALL2_2()          HAL_GPIO_ReadPin(LEFT_HALL_V_PORT, LEFT_HALL_V_PIN)
#define READ_HALL3_2()          HAL_GPIO_ReadPin(LEFT_HALL_W_PORT, LEFT_HALL_W_PIN)

/* ========================================================================
 * Hall encoder pins (also used for SPI defaults)
 * ======================================================================== */

#define HW_HALL_ENC_GPIO1       GPIOC
#define HW_HALL_ENC_PIN1        10
#define HW_HALL_ENC_GPIO2       GPIOC
#define HW_HALL_ENC_PIN2        11
#define HW_HALL_ENC_GPIO3       GPIOC
#define HW_HALL_ENC_PIN3        12

/* SPI software pins */
#define HW_SPI_PORT_NSS         GPIOC
#define HW_SPI_PIN_NSS          12
#define HW_SPI_PORT_SCK         GPIOC
#define HW_SPI_PIN_SCK          10
#define HW_SPI_PORT_MOSI        GPIOC
#define HW_SPI_PIN_MOSI         11
#define HW_SPI_PORT_MISO        GPIOC
#define HW_SPI_PIN_MISO         12

/* ========================================================================
 * UART pins (for VESC comm)
 * ======================================================================== */

#define HW_UART_TX_PORT         GPIOB
#define HW_UART_TX_PIN          10
#define HW_UART_RX_PORT         GPIOB
#define HW_UART_RX_PIN          11

/* ========================================================================
 * Gate Driver Control (no DRV83xx on hoverboard)
 * ======================================================================== */

#define INIT_BR()               do {} while(0)
#define ENABLE_BR()             do {} while(0)
#define DISABLE_BR()            do {} while(0)
#define ENABLE_BR_2()           do {} while(0)
#define DISABLE_BR_2()          do {} while(0)

/* ========================================================================
 * Motor Timer Configuration
 * ======================================================================== */

/* PWM timer definitions match hw_config.h:
 * RIGHT_TIM = TIM1
 * LEFT_TIM  = TIM8
 */

#define MCPWM_FOC_TIMER_TIM1    1
#define MCPWM_FOC_TIMER_TIM8    8

/* ========================================================================
 * Limits
 * ======================================================================== */

#define HW_LIM_CURRENT          -MAX_MOTOR_CURRENT_A, MAX_MOTOR_CURRENT_A
#define HW_LIM_CURRENT_IN       -MAX_INPUT_CURRENT_A, MAX_INPUT_CURRENT_A
#define HW_LIM_CURRENT_ABS      0.0f, (MAX_MOTOR_CURRENT_A * 1.5f)
#define HW_LIM_VIN              10.0f, 60.0f
#define HW_LIM_ERPM             -MAX_ERPM_COMMAND, MAX_ERPM_COMMAND
#define HW_LIM_DUTY_MIN         0.005f, 0.1f
#define HW_LIM_DUTY_MAX         0.5f, 0.99f
#define HW_LIM_TEMP_FET         -40.0f, OV_TEMPERATURE_THRESHOLD_C

/* ========================================================================
 * Encoder / EXTI Configuration
 * ======================================================================== */

#define HW_ENC_TIM              TIM4
#define HW_ENC_TIM_ISR_CH      TIM4_IRQn
#define HW_ENC_TIM_ISR_VEC     TIM4_IRQHandler
#define HW_ENC_EXTI_ISR_CH     EXTI15_10_IRQn
#define HW_ENC_EXTI_ISR_VEC    EXTI15_10_IRQHandler
#define HW_ENC_EXTI_LINE       0x00008000U   /* EXTI Line 15 */
#define HW_ENC_EXTI_PORTSRC    2             /* GPIOC */
#define HW_ENC_EXTI_PINSRC     15
#define HW_ENC_TIM_CLK_EN()    __HAL_RCC_TIM4_CLK_ENABLE()
#define HW_ENC_EXTI_CLK_EN()   __HAL_RCC_AFIO_CLK_ENABLE()

/* ========================================================================
 * Hardware Setup Functions
 * ======================================================================== */

void hw_setup_adc_channels(void);
void hw_start_i2c(void);
void hw_stop_i2c(void);
void hw_try_restore_i2c(void);

#ifdef __cplusplus
}
#endif

#endif /* HW_HOVERBOARD_H */
