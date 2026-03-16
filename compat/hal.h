/*
 * ChibiOS hal.h compatibility shim → STM32 HAL
 * Provides ChibiOS HAL API (PAL, CAN, Serial, etc.) mapped to STM32 HAL.
 */

#ifndef HAL_H_COMPAT
#define HAL_H_COMPAT

#include "stm32f1xx_hal.h"
#include "ch.h"
#include "spl_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * ChibiOS STM32 type mappings
 * ======================================================================== */

/* stm32_gpio_t is ChibiOS's name for GPIO_TypeDef */
typedef GPIO_TypeDef stm32_gpio_t;

/* Stack alignment type */
typedef uint32_t stkalign_t;

/* ========================================================================
 * HAL initialization (replaces ChibiOS halInit)
 * ======================================================================== */

static inline void halInit(void) {
    HAL_Init();
}

/* ========================================================================
 * PAL (Port Abstraction Layer) → STM32 HAL GPIO
 * ChibiOS uses pin numbers (0-15), STM32 HAL uses bitmasks (GPIO_PIN_x)
 * ======================================================================== */

/* PAL pin modes */
#define PAL_MODE_INPUT              0x00
#define PAL_MODE_INPUT_PULLUP       0x01
#define PAL_MODE_INPUT_PULLDOWN     0x02
#define PAL_MODE_INPUT_ANALOG       0x03
#define PAL_MODE_OUTPUT_PUSHPULL    0x10
#define PAL_MODE_OUTPUT_OPENDRAIN   0x11
#define PAL_MODE_ALTERNATE(n)       (0x20 | (n))
#define PAL_STM32_OSPEED_HIGHEST    0x100
#define PAL_MODE_STM32_ALTERNATE_PUSHPULL  0x30
#define PAL_MODE_STM32_ALTERNATE_OPENDRAIN 0x31
#define PAL_STM32_PUDR_FLOATING     0x00
#define PAL_STM32_PUDR_PULLUP       0x01
#define PAL_STM32_PUDR_PULLDOWN     0x02

/* PAL pad state */
#define PAL_LOW     0
#define PAL_HIGH    1

/* Convert ChibiOS pin number to HAL pin bitmask */
#define PAL_PIN_MASK(pin)   (1U << (pin))

static inline void palSetPad(GPIO_TypeDef *port, uint32_t pin) {
    HAL_GPIO_WritePin(port, (uint16_t)PAL_PIN_MASK(pin), GPIO_PIN_SET);
}

static inline void palClearPad(GPIO_TypeDef *port, uint32_t pin) {
    HAL_GPIO_WritePin(port, (uint16_t)PAL_PIN_MASK(pin), GPIO_PIN_RESET);
}

static inline uint32_t palReadPad(GPIO_TypeDef *port, uint32_t pin) {
    return (uint32_t)HAL_GPIO_ReadPin(port, (uint16_t)PAL_PIN_MASK(pin));
}

static inline void palWritePad(GPIO_TypeDef *port, uint32_t pin, uint32_t val) {
    HAL_GPIO_WritePin(port, (uint16_t)PAL_PIN_MASK(pin),
                      val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void palSetPadMode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PAL_PIN_MASK(pin);

    if (mode == PAL_MODE_INPUT || mode == PAL_MODE_INPUT_PULLUP || mode == PAL_MODE_INPUT_PULLDOWN) {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        if (mode == PAL_MODE_INPUT_PULLUP) GPIO_InitStruct.Pull = GPIO_PULLUP;
        else if (mode == PAL_MODE_INPUT_PULLDOWN) GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        else GPIO_InitStruct.Pull = GPIO_NOPULL;
    } else if (mode == PAL_MODE_INPUT_ANALOG) {
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    } else if (mode == PAL_MODE_OUTPUT_PUSHPULL) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    } else if (mode == PAL_MODE_OUTPUT_OPENDRAIN) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    } else {
        /* Alternate function or other modes */
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    }

    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static inline void palWriteGroup(GPIO_TypeDef *port, uint32_t mask, uint32_t offset, uint32_t bits) {
    uint32_t current = port->ODR;
    current &= ~(mask << offset);
    current |= (bits & mask) << offset;
    port->ODR = current;
}

static inline uint32_t palReadPort(GPIO_TypeDef *port) {
    return port->IDR;
}

static inline void palTogglePad(GPIO_TypeDef *port, uint32_t pin) {
    HAL_GPIO_TogglePin(port, (uint16_t)PAL_PIN_MASK(pin));
}

/* ========================================================================
 * CAN Driver stubs
 * ChibiOS CAN frames mapped to generic structures
 * ======================================================================== */

typedef struct {
    uint32_t SID;    /* Standard ID */
    uint32_t EID;    /* Extended ID */
    uint8_t  IDE;    /* Extended flag */
    uint8_t  RTR;    /* Remote flag */
    uint8_t  DLC;    /* Data length */
    uint8_t  data8[8];
    uint16_t data16[4];
    uint32_t data32[2];
    systime_t TIME;
    uint8_t  FMI;
} CANRxFrame;

typedef struct {
    uint32_t SID;    /* Standard ID */
    uint32_t EID;    /* Extended ID */
    uint8_t  IDE;    /* Extended flag */
    uint8_t  RTR;    /* Remote flag */
    uint8_t  DLC;    /* Data length */
    union {
        uint8_t  data8[8];
        uint16_t data16[4];
        uint32_t data32[2];
    };
} CANTxFrame;

/* CAN mailbox defines */
#define CAN_ANY_MAILBOX     0

/* CAN driver stub */
typedef struct {
    int dummy;
} CANDriver;

typedef struct {
    uint32_t btr;   /* Bit timing register */
    uint32_t mcr;   /* Mode control register */
} CANConfig;

/* CAN driver instances */
#ifndef CAND1
#define CAND1   can_driver_1
#endif
#ifndef CAND2
#define CAND2   can_driver_2
#endif

extern CANDriver can_driver_1;
extern CANDriver can_driver_2;

static inline void canStart(CANDriver *canp, const CANConfig *config) {
    (void)canp; (void)config;
}

static inline void canStop(CANDriver *canp) {
    (void)canp;
}

static inline int canReceive(CANDriver *canp, int mailbox, CANRxFrame *crfp, sysinterval_t timeout) {
    (void)canp; (void)mailbox; (void)crfp; (void)timeout;
    return -1;
}

static inline int canTransmit(CANDriver *canp, int mailbox, const CANTxFrame *ctfp, sysinterval_t timeout) {
    (void)canp; (void)mailbox; (void)ctfp; (void)timeout;
    return 0;
}

/* ========================================================================
 * Serial USB Driver stubs
 * ======================================================================== */

typedef struct {
    BaseSequentialStream vmt_base;
    event_source_t event;
    int state;
} SerialUSBDriver;

typedef struct {
    int dummy;
} SerialUSBConfig;

typedef struct {
    int dummy;
} USBDriver;

typedef struct {
    void *usb_lld_ep_fields;
} USBInEndpointState;

typedef struct {
    void *usb_lld_ep_fields;
} USBOutEndpointState;

typedef struct {
    USBInEndpointState *in_state;
    USBOutEndpointState *out_state;
    uint16_t in_maxsize;
    uint16_t out_maxsize;
    int in_cb;
    int out_cb;
    int setup_cb;
} USBEndpointConfig;

typedef struct {
    int dummy;
} USBDescriptor;

/* USB defines */
#define USB_EP_MODE_TYPE_BULK   0x00
#define USB_EP_MODE_TYPE_INTR   0x01
#define USB_EP_MODE_TYPE_ISOC   0x02
#define USB_EP_MODE_TYPE_CTRL   0x03

extern SerialUSBDriver SDU1;

#ifndef USBD1
#define USBD1   usb_driver_1
#endif

extern USBDriver usb_driver_1;

static inline void sduObjectInit(SerialUSBDriver *sdup) {
    (void)sdup;
}

static inline void sduStart(SerialUSBDriver *sdup, const SerialUSBConfig *config) {
    (void)sdup; (void)config;
}

static inline void sduStop(SerialUSBDriver *sdup) {
    (void)sdup;
}

static inline void sduConfigureHookI(SerialUSBDriver *sdup) {
    (void)sdup;
}

static inline void sduSOFHookI(SerialUSBDriver *sdup) {
    (void)sdup;
}

static inline void sduSuspendHookI(SerialUSBDriver *sdup) {
    (void)sdup;
}

static inline void sduWakeupHookI(SerialUSBDriver *sdup) {
    (void)sdup;
}

static inline void sduDataTransmitted(SerialUSBDriver *sdup, int ep) {
    (void)sdup; (void)ep;
}

static inline void sduDataReceived(SerialUSBDriver *sdup, int ep) {
    (void)sdup; (void)ep;
}

static inline void sduInterruptTransmitted(SerialUSBDriver *sdup, int ep) {
    (void)sdup; (void)ep;
}

static inline void usbStart(USBDriver *usbp, void *config) {
    (void)usbp; (void)config;
}

static inline void usbStop(USBDriver *usbp) {
    (void)usbp;
}

static inline void usbInitEndpointI(USBDriver *usbp, int ep, const USBEndpointConfig *config) {
    (void)usbp; (void)ep; (void)config;
}

static inline void usbDisconnectBus(USBDriver *usbp) {
    (void)usbp;
}

static inline void usbConnectBus(USBDriver *usbp) {
    (void)usbp;
}

/* ========================================================================
 * Serial Driver stubs
 * ======================================================================== */

typedef struct {
    BaseSequentialStream vmt_base;
    int state;
    EventGroupHandle_t event;
} SerialDriver;

typedef struct {
    uint32_t speed;
} SerialConfig;

#ifndef SD1
extern SerialDriver sd1;
#define SD1 sd1
#endif
#ifndef SD2
extern SerialDriver sd2;
#define SD2 sd2
#endif
#ifndef SD3
extern SerialDriver sd3;
#define SD3 sd3
#endif

static inline void sdStart(SerialDriver *sdp, const SerialConfig *config) {
    (void)sdp; (void)config;
}

static inline void sdStop(SerialDriver *sdp) {
    (void)sdp;
}

static inline int sdWrite(SerialDriver *sdp, const void *bp, size_t n) {
    (void)sdp; (void)bp; (void)n;
    return n;
}

static inline int sdRead(SerialDriver *sdp, void *bp, size_t n) {
    (void)sdp; (void)bp; (void)n;
    return 0;
}

static inline int sdGetTimeout(SerialDriver *sdp, sysinterval_t timeout) {
    (void)sdp; (void)timeout;
    return -1;
}

static inline int sdReadTimeout(SerialDriver *sdp, void *bp, size_t n, sysinterval_t timeout) {
    (void)sdp; (void)bp; (void)n; (void)timeout;
    return 0;
}

static inline int sdWriteTimeout(SerialDriver *sdp, const void *bp, size_t n, sysinterval_t timeout) {
    (void)sdp; (void)bp; (void)n; (void)timeout;
    return n;
}

static inline int sdGet(SerialDriver *sdp) {
    (void)sdp;
    return -1;
}

static inline int sdPut(SerialDriver *sdp, uint8_t b) {
    (void)sdp; (void)b;
    return 0;
}

static inline int sdAsynchronousRead(SerialDriver *sdp, void *bp, size_t n) {
    (void)sdp; (void)bp; (void)n;
    return 0;
}

/* ========================================================================
 * SPI Driver stubs
 * ======================================================================== */

typedef enum {
    SPI_UNINIT = 0,
    SPI_STOP,
    SPI_READY,
    SPI_ACTIVE
} spistate_t;

typedef struct SPIDriver_s {
    spistate_t state;
    SPI_TypeDef *spi;
    void *app_arg;
    void (*err_cb)(struct SPIDriver_s *spip, uint32_t flags);
} SPIDriver;

typedef struct {
    uint16_t cr1;
    uint16_t cr2;
    GPIO_TypeDef *ssport;
    uint16_t sspad;
} SPIConfig;

#ifndef SPID1
extern SPIDriver spid1;
#define SPID1 spid1
#endif
#ifndef SPID2
extern SPIDriver spid2;
#define SPID2 spid2
#endif

static inline void spiStart(SPIDriver *spip, const SPIConfig *config) {
    (void)spip; (void)config;
}

static inline void spiStop(SPIDriver *spip) {
    (void)spip;
}

static inline void spiStartExchangeI(SPIDriver *spip, size_t n, const void *txbuf, void *rxbuf) {
    (void)spip; (void)n; (void)txbuf; (void)rxbuf;
}

static inline void spiStartReceiveI(SPIDriver *spip, size_t n, void *rxbuf) {
    (void)spip; (void)n; (void)rxbuf;
}

static inline void spiSelect(SPIDriver *spip) {
    (void)spip;
}

static inline void spiUnselect(SPIDriver *spip) {
    (void)spip;
}

static inline void spiExchange(SPIDriver *spip, size_t n, const void *txbuf, void *rxbuf) {
    (void)spip; (void)n; (void)txbuf; (void)rxbuf;
}

/* ========================================================================
 * I2C Driver stubs
 * ======================================================================== */

typedef struct {
    int dummy;
} I2CDriver;

typedef struct {
    uint32_t op_mode;
    uint32_t clock_speed;
    uint32_t duty_cycle;
} I2CConfig;

#define OPMODE_I2C          0
#define STD_DUTY_CYCLE      0
#define FAST_DUTY_CYCLE_2   1

#ifndef I2CD1
extern I2CDriver i2cd1;
#define I2CD1 i2cd1
#endif
#ifndef I2CD2
extern I2CDriver i2cd2;
#define I2CD2 i2cd2
#endif

static inline void i2cStart(I2CDriver *i2cp, const I2CConfig *config) {
    (void)i2cp; (void)config;
}

static inline void i2cStop(I2CDriver *i2cp) {
    (void)i2cp;
}

static inline int i2cMasterTransmitTimeout(I2CDriver *i2cp, uint8_t addr,
                                            const uint8_t *txbuf, size_t txlen,
                                            uint8_t *rxbuf, size_t rxlen,
                                            sysinterval_t timeout) {
    (void)i2cp; (void)addr; (void)txbuf; (void)txlen;
    (void)rxbuf; (void)rxlen; (void)timeout;
    return 0;
}

static inline int i2cMasterReceiveTimeout(I2CDriver *i2cp, uint8_t addr,
                                           uint8_t *rxbuf, size_t rxlen,
                                           sysinterval_t timeout) {
    (void)i2cp; (void)addr; (void)rxbuf; (void)rxlen; (void)timeout;
    return 0;
}

static inline void i2cAcquireBus(I2CDriver *i2cp) {
    (void)i2cp;
}

static inline void i2cReleaseBus(I2CDriver *i2cp) {
    (void)i2cp;
}

static inline int i2cGetErrors(I2CDriver *i2cp) {
    (void)i2cp;
    return 0;
}

/* ========================================================================
 * ICU (Input Capture Unit) stubs
 * ======================================================================== */

typedef enum {
    ICU_UNINIT = 0,
    ICU_STOP,
    ICU_READY,
    ICU_WAITING,
    ICU_ACTIVE
} icustate_t;

typedef struct {
    icustate_t state;
    int dummy;
} ICUDriver;

typedef enum {
    ICU_INPUT_ACTIVE_HIGH = 0,
    ICU_INPUT_ACTIVE_LOW = 1
} icumode_t;

typedef struct {
    icumode_t mode;
    uint32_t frequency;
    void (*width_cb)(ICUDriver *);
    void (*period_cb)(ICUDriver *);
    void (*overflow_cb)(ICUDriver *);
    int channel;
    uint32_t dier;
} ICUConfig;

#define ICU_CHANNEL_1   0
#define ICU_CHANNEL_2   1

#ifndef ICUD3
extern ICUDriver icud3;
#define ICUD3 icud3
#endif
#ifndef ICUD4
extern ICUDriver icud4;
#define ICUD4 icud4
#endif

static inline void icuStart(ICUDriver *icup, const ICUConfig *config) {
    (void)icup; (void)config;
}

static inline void icuStartCapture(ICUDriver *icup) {
    (void)icup;
}

static inline void icuStop(ICUDriver *icup) {
    (void)icup;
}

static inline uint32_t icuGetWidthX(ICUDriver *icup) {
    (void)icup;
    return 0;
}

static inline uint32_t icuGetPeriodX(ICUDriver *icup) {
    (void)icup;
    return 0;
}

/* ========================================================================
 * ADC stubs
 * ======================================================================== */

static inline void adcAcquireBus(void *adcp) {
    (void)adcp;
}

static inline void adcReleaseBus(void *adcp) {
    (void)adcp;
}

/* ========================================================================
 * DMA stubs
 * ======================================================================== */

typedef void (*stm32_dmaisr_t)(void *p, uint32_t flags);

static inline int dmaStreamAllocate(void *stream, int prio, stm32_dmaisr_t func, void *param) {
    (void)stream; (void)prio; (void)func; (void)param;
    return 0;
}

static inline void dmaStreamRelease(void *stream) {
    (void)stream;
}

/* ========================================================================
 * UART driver stubs
 * ======================================================================== */

typedef struct {
    int dummy;
} UARTDriver;

typedef struct {
    uint32_t speed;
} UARTConfig;

static inline void uartStart(UARTDriver *uartp, const UARTConfig *config) {
    (void)uartp; (void)config;
}

static inline void uartStop(UARTDriver *uartp) {
    (void)uartp;
}

static inline void uartAcquireBus(UARTDriver *uartp) {
    (void)uartp;
}

static inline void uartReleaseBus(UARTDriver *uartp) {
    (void)uartp;
}

/* ========================================================================
 * PWM driver stubs
 * ======================================================================== */

typedef struct {
    int dummy;
} PWMDriver;

typedef struct {
    uint32_t frequency;
    uint32_t period;
} PWMConfig;

static inline void pwmStart(PWMDriver *pwmp, const PWMConfig *config) {
    (void)pwmp; (void)config;
}

static inline void pwmStop(PWMDriver *pwmp) {
    (void)pwmp;
}

static inline void pwmEnableChannel(PWMDriver *pwmp, int channel, uint32_t width) {
    (void)pwmp; (void)channel; (void)width;
}

/* ========================================================================
 * GPIO AF defines (for CAN, SPI etc.)
 * On STM32F1xx these are not used the same way as F4xx,
 * but provide defines for compatibility.
 * ======================================================================== */

#ifndef GPIO_AF_CAN1
#define GPIO_AF_CAN1    0x09
#endif
#ifndef GPIO_AF_CAN2
#define GPIO_AF_CAN2    0x09
#endif

/* ========================================================================
 * Watchdog stubs
 * ======================================================================== */

typedef struct {
    int dummy;
} WDGDriver;

typedef struct {
    uint16_t pr;
    uint16_t rlr;
} WDGConfig;

static inline void wdgStart(WDGDriver *wdgp, const WDGConfig *config) {
    (void)wdgp; (void)config;
}

static inline void wdgReset(WDGDriver *wdgp) {
    (void)wdgp;
}

/* ========================================================================
 * STM32F4 specific peripheral function stubs
 * These are used in the VESC code but reference F4 Standard Peripheral Library.
 * We provide HAL-compatible stubs here.
 * ======================================================================== */

/* RCC peripheral clock control stubs */
static inline void RCC_AHB1PeriphClockCmd(uint32_t periph, int state) {
    (void)periph; (void)state;
    /* On F1, CRC clock is on AHB: __HAL_RCC_CRC_CLK_ENABLE() */
    __HAL_RCC_CRC_CLK_ENABLE();
}

static inline void RCC_APB1PeriphClockCmd(uint32_t periph, int state) {
    (void)periph; (void)state;
}

static inline void RCC_APB2PeriphClockCmd(uint32_t periph, int state) {
    (void)periph; (void)state;
}

/* RCC defines */
#ifndef RCC_AHB1Periph_CRC
#define RCC_AHB1Periph_CRC     0x00001000U
#endif
#ifndef RCC_APB1Periph_TIM3
#define RCC_APB1Periph_TIM3    0x00000002U
#endif
#ifndef RCC_APB1Periph_TIM4
#define RCC_APB1Periph_TIM4    0x00000004U
#endif
#ifndef ENABLE
#define ENABLE                 1
#endif
#ifndef DISABLE
#define DISABLE                0
#endif

/* ========================================================================
 * TIM Standard Peripheral Library compatibility
 * ======================================================================== */

/* TIM_Channel defines */
#ifndef TIM_Channel_1
#define TIM_Channel_1   0x0000
#endif
#ifndef TIM_Channel_2
#define TIM_Channel_2   0x0004
#endif
#ifndef TIM_Channel_3
#define TIM_Channel_3   0x0008
#endif
#ifndef TIM_Channel_4
#define TIM_Channel_4   0x000C
#endif

/* TIM forced action */
#ifndef TIM_ForcedAction_InActive
#define TIM_ForcedAction_InActive   0x0040
#endif
#ifndef TIM_ForcedAction_Active
#define TIM_ForcedAction_Active     0x0050
#endif

/* TIM CC enable */
#ifndef TIM_CCx_Enable
#define TIM_CCx_Enable    0x0001
#endif
#ifndef TIM_CCx_Disable
#define TIM_CCx_Disable   0x0000
#endif
#ifndef TIM_CCxN_Enable
#define TIM_CCxN_Enable   0x0004
#endif
#ifndef TIM_CCxN_Disable
#define TIM_CCxN_Disable  0x0000
#endif

/* TIM event source */
#ifndef TIM_EventSource_COM
#define TIM_EventSource_COM  TIM_EGR_COMG
#endif

/* TIM OC mode */
#ifndef TIM_OCMode_PWM1
#define TIM_OCMode_PWM1     0x0060
#endif
#ifndef TIM_OCMode_PWM2
#define TIM_OCMode_PWM2     0x0070
#endif
#ifndef TIM_OCMode_Timing
#define TIM_OCMode_Timing   0x0000
#endif

/* TIM function stubs */
static inline void TIM_SelectOCxM(TIM_TypeDef *TIMx, uint16_t channel, uint16_t mode) {
    (void)TIMx; (void)channel; (void)mode;
}

static inline void TIM_CCxCmd(TIM_TypeDef *TIMx, uint16_t channel, uint16_t state) {
    (void)TIMx; (void)channel; (void)state;
}

static inline void TIM_CCxNCmd(TIM_TypeDef *TIMx, uint16_t channel, uint16_t state) {
    (void)TIMx; (void)channel; (void)state;
}

/* Functions like TIM_GenerateEvent, TIM_SetCompareX, TIM_GetCaptureX
 * are defined in spl_compat.h - do not redefine here */

static inline void TIM_Cmd(TIM_TypeDef *TIMx, int state) {
    if (state) TIMx->CR1 |= TIM_CR1_CEN;
    else TIMx->CR1 &= ~TIM_CR1_CEN;
}

static inline void TIM_CtrlPWMOutputs(TIM_TypeDef *TIMx, int state) {
    if (state) TIMx->BDTR |= TIM_BDTR_MOE;
    else TIMx->BDTR &= ~TIM_BDTR_MOE;
}

static inline void TIM_ITConfig(TIM_TypeDef *TIMx, uint16_t it, int state) {
    if (state) TIMx->DIER |= it;
    else TIMx->DIER &= ~it;
}

static inline void TIM_SetAutoreload(TIM_TypeDef *TIMx, uint32_t val) {
    TIMx->ARR = val;
}

static inline uint32_t TIM_GetCounter(TIM_TypeDef *TIMx) {
    return TIMx->CNT;
}

static inline void TIM_ClearFlag(TIM_TypeDef *TIMx, uint32_t flag) {
    TIMx->SR &= ~flag;
}

static inline uint8_t TIM_GetFlagStatus(TIM_TypeDef *TIMx, uint32_t flag) {
    return (TIMx->SR & flag) ? 1 : 0;
}

static inline void TIM_ClearITPendingBit(TIM_TypeDef *TIMx, uint32_t it) {
    TIMx->SR &= ~it;
}

/* TIM IT flags */
#ifndef TIM_IT_Update
#define TIM_IT_Update       TIM_DIER_UIE
#endif
#ifndef TIM_IT_CC1
#define TIM_IT_CC1          TIM_DIER_CC1IE
#endif
#ifndef TIM_IT_CC2
#define TIM_IT_CC2          TIM_DIER_CC2IE
#endif
#ifndef TIM_IT_CC3
#define TIM_IT_CC3          TIM_DIER_CC3IE
#endif
#ifndef TIM_IT_CC4
#define TIM_IT_CC4          TIM_DIER_CC4IE
#endif

#ifndef TIM_FLAG_Update
#define TIM_FLAG_Update     TIM_SR_UIF
#endif
#ifndef TIM_FLAG_CC1
#define TIM_FLAG_CC1        TIM_SR_CC1IF
#endif

/* ========================================================================
 * ADC/EXTI/DMA functions are now in spl_compat.h — do not duplicate here
 * ======================================================================== */

/* ========================================================================
 * EXTI Standard Peripheral Library compatibility
 * ======================================================================== */

/* EXTI Line defines */
#ifndef EXTI_Line0
#define EXTI_Line0      0x00000001U
#define EXTI_Line1      0x00000002U
#define EXTI_Line2      0x00000004U
#define EXTI_Line3      0x00000008U
#define EXTI_Line4      0x00000010U
#define EXTI_Line5      0x00000020U
#define EXTI_Line6      0x00000040U
#define EXTI_Line7      0x00000080U
#define EXTI_Line8      0x00000100U
#define EXTI_Line9      0x00000200U
#define EXTI_Line10     0x00000400U
#define EXTI_Line11     0x00000800U
#define EXTI_Line12     0x00001000U
#define EXTI_Line13     0x00002000U
#define EXTI_Line14     0x00004000U
#define EXTI_Line15     0x00008000U
#define EXTI_Line16     0x00010000U
#define EXTI_Line17     0x00020000U
#define EXTI_Line18     0x00040000U
#define EXTI_Line19     0x00080000U
#endif

static inline uint8_t EXTI_GetITStatus(uint32_t line) {
    return (EXTI->PR & line) ? 1 : 0;
}

static inline void EXTI_ClearITPendingBit(uint32_t line) {
    EXTI->PR = line;
}

/* EXTI Init structure (SPL-style) */
typedef struct {
    uint32_t EXTI_Line;
    uint32_t EXTI_Mode;
    uint32_t EXTI_Trigger;
    uint32_t EXTI_LineCmd;
} EXTI_InitTypeDef;

/* EXTI Mode */
#ifndef EXTI_Mode_Interrupt
#define EXTI_Mode_Interrupt     0x00
#define EXTI_Mode_Event         0x01
#endif

/* EXTI Trigger */
#ifndef EXTI_Trigger_Rising
#define EXTI_Trigger_Rising             0x00
#define EXTI_Trigger_Falling            0x01
#define EXTI_Trigger_Rising_Falling     0x02
#endif

static inline void EXTI_Init(EXTI_InitTypeDef *init) {
    if (init->EXTI_LineCmd) {
        if (init->EXTI_Mode == EXTI_Mode_Interrupt)
            EXTI->IMR |= init->EXTI_Line;
        if (init->EXTI_Trigger == EXTI_Trigger_Rising || init->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
            EXTI->RTSR |= init->EXTI_Line;
        if (init->EXTI_Trigger == EXTI_Trigger_Falling || init->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
            EXTI->FTSR |= init->EXTI_Line;
    } else {
        EXTI->IMR &= ~init->EXTI_Line;
        EXTI->RTSR &= ~init->EXTI_Line;
        EXTI->FTSR &= ~init->EXTI_Line;
    }
}

/* EXTI port source defines */
#ifndef EXTI_PortSourceGPIOA
#define EXTI_PortSourceGPIOA    0x00
#define EXTI_PortSourceGPIOB    0x01
#define EXTI_PortSourceGPIOC    0x02
#define EXTI_PortSourceGPIOD    0x03
#endif

#ifndef EXTI_PinSource0
#define EXTI_PinSource0     0
#define EXTI_PinSource1     1
#define EXTI_PinSource2     2
#define EXTI_PinSource3     3
#define EXTI_PinSource4     4
#define EXTI_PinSource5     5
#define EXTI_PinSource6     6
#define EXTI_PinSource7     7
#define EXTI_PinSource8     8
#define EXTI_PinSource9     9
#define EXTI_PinSource10    10
#define EXTI_PinSource11    11
#define EXTI_PinSource12    12
#define EXTI_PinSource13    13
#define EXTI_PinSource14    14
#define EXTI_PinSource15    15
#endif

/* RCC_APB2Periph_SYSCFG - F4 has SYSCFG, F1 uses AFIO */
#ifndef RCC_APB2Periph_SYSCFG
#define RCC_APB2Periph_SYSCFG   0x00000001U  /* Maps to AFIO enable on F1 */
#endif

/* ========================================================================
 * DMA Standard Peripheral Library compatibility
 * ======================================================================== */

static inline void DMA_ClearITPendingBit(uint32_t flag) {
    (void)flag;
}

/* DMA flag defines */
#ifndef DMA1_IT_TC1
#define DMA1_IT_TC1     0x00000002U
#endif
#ifndef DMA2_IT_TC1
#define DMA2_IT_TC1     0x00000002U
#endif

/* ========================================================================
 * NVIC Standard Peripheral Library compatibility
 * ======================================================================== */

static inline void NVIC_EnableIRQ_compat(IRQn_Type irq) {
    NVIC_EnableIRQ(irq);
}

static inline void NVIC_DisableIRQ_compat(IRQn_Type irq) {
    NVIC_DisableIRQ(irq);
}

/* ========================================================================
 * FLASH Standard Peripheral Library compatibility
 * STM32F4-specific flags mapped to STM32F1 equivalents
 * ======================================================================== */

/* Flash status enum — use unique names to avoid HAL conflicts */
typedef enum {
    FLASH_SPL_BUSY = 1,
    FLASH_SPL_ERROR_PG,
    FLASH_SPL_ERROR_WRP,
    FLASH_SPL_COMPLETE,
    FLASH_SPL_TIMEOUT
} FLASH_Status;

/* FLASH_COMPLETE used by VESC code (SPL naming) */
#ifndef FLASH_COMPLETE
#define FLASH_COMPLETE          FLASH_SPL_COMPLETE
#endif

/* STM32F4 flash flags → map to F1 equivalents */
#ifndef FLASH_FLAG_OPERR
#define FLASH_FLAG_OPERR        FLASH_FLAG_PGERR
#endif
#ifndef FLASH_FLAG_PGAERR
#define FLASH_FLAG_PGAERR       FLASH_FLAG_PGERR
#endif
#ifndef FLASH_FLAG_PGPERR
#define FLASH_FLAG_PGPERR       FLASH_FLAG_PGERR
#endif
#ifndef FLASH_FLAG_PGSERR
#define FLASH_FLAG_PGSERR       FLASH_FLAG_PGERR
#endif

/* STM32F4 flash sector defines → map to F1 page-based flash */
#define FLASH_Sector_0          0
#define FLASH_Sector_1          1
#define FLASH_Sector_2          2
#define FLASH_Sector_3          3
#define FLASH_Sector_4          4
#define FLASH_Sector_5          5
#define FLASH_Sector_6          6
#define FLASH_Sector_7          7
#define FLASH_Sector_8          8
#define FLASH_Sector_9          9
#define FLASH_Sector_10         10
#define FLASH_Sector_11         11
#define FLASH_SECTORS           12

/* STM32F4 flash functions → stubs for F1 */
static inline FLASH_Status FLASH_EraseSector(uint32_t sector, uint8_t voltage) {
    (void)sector; (void)voltage;
    return FLASH_SPL_COMPLETE;
}

static inline FLASH_Status FLASH_ProgramWord(uint32_t addr, uint32_t data) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);
    return FLASH_SPL_COMPLETE;
}

static inline FLASH_Status FLASH_ProgramHalfWord(uint32_t addr, uint16_t data) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data);
    return FLASH_SPL_COMPLETE;
}

static inline void FLASH_ClearFlag(uint32_t flag) {
    __HAL_FLASH_CLEAR_FLAG(flag);
}

/* VoltageRange defines (F4 specific, not used on F1 but needed for compilation) */
#define VoltageRange_1          0x00
#define VoltageRange_2          0x01
#define VoltageRange_3          0x02
#define VoltageRange_4          0x03

/* Flash latency */
#ifndef FLASH_Latency_0
#define FLASH_Latency_0     FLASH_ACR_LATENCY_0
#endif
#ifndef FLASH_Latency_1
#define FLASH_Latency_1     FLASH_ACR_LATENCY_1
#endif
#ifndef FLASH_Latency_2
#define FLASH_Latency_2     FLASH_ACR_LATENCY_2
#endif

/* ========================================================================
 * IWDG Standard Peripheral Library compatibility
 * ======================================================================== */

#ifndef IWDG_WriteAccess_Enable
#define IWDG_WriteAccess_Enable     0x5555
#endif

#ifndef IWDG_Prescaler_4
#define IWDG_Prescaler_4            0x00
#endif

#ifndef IWDG_Prescaler_8
#define IWDG_Prescaler_8            0x01
#endif

#ifndef IWDG_Prescaler_16
#define IWDG_Prescaler_16           0x02
#endif

#ifndef IWDG_Prescaler_32
#define IWDG_Prescaler_32           0x03
#endif

#ifndef IWDG_Prescaler_64
#define IWDG_Prescaler_64           0x04
#endif

#ifndef IWDG_Prescaler_128
#define IWDG_Prescaler_128          0x05
#endif

#ifndef IWDG_Prescaler_256
#define IWDG_Prescaler_256          0x06
#endif

static inline void IWDG_WriteAccessCmd(uint16_t cmd) {
    IWDG->KR = cmd;
}

static inline void IWDG_SetPrescaler(uint8_t prescaler) {
    IWDG->PR = prescaler;
}

static inline void IWDG_SetReload(uint16_t reload) {
    IWDG->RLR = reload;
}

static inline void IWDG_ReloadCounter(void) {
    IWDG->KR = 0xAAAA;
}

static inline void IWDG_Enable(void) {
    IWDG->KR = 0xCCCC;
}

/* ========================================================================
 * Misc ChibiOS board defines
 * ======================================================================== */

#ifndef STM32_UUID
#define STM32_UUID      ((uint32_t *)0x1FFFF7E8)
#endif

/* ChibiOS STM32 specific defines */
#ifndef STM32_PCLK1
#define STM32_PCLK1     (SystemCoreClock / 2)
#endif
#ifndef STM32_PCLK2
#define STM32_PCLK2     SystemCoreClock
#endif
#ifndef STM32_TIMCLK1
#define STM32_TIMCLK1   SystemCoreClock
#endif
#ifndef STM32_TIMCLK2
#define STM32_TIMCLK2   SystemCoreClock
#endif

/* System clock extern */
extern uint32_t SystemCoreClock;

/* ========================================================================
 * I2C address type
 * ======================================================================== */
typedef uint8_t i2caddr_t;

/* ========================================================================
 * Channel notification flags (serial/stream)
 * ======================================================================== */
#ifndef CHN_INPUT_AVAILABLE
#define CHN_INPUT_AVAILABLE     0x01
#define CHN_OUTPUT_EMPTY        0x02
#define CHN_CONNECTED           0x04
#define CHN_DISCONNECTED        0x08
#define CHN_TRANSMISSION_END    0x10
#endif

/* chEvtRegisterMaskWithFlags: Register event source with flags */
#ifndef COMPAT_EVENT_LISTENER_DEFINED
#define COMPAT_EVENT_LISTENER_DEFINED
typedef struct {
    int _evt_dummy;
} event_listener_t;
#endif

#ifndef COMPAT_EVT_REGISTER_DEFINED
#define COMPAT_EVT_REGISTER_DEFINED
static inline void chEvtRegisterMaskWithFlags(EventGroupHandle_t *source,
                                               event_listener_t *el,
                                               eventmask_t events,
                                               uint32_t flags) {
    (void)source; (void)el; (void)events; (void)flags;
}

static inline eventmask_t chEvtWaitAnyTimeout(eventmask_t events, sysinterval_t timeout) {
    (void)events; (void)timeout;
    return 0;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_H_COMPAT */
