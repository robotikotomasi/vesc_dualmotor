/*
 * STM32F4xx Standard Peripheral Library → STM32F1xx HAL compatibility
 * Provides type definitions and constant mappings for VESC motor control code.
 */

#ifndef SPL_COMPAT_H
#define SPL_COMPAT_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * TIM Type Mappings (SPL → HAL)
 * ======================================================================== */

/* ========================================================================
 * TIM Type Mappings (SPL → HAL)
 * SPL uses TIM_Prescaler, HAL uses Prescaler, etc.
 * We must define our own structs with SPL-compatible member names.
 * ======================================================================== */

typedef struct {
    uint16_t TIM_Prescaler;
    uint16_t TIM_CounterMode;
    uint32_t TIM_Period;
    uint16_t TIM_ClockDivision;
    uint8_t  TIM_RepetitionCounter;
} TIM_TimeBaseInitTypeDef;

typedef struct {
    uint16_t TIM_OCMode;
    uint16_t TIM_OutputState;
    uint16_t TIM_OutputNState;
    uint32_t TIM_Pulse;
    uint16_t TIM_OCPolarity;
    uint16_t TIM_OCNPolarity;
    uint16_t TIM_OCIdleState;
    uint16_t TIM_OCNIdleState;
} TIM_OCInitTypeDef;

/* TIM BDTR InitTypeDef (SPL structure, map fields manually) */
typedef struct {
    uint16_t TIM_OSSRState;
    uint16_t TIM_OSSIState;
    uint16_t TIM_LOCKLevel;
    uint16_t TIM_DeadTime;
    uint16_t TIM_Break;
    uint16_t TIM_BreakPolarity;
    uint16_t TIM_AutomaticOutput;
} TIM_BDTRInitTypeDef;

/* TIM TimeBase init member mappings: SPL member names → HAL member names */
/* SPL uses TIM_Prescaler, HAL uses Prescaler, etc. */
/* We use macros to redirect SPL-style member access */

/* TIM TimeBase constants */
#ifndef TIM_CounterMode_Up
#define TIM_CounterMode_Up          TIM_COUNTERMODE_UP
#endif
#ifndef TIM_CounterMode_Down
#define TIM_CounterMode_Down        TIM_COUNTERMODE_DOWN
#endif
#ifndef TIM_CounterMode_CenterAligned1
#define TIM_CounterMode_CenterAligned1  TIM_COUNTERMODE_CENTERALIGNED1
#endif
#ifndef TIM_CounterMode_CenterAligned2
#define TIM_CounterMode_CenterAligned2  TIM_COUNTERMODE_CENTERALIGNED2
#endif
#ifndef TIM_CounterMode_CenterAligned3
#define TIM_CounterMode_CenterAligned3  TIM_COUNTERMODE_CENTERALIGNED3
#endif
#ifndef TIM_CKD_DIV1
#define TIM_CKD_DIV1                TIM_CLOCKDIVISION_DIV1
#endif

/* TIM OC constants */
#ifndef TIM_OutputState_Enable
#define TIM_OutputState_Enable      TIM_OUTPUTSTATE_ENABLE
#endif
#ifndef TIM_OutputState_Disable
#define TIM_OutputState_Disable     TIM_OUTPUTSTATE_DISABLE
#endif
#ifndef TIM_OutputNState_Enable
#define TIM_OutputNState_Enable     TIM_OUTPUTNSTATE_ENABLE
#endif
#ifndef TIM_OutputNState_Disable
#define TIM_OutputNState_Disable    TIM_OUTPUTNSTATE_DISABLE
#endif
#ifndef TIM_OCPolarity_High
#define TIM_OCPolarity_High         TIM_OCPOLARITY_HIGH
#endif
#ifndef TIM_OCPolarity_Low
#define TIM_OCPolarity_Low          TIM_OCPOLARITY_LOW
#endif
#ifndef TIM_OCNPolarity_High
#define TIM_OCNPolarity_High        TIM_OCNPOLARITY_HIGH
#endif
#ifndef TIM_OCNPolarity_Low
#define TIM_OCNPolarity_Low         TIM_OCNPOLARITY_LOW
#endif
#ifndef TIM_OCIdleState_Set
#define TIM_OCIdleState_Set         TIM_OCIDLESTATE_SET
#endif
#ifndef TIM_OCIdleState_Reset
#define TIM_OCIdleState_Reset       TIM_OCIDLESTATE_RESET
#endif
#ifndef TIM_OCNIdleState_Set
#define TIM_OCNIdleState_Set        TIM_OCNIDLESTATE_SET
#endif
#ifndef TIM_OCNIdleState_Reset
#define TIM_OCNIdleState_Reset      TIM_OCNIDLESTATE_RESET
#endif
#ifndef TIM_OCMode_Inactive
#define TIM_OCMode_Inactive         TIM_OCMODE_INACTIVE
#endif
#ifndef TIM_OCPreload_Enable
#define TIM_OCPreload_Enable        TIM_CCMR1_OC1PE
#endif
#ifndef TIM_OCPreload_Disable
#define TIM_OCPreload_Disable       0x0000U
#endif

/* BDTR constants */
#ifndef TIM_OSSRState_Enable
#define TIM_OSSRState_Enable        TIM_OSSR_ENABLE
#endif
#ifndef TIM_OSSRState_Disable
#define TIM_OSSRState_Disable       TIM_OSSR_DISABLE
#endif
#ifndef TIM_OSSIState_Enable
#define TIM_OSSIState_Enable        TIM_OSSI_ENABLE
#endif
#ifndef TIM_OSSIState_Disable
#define TIM_OSSIState_Disable       TIM_OSSI_DISABLE
#endif
#ifndef TIM_LOCKLevel_OFF
#define TIM_LOCKLevel_OFF           TIM_LOCKLEVEL_OFF
#endif
#ifndef TIM_Break_Disable
#define TIM_Break_Disable           TIM_BREAK_DISABLE
#endif
#ifndef TIM_Break_Enable
#define TIM_Break_Enable            TIM_BREAK_ENABLE
#endif
#ifndef TIM_BreakPolarity_High
#define TIM_BreakPolarity_High      TIM_BREAKPOLARITY_HIGH
#endif
#ifndef TIM_BreakPolarity_Low
#define TIM_BreakPolarity_Low       TIM_BREAKPOLARITY_LOW
#endif
#ifndef TIM_AutomaticOutput_Enable
#define TIM_AutomaticOutput_Enable  TIM_AUTOMATICOUTPUT_ENABLE
#endif
#ifndef TIM_AutomaticOutput_Disable
#define TIM_AutomaticOutput_Disable TIM_AUTOMATICOUTPUT_DISABLE
#endif

/* ========================================================================
 * TIM SPL-style init functions
 * ======================================================================== */

static inline void TIM_TimeBaseInit(TIM_TypeDef *TIMx, TIM_TimeBaseInitTypeDef *init) {
    TIM_HandleTypeDef htim;
    memset(&htim, 0, sizeof(htim));
    htim.Instance = TIMx;
    htim.Init.Prescaler = init->TIM_Prescaler;
    htim.Init.CounterMode = init->TIM_CounterMode;
    htim.Init.Period = init->TIM_Period;
    htim.Init.ClockDivision = init->TIM_ClockDivision;
    htim.Init.RepetitionCounter = init->TIM_RepetitionCounter;
    HAL_TIM_Base_Init(&htim);
}

static inline void TIM_OC1Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *init) {
    TIM_HandleTypeDef htim;
    TIM_OC_InitTypeDef oc;
    memset(&htim, 0, sizeof(htim));
    memset(&oc, 0, sizeof(oc));
    htim.Instance = TIMx;
    oc.OCMode = init->TIM_OCMode;
    oc.Pulse = init->TIM_Pulse;
    oc.OCPolarity = init->TIM_OCPolarity;
    oc.OCNPolarity = init->TIM_OCNPolarity;
    oc.OCIdleState = init->TIM_OCIdleState;
    oc.OCNIdleState = init->TIM_OCNIdleState;
    HAL_TIM_OC_ConfigChannel(&htim, &oc, TIM_CHANNEL_1);
}

static inline void TIM_OC2Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *init) {
    TIM_HandleTypeDef htim;
    TIM_OC_InitTypeDef oc;
    memset(&htim, 0, sizeof(htim));
    memset(&oc, 0, sizeof(oc));
    htim.Instance = TIMx;
    oc.OCMode = init->TIM_OCMode;
    oc.Pulse = init->TIM_Pulse;
    oc.OCPolarity = init->TIM_OCPolarity;
    oc.OCNPolarity = init->TIM_OCNPolarity;
    oc.OCIdleState = init->TIM_OCIdleState;
    oc.OCNIdleState = init->TIM_OCNIdleState;
    HAL_TIM_OC_ConfigChannel(&htim, &oc, TIM_CHANNEL_2);
}

static inline void TIM_OC3Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *init) {
    TIM_HandleTypeDef htim;
    TIM_OC_InitTypeDef oc;
    memset(&htim, 0, sizeof(htim));
    memset(&oc, 0, sizeof(oc));
    htim.Instance = TIMx;
    oc.OCMode = init->TIM_OCMode;
    oc.Pulse = init->TIM_Pulse;
    oc.OCPolarity = init->TIM_OCPolarity;
    oc.OCNPolarity = init->TIM_OCNPolarity;
    oc.OCIdleState = init->TIM_OCIdleState;
    oc.OCNIdleState = init->TIM_OCNIdleState;
    HAL_TIM_OC_ConfigChannel(&htim, &oc, TIM_CHANNEL_3);
}

static inline void TIM_OC4Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *init) {
    TIM_HandleTypeDef htim;
    TIM_OC_InitTypeDef oc;
    memset(&htim, 0, sizeof(htim));
    memset(&oc, 0, sizeof(oc));
    htim.Instance = TIMx;
    oc.OCMode = init->TIM_OCMode;
    oc.Pulse = init->TIM_Pulse;
    oc.OCPolarity = init->TIM_OCPolarity;
    oc.OCNPolarity = init->TIM_OCNPolarity;
    oc.OCIdleState = init->TIM_OCIdleState;
    oc.OCNIdleState = init->TIM_OCNIdleState;
    HAL_TIM_OC_ConfigChannel(&htim, &oc, TIM_CHANNEL_4);
}

static inline void TIM_OC1PreloadConfig(TIM_TypeDef *TIMx, uint16_t preload) {
    if (preload == TIM_OCPreload_Enable)
        TIMx->CCMR1 |= TIM_CCMR1_OC1PE;
    else
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1PE;
}

static inline void TIM_OC2PreloadConfig(TIM_TypeDef *TIMx, uint16_t preload) {
    if (preload == TIM_OCPreload_Enable)
        TIMx->CCMR1 |= TIM_CCMR1_OC2PE;
    else
        TIMx->CCMR1 &= ~TIM_CCMR1_OC2PE;
}

static inline void TIM_OC3PreloadConfig(TIM_TypeDef *TIMx, uint16_t preload) {
    if (preload == TIM_OCPreload_Enable)
        TIMx->CCMR2 |= TIM_CCMR2_OC3PE;
    else
        TIMx->CCMR2 &= ~TIM_CCMR2_OC3PE;
}

static inline void TIM_OC4PreloadConfig(TIM_TypeDef *TIMx, uint16_t preload) {
    if (preload == TIM_OCPreload_Enable)
        TIMx->CCMR2 |= TIM_CCMR2_OC4PE;
    else
        TIMx->CCMR2 &= ~TIM_CCMR2_OC4PE;
}

static inline void TIM_BDTRConfig(TIM_TypeDef *TIMx, TIM_BDTRInitTypeDef *init) {
    uint32_t bdtr = 0;
    bdtr |= init->TIM_DeadTime;
    bdtr |= (uint32_t)init->TIM_LOCKLevel << 8;
    bdtr |= (uint32_t)init->TIM_OSSIState;
    bdtr |= (uint32_t)init->TIM_OSSRState;
    bdtr |= (uint32_t)init->TIM_Break;
    bdtr |= (uint32_t)init->TIM_BreakPolarity;
    bdtr |= (uint32_t)init->TIM_AutomaticOutput;
    TIMx->BDTR = bdtr;
}

static inline void TIM_ARRPreloadConfig(TIM_TypeDef *TIMx, int state) {
    if (state) TIMx->CR1 |= TIM_CR1_ARPE;
    else TIMx->CR1 &= ~TIM_CR1_ARPE;
}

static inline void TIM_InternalClockConfig(TIM_TypeDef *TIMx) {
    TIMx->SMCR &= ~TIM_SMCR_SMS;
}

static inline void TIM_SetCounter(TIM_TypeDef *TIMx, uint32_t val) {
    TIMx->CNT = val;
}

static inline void TIM_SelectOutputTrigger(TIM_TypeDef *TIMx, uint16_t source) {
    TIMx->CR2 &= ~TIM_CR2_MMS;
    TIMx->CR2 |= source;
}

static inline void TIM_SelectMasterSlaveMode(TIM_TypeDef *TIMx, uint16_t mode) {
    TIMx->SMCR &= ~TIM_SMCR_MSM;
    TIMx->SMCR |= mode;
}

static inline void TIM_SelectSlaveMode(TIM_TypeDef *TIMx, uint16_t mode) {
    TIMx->SMCR &= ~TIM_SMCR_SMS;
    TIMx->SMCR |= mode;
}

static inline void TIM_SelectInputTrigger(TIM_TypeDef *TIMx, uint16_t source) {
    TIMx->SMCR &= ~TIM_SMCR_TS;
    TIMx->SMCR |= source;
}

static inline void TIM_SetCompare1(TIM_TypeDef *TIMx, uint32_t val) { TIMx->CCR1 = val; }
static inline void TIM_SetCompare2(TIM_TypeDef *TIMx, uint32_t val) { TIMx->CCR2 = val; }
static inline void TIM_SetCompare3(TIM_TypeDef *TIMx, uint32_t val) { TIMx->CCR3 = val; }
static inline void TIM_SetCompare4(TIM_TypeDef *TIMx, uint32_t val) { TIMx->CCR4 = val; }

static inline uint32_t TIM_GetCapture1(TIM_TypeDef *TIMx) { return TIMx->CCR1; }
static inline uint32_t TIM_GetCapture2(TIM_TypeDef *TIMx) { return TIMx->CCR2; }
static inline uint32_t TIM_GetCapture3(TIM_TypeDef *TIMx) { return TIMx->CCR3; }
static inline uint32_t TIM_GetCapture4(TIM_TypeDef *TIMx) { return TIMx->CCR4; }

static inline void TIM_GenerateEvent(TIM_TypeDef *TIMx, uint16_t event) {
    TIMx->EGR = event;
}

/* TIM TRGO source defines */
#ifndef TIM_TRGOSource_Update
#define TIM_TRGOSource_Update       TIM_TRGO_UPDATE
#endif
#ifndef TIM_TRGOSource_Reset
#define TIM_TRGOSource_Reset        TIM_TRGO_RESET
#endif
#ifndef TIM_TRGOSource_Enable
#define TIM_TRGOSource_Enable       TIM_TRGO_ENABLE
#endif

/* TIM Master Slave Mode */
#ifndef TIM_MasterSlaveMode_Enable
#define TIM_MasterSlaveMode_Enable  TIM_SMCR_MSM
#endif
#ifndef TIM_MasterSlaveMode_Disable
#define TIM_MasterSlaveMode_Disable 0x0000
#endif

/* TIM Slave Mode */
#ifndef TIM_SlaveMode_Reset
#define TIM_SlaveMode_Reset         TIM_SLAVEMODE_RESET
#endif
#ifndef TIM_SlaveMode_Gated
#define TIM_SlaveMode_Gated         TIM_SLAVEMODE_GATED
#endif
#ifndef TIM_SlaveMode_Trigger
#define TIM_SlaveMode_Trigger       TIM_SLAVEMODE_TRIGGER
#endif
#ifndef TIM_SlaveMode_External1
#define TIM_SlaveMode_External1     TIM_SLAVEMODE_EXTERNAL1
#endif

/* TIM input trigger source */
#ifndef TIM_TS_ITR0
#define TIM_TS_ITR0                 TIM_TS_ITR0
#endif

/* TIM PSC reload mode */
#ifndef TIM_PSCReloadMode_Immediate
#define TIM_PSCReloadMode_Immediate 0x0000
#endif
#ifndef TIM_PSCReloadMode_Update
#define TIM_PSCReloadMode_Update    0x0001
#endif

/* TIM Encoder Mode */
#ifndef TIM_EncoderMode_TI1
#define TIM_EncoderMode_TI1         TIM_ENCODERMODE_TI1
#endif
#ifndef TIM_EncoderMode_TI2
#define TIM_EncoderMode_TI2         TIM_ENCODERMODE_TI2
#endif
#ifndef TIM_EncoderMode_TI12
#define TIM_EncoderMode_TI12        TIM_ENCODERMODE_TI12
#endif

/* TIM IC Polarity */
#ifndef TIM_ICPolarity_Rising
#define TIM_ICPolarity_Rising       TIM_ICPOLARITY_RISING
#endif
#ifndef TIM_ICPolarity_Falling
#define TIM_ICPolarity_Falling      TIM_ICPOLARITY_FALLING
#endif
#ifndef TIM_ICPolarity_BothEdge
#define TIM_ICPolarity_BothEdge     TIM_ICPOLARITY_BOTHEDGE
#endif

static inline void TIM_EncoderInterfaceConfig(TIM_TypeDef *TIMx, uint32_t mode,
                                               uint32_t ic1pol, uint32_t ic2pol) {
    TIM_HandleTypeDef htim;
    TIM_Encoder_InitTypeDef enc;
    memset(&htim, 0, sizeof(htim));
    memset(&enc, 0, sizeof(enc));
    htim.Instance = TIMx;
    enc.EncoderMode = mode;
    enc.IC1Polarity = ic1pol;
    enc.IC2Polarity = ic2pol;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 0;
    enc.IC2Filter = 0;
    HAL_TIM_Encoder_Init(&htim, &enc);
}

static inline void TIM_PrescalerConfig(TIM_TypeDef *TIMx, uint16_t prescaler, uint16_t mode) {
    TIMx->PSC = prescaler;
    if (mode == TIM_PSCReloadMode_Immediate)
        TIMx->EGR |= TIM_EGR_UG;
}

/* RCC Peripheral clock defines for APB1 */
#ifndef RCC_APB1Periph_TIM2
#define RCC_APB1Periph_TIM2         0x00000001U
#endif
#ifndef RCC_APB1Periph_TIM4
#define RCC_APB1Periph_TIM4         0x00000004U
#endif
#ifndef RCC_APB1Periph_TIM5
#define RCC_APB1Periph_TIM5         0x00000008U
#endif
#ifndef RCC_APB1Periph_TIM12
#define RCC_APB1Periph_TIM12        0x00000040U
#endif

/* HW_ADC_INJ_CHANNELS default */
#ifndef HW_ADC_INJ_CHANNELS
#define HW_ADC_INJ_CHANNELS         3
#endif

/* ========================================================================
 * ADC SPL-style mappings
 * ======================================================================== */

/* ADC Common Init structure (F4 has multi-ADC, F1 doesn't) */
typedef struct {
    uint32_t ADC_Mode;
    uint32_t ADC_Prescaler;
    uint32_t ADC_DMAAccessMode;
    uint32_t ADC_TwoSamplingDelay;
} ADC_CommonInitTypeDef;

/* ADC Init structure (SPL-style, different from HAL ADC_InitTypeDef) */
typedef struct {
    uint32_t ADC_Resolution;
    uint32_t ADC_ScanConvMode;
    uint32_t ADC_ContinuousConvMode;
    uint32_t ADC_ExternalTrigConvEdge;
    uint32_t ADC_ExternalTrigConv;
    uint32_t ADC_DataAlign;
    uint32_t ADC_NbrOfConversion;
} ADC_SPL_InitTypeDef;

/* Override HAL's ADC_InitTypeDef in user code (HAL handles are already parsed) */
#define ADC_InitTypeDef ADC_SPL_InitTypeDef

/* ADC defines */
#ifndef ADC_Mode_Independent
#define ADC_Mode_Independent        0x00
#endif
#ifndef ADC_Prescaler_Div2
#define ADC_Prescaler_Div2          0x00
#endif
#ifndef ADC_DMAAccessMode_Disabled
#define ADC_DMAAccessMode_Disabled  0x00
#endif
#ifndef ADC_TwoSamplingDelay_5Cycles
#define ADC_TwoSamplingDelay_5Cycles  0x00
#endif
#ifndef ADC_Resolution_12b
#define ADC_Resolution_12b          0x00
#endif
#ifndef ADC_DataAlign_Right
#define ADC_DataAlign_Right         ADC_DATAALIGN_RIGHT
#endif
#ifndef ADC_DataAlign_Left
#define ADC_DataAlign_Left          ADC_DATAALIGN_LEFT
#endif
#ifndef ADC_SampleTime_15Cycles
#define ADC_SampleTime_15Cycles     ADC_SAMPLETIME_13CYCLES_5
#endif
#ifndef ADC_SampleTime_3Cycles
#define ADC_SampleTime_3Cycles      ADC_SAMPLETIME_1CYCLE_5
#endif
#ifndef ADC_SampleTime_84Cycles
#define ADC_SampleTime_84Cycles     ADC_SAMPLETIME_71CYCLES_5
#endif
#ifndef ADC_SampleTime_56Cycles
#define ADC_SampleTime_56Cycles     ADC_SAMPLETIME_55CYCLES_5
#endif
#ifndef ADC_SampleTime_28Cycles
#define ADC_SampleTime_28Cycles     ADC_SAMPLETIME_28CYCLES_5
#endif
#ifndef ADC_SampleTime_112Cycles
#define ADC_SampleTime_112Cycles    ADC_SAMPLETIME_71CYCLES_5
#endif
#ifndef ADC_ExternalTrigConvEdge_None
#define ADC_ExternalTrigConvEdge_None    0x00
#endif
#ifndef ADC_ExternalTrigConv_T1_CC1
#define ADC_ExternalTrigConv_T1_CC1      0x00
#endif

/* ADC Channel defines */
#ifndef ADC_Channel_0
#define ADC_Channel_0   ADC_CHANNEL_0
#define ADC_Channel_1   ADC_CHANNEL_1
#define ADC_Channel_2   ADC_CHANNEL_2
#define ADC_Channel_3   ADC_CHANNEL_3
#define ADC_Channel_4   ADC_CHANNEL_4
#define ADC_Channel_5   ADC_CHANNEL_5
#define ADC_Channel_6   ADC_CHANNEL_6
#define ADC_Channel_7   ADC_CHANNEL_7
#define ADC_Channel_8   ADC_CHANNEL_8
#define ADC_Channel_9   ADC_CHANNEL_9
#define ADC_Channel_10  ADC_CHANNEL_10
#define ADC_Channel_11  ADC_CHANNEL_11
#define ADC_Channel_12  ADC_CHANNEL_12
#define ADC_Channel_13  ADC_CHANNEL_13
#define ADC_Channel_14  ADC_CHANNEL_14
#define ADC_Channel_15  ADC_CHANNEL_15
#define ADC_Channel_Vrefint     ADC_CHANNEL_VREFINT
#define ADC_Channel_TempSensor  ADC_CHANNEL_TEMPSENSOR
#endif

/* ADC SPL function stubs */
static inline void ADC_CommonInit(ADC_CommonInitTypeDef *init) {
    (void)init;
}

static inline void ADC_Init(ADC_TypeDef *ADCx, ADC_SPL_InitTypeDef *init) {
    (void)ADCx; (void)init;
}

static inline void ADC_RegularChannelConfig(ADC_TypeDef *ADCx, uint32_t channel,
                                             uint32_t rank, uint32_t sampleTime) {
    (void)ADCx; (void)channel; (void)rank; (void)sampleTime;
}

static inline void ADC_InjectedChannelConfig(ADC_TypeDef *ADCx, uint32_t channel,
                                              uint32_t rank, uint32_t sampleTime) {
    (void)ADCx; (void)channel; (void)rank; (void)sampleTime;
}

static inline void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef *ADCx, uint32_t source) {
    (void)ADCx; (void)source;
}

static inline void ADC_ExternalTrigInjectedConvEdgeConfig(ADC_TypeDef *ADCx, uint32_t edge) {
    (void)ADCx; (void)edge;
}

static inline void ADC_InjectedSequencerLengthConfig(ADC_TypeDef *ADCx, uint32_t length) {
    (void)ADCx; (void)length;
}

static inline void ADC_SetInjectedOffset(ADC_TypeDef *ADCx, uint8_t channel, uint16_t offset) {
    (void)ADCx; (void)channel; (void)offset;
}

static inline void ADC_DMACmd(ADC_TypeDef *ADCx, int state) {
    if (state) ADCx->CR2 |= ADC_CR2_DMA;
    else ADCx->CR2 &= ~ADC_CR2_DMA;
}

static inline void ADC_DMARequestAfterLastTransferCmd(ADC_TypeDef *ADCx, int state) {
    (void)ADCx; (void)state;
}

static inline void ADC_MultiModeDMARequestAfterLastTransferCmd(int state) {
    (void)state;
}

static inline void ADC_SoftwareStartConv(ADC_TypeDef *ADCx) {
    ADCx->CR2 |= ADC_CR2_SWSTART;
}

static inline void ADC_SoftwareStartInjectedConv(ADC_TypeDef *ADCx) {
    ADCx->CR2 |= ADC_CR2_JSWSTART;
}

/* ADC External trigger injection source defines */
#ifndef ADC_ExternalTrigInjecConv_T1_CC4
#define ADC_ExternalTrigInjecConv_T1_CC4      0x00
#endif
#ifndef ADC_ExternalTrigInjecConv_T1_TRGO
#define ADC_ExternalTrigInjecConv_T1_TRGO     0x00
#endif
#ifndef ADC_ExternalTrigInjecConv_T2_CC1
#define ADC_ExternalTrigInjecConv_T2_CC1      0x00
#endif
#ifndef ADC_ExternalTrigInjecConv_T8_CC2
#define ADC_ExternalTrigInjecConv_T8_CC2      0x00
#endif
#ifndef ADC_ExternalTrigInjecConv_T8_CC3
#define ADC_ExternalTrigInjecConv_T8_CC3      0x00
#endif
#ifndef ADC_ExternalTrigInjecConvEdge_Rising
#define ADC_ExternalTrigInjecConvEdge_Rising   0x00
#endif
#ifndef ADC_ExternalTrigInjecConvEdge_Falling
#define ADC_ExternalTrigInjecConvEdge_Falling  0x00
#endif
#ifndef ADC_ExternalTrigConvEdge_Falling
#define ADC_ExternalTrigConvEdge_Falling       0x00
#endif
#ifndef ADC_ExternalTrigConv_T8_CC1
#define ADC_ExternalTrigConv_T8_CC1            0x00
#endif
#ifndef ADC_ExternalTrigConv_T2_CC2
#define ADC_ExternalTrigConv_T2_CC2            0x00
#endif
#ifndef ADC_TripleMode_RegSimult
#define ADC_TripleMode_RegSimult               0x00
#endif
#ifndef ADC_DMAAccessMode_1
#define ADC_DMAAccessMode_1                    0x00
#endif

/* ADC_IRQn on F1 is ADC1_2_IRQn */
#ifndef ADC_IRQn
#define ADC_IRQn        ADC1_2_IRQn
#endif

/* ADC_IT defines */
#ifndef ADC_IT_EOC
#define ADC_IT_EOC      ADC_CR1_EOCIE
#endif
#ifndef ADC_IT_JEOC
#define ADC_IT_JEOC     ADC_CR1_JEOCIE
#endif

/* Additional ADC function stubs */
static inline void ADC_Cmd(ADC_TypeDef *ADCx, int state) {
    if (state) ADCx->CR2 |= ADC_CR2_ADON;
    else ADCx->CR2 &= ~ADC_CR2_ADON;
}

static inline void ADC_ITConfig(ADC_TypeDef *ADCx, uint32_t it, int state) {
    if (state) ADCx->CR1 |= it;
    else ADCx->CR1 &= ~it;
}

static inline void ADC_ClearITPendingBit(ADC_TypeDef *ADCx, uint32_t it) {
    ADCx->SR &= ~it;
}

static inline uint32_t ADC_GetInjectedConversionValue(ADC_TypeDef *ADCx, uint8_t channel) {
    switch (channel) {
    case 1: return ADCx->JDR1;
    case 2: return ADCx->JDR2;
    case 3: return ADCx->JDR3;
    case 4: return ADCx->JDR4;
    default: return 0;
    }
}

#ifndef ADC_InjectedChannel_1
#define ADC_InjectedChannel_1   1
#define ADC_InjectedChannel_2   2
#define ADC_InjectedChannel_3   3
#define ADC_InjectedChannel_4   4
#endif

static inline void ADC_DeInit(void) { /* No-op stub */ }

static inline void ADC_TempSensorVrefintCmd(int state) {
    if (state) ADC1->CR2 |= ADC_CR2_TSVREFE;
    else ADC1->CR2 &= ~ADC_CR2_TSVREFE;
}

/* ========================================================================
 * DMA SPL-style mappings (F4 → F1)
 * STM32F4 uses streams, STM32F1 uses channels.
 * ======================================================================== */

/* DMA_InitTypeDef_SPL with F4-style members */
typedef struct {
    uint32_t DMA_Channel;
    uint32_t DMA_PeripheralBaseAddr;
    uint32_t DMA_Memory0BaseAddr;
    uint32_t DMA_DIR;
    uint32_t DMA_BufferSize;
    uint32_t DMA_PeripheralInc;
    uint32_t DMA_MemoryInc;
    uint32_t DMA_PeripheralDataSize;
    uint32_t DMA_MemoryDataSize;
    uint32_t DMA_Mode;
    uint32_t DMA_Priority;
    uint32_t DMA_FIFOMode;
    uint32_t DMA_FIFOThreshold;
    uint32_t DMA_MemoryBurst;
    uint32_t DMA_PeripheralBurst;
} DMA_InitTypeDef_SPL;

/* Override HAL's DMA_InitTypeDef in user code (HAL handles are already parsed) */
#define DMA_InitTypeDef DMA_InitTypeDef_SPL

/* DMA Channel defines (F4-style) */
#ifndef DMA_Channel_0
#define DMA_Channel_0   0x00000000U
#define DMA_Channel_1   0x02000000U
#define DMA_Channel_2   0x04000000U
#define DMA_Channel_3   0x06000000U
#endif

/* DMA Direction */
#ifndef DMA_DIR_PeripheralToMemory
#define DMA_DIR_PeripheralToMemory      0x00
#define DMA_DIR_MemoryToPeripheral      0x01
#define DMA_DIR_MemoryToMemory          0x02
#endif

/* DMA Inc/size/mode */
#ifndef DMA_PeripheralInc_Disable
#define DMA_PeripheralInc_Disable       0x00
#define DMA_PeripheralInc_Enable        0x01
#define DMA_MemoryInc_Disable           0x00
#define DMA_MemoryInc_Enable            0x01
#define DMA_PeripheralDataSize_HalfWord 0x01
#define DMA_PeripheralDataSize_Word     0x02
#define DMA_PeripheralDataSize_Byte     0x00
#define DMA_MemoryDataSize_HalfWord     0x01
#define DMA_MemoryDataSize_Word         0x02
#define DMA_MemoryDataSize_Byte         0x00
#define DMA_Mode_Circular               0x01
#define DMA_Mode_Normal                 0x00
#define DMA_Priority_High               0x02
#define DMA_Priority_Low                0x00
#define DMA_FIFOMode_Disable            0x00
#define DMA_FIFOMode_Enable             0x01
#define DMA_FIFOThreshold_1QuarterFull  0x00
#define DMA_FIFOThreshold_HalfFull      0x01
#define DMA_MemoryBurst_Single          0x00
#define DMA_PeripheralBurst_Single      0x00
#endif

/* DMA Stream defines → map to F1 Channel instances */
/* F4: DMA2_Stream0 etc., F1: DMA1_Channel1 etc. */
#ifndef DMA2_Stream0
#define DMA2_Stream0    DMA1_Channel1
#endif
#ifndef DMA2_Stream4
#define DMA2_Stream4    DMA1_Channel1
#endif

/* DMA F4-specific peripheral base */
#ifndef ADC_CDR_ADDRESS
#define ADC_CDR_ADDRESS     ((uint32_t)&(ADC1->DR))
#endif
/* ADC->CDR exists on F4 (common data register), map to ADC1->DR on F1 */
/* We redefine ADC to use our compat struct that has a CDR member at the right address */
typedef struct {
    uint32_t _pad[0x4C / 4]; /* Offset to match DR position */
    volatile uint32_t CDR;   /* Maps to ADC1->DR offset */
} ADC_Common_SPL_TypeDef;

#ifndef ADC
#define ADC   ((ADC_Common_SPL_TypeDef *)ADC1_BASE)
#endif

/* DMA function stubs */
static inline void DMA_Init(void *stream, DMA_InitTypeDef_SPL *init) {
    (void)stream; (void)init;
}

static inline void DMA_Cmd(void *stream, int state) {
    (void)stream; (void)state;
}

static inline void DMA_DeInit(void *stream) {
    (void)stream;
}

static inline void DMA_ITConfig(void *stream, uint32_t it, int state) {
    (void)stream; (void)it; (void)state;
}

/* DMA IT defines */
#ifndef DMA_IT_TC
#define DMA_IT_TC       0x00000002U
#endif
#ifndef DMA_IT_HT
#define DMA_IT_HT       0x00000004U
#endif

/* ========================================================================
 * RCC SPL-style peripheral clock defines
 * ======================================================================== */

#ifndef RCC_APB2Periph_TIM1
#define RCC_APB2Periph_TIM1     0x00000800U
#endif
#ifndef RCC_APB2Periph_TIM8
#define RCC_APB2Periph_TIM8     0x00002000U
#endif
#ifndef RCC_APB2Periph_ADC1
#define RCC_APB2Periph_ADC1     0x00000200U
#endif
#ifndef RCC_APB2Periph_ADC2
#define RCC_APB2Periph_ADC2     0x00000400U
#endif
#ifndef RCC_APB2Periph_ADC3
#define RCC_APB2Periph_ADC3     0x00008000U
#endif
#ifndef RCC_AHB1Periph_DMA2
#define RCC_AHB1Periph_DMA2     0x00400000U
#endif
#ifndef RCC_AHB1Periph_DMA1
#define RCC_AHB1Periph_DMA1     0x00200000U
#endif
#ifndef RCC_AHB1Periph_GPIOA
#define RCC_AHB1Periph_GPIOA    0x00000001U
#endif
#ifndef RCC_AHB1Periph_GPIOB
#define RCC_AHB1Periph_GPIOB    0x00000002U
#endif
#ifndef RCC_AHB1Periph_GPIOC
#define RCC_AHB1Periph_GPIOC    0x00000004U
#endif
#ifndef RCC_AHB1Periph_GPIOD
#define RCC_AHB1Periph_GPIOD    0x00000008U
#endif
#ifndef RCC_AHB1Periph_GPIOE
#define RCC_AHB1Periph_GPIOE    0x00000010U
#endif
#ifndef RCC_APB1Periph_SPI2
#define RCC_APB1Periph_SPI2     0x00004000U
#endif
#ifndef RCC_APB1Periph_SPI3
#define RCC_APB1Periph_SPI3     0x00008000U
#endif
#ifndef RCC_APB2Periph_SPI1
#define RCC_APB2Periph_SPI1     0x00001000U
#endif

/* ========================================================================
 * GPIO SPL-style mappings
 * ======================================================================== */

typedef struct {
    uint32_t GPIO_Pin;
    uint32_t GPIO_Mode;
    uint32_t GPIO_Speed;
    uint32_t GPIO_OType;
    uint32_t GPIO_PuPd;
} GPIO_InitTypeDef_SPL;

/* GPIO mode defines */
#ifndef GPIO_Mode_IN
#define GPIO_Mode_IN    0x00
#define GPIO_Mode_OUT   0x01
#define GPIO_Mode_AF    0x02
#define GPIO_Mode_AN    0x03
#endif

#ifndef GPIO_OType_PP
#define GPIO_OType_PP   0x00
#define GPIO_OType_OD   0x01
#endif

#ifndef GPIO_PuPd_NOPULL
#define GPIO_PuPd_NOPULL    0x00
#define GPIO_PuPd_UP        0x01
#define GPIO_PuPd_DOWN      0x02
#endif

#ifndef GPIO_Speed_25MHz
#define GPIO_Speed_2MHz     0x00
#define GPIO_Speed_25MHz    0x01
#define GPIO_Speed_50MHz    0x02
#define GPIO_Speed_100MHz   0x03
#endif

/* NVIC SPL-style init */
typedef struct {
    IRQn_Type NVIC_IRQChannel;
    uint8_t   NVIC_IRQChannelPreemptionPriority;
    uint8_t   NVIC_IRQChannelSubPriority;
    uint8_t   NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

static inline void NVIC_Init(NVIC_InitTypeDef *init) {
    if (init->NVIC_IRQChannelCmd) {
        NVIC_SetPriority(init->NVIC_IRQChannel,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                init->NVIC_IRQChannelPreemptionPriority,
                init->NVIC_IRQChannelSubPriority));
        NVIC_EnableIRQ(init->NVIC_IRQChannel);
    } else {
        NVIC_DisableIRQ(init->NVIC_IRQChannel);
    }
}

static inline void NVIC_PriorityGroupConfig(uint32_t group) {
    NVIC_SetPriorityGrouping(group);
}

#ifndef NVIC_PriorityGroup_4
#define NVIC_PriorityGroup_4    3U
#endif

/* ========================================================================
 * SPI SPL-style defines
 * ======================================================================== */

#ifndef SPI_Direction_2Lines_FullDuplex
#define SPI_Direction_2Lines_FullDuplex  0x0000
#endif
#ifndef SPI_Mode_Master
#define SPI_Mode_Master     0x0104
#endif
#ifndef SPI_DataSize_8b
#define SPI_DataSize_8b     SPI_DATASIZE_8BIT
#endif
#ifndef SPI_DataSize_16b
#define SPI_DataSize_16b    SPI_DATASIZE_16BIT
#endif
#ifndef SPI_CPOL_Low
#define SPI_CPOL_Low        0x0000
#endif
#ifndef SPI_CPOL_High
#define SPI_CPOL_High       0x0002
#endif
#ifndef SPI_CPHA_1Edge
#define SPI_CPHA_1Edge      0x0000
#endif
#ifndef SPI_CPHA_2Edge
#define SPI_CPHA_2Edge      0x0001
#endif
#ifndef SPI_NSS_Soft
#define SPI_NSS_Soft        SPI_NSS_SOFT
#endif
#ifndef SPI_BaudRatePrescaler_4
#define SPI_BaudRatePrescaler_4     SPI_BAUDRATEPRESCALER_4
#endif
#ifndef SPI_BaudRatePrescaler_16
#define SPI_BaudRatePrescaler_16    SPI_BAUDRATEPRESCALER_16
#endif
#ifndef SPI_FirstBit_MSB
#define SPI_FirstBit_MSB    SPI_FIRSTBIT_MSB
#endif

/* ========================================================================
 * DBGMCU
 * ======================================================================== */

static inline void DBGMCU_APB1PeriphConfig(uint32_t periph, int state) {
    (void)periph; (void)state;
}

static inline void DBGMCU_APB2PeriphConfig(uint32_t periph, int state) {
    (void)periph; (void)state;
}

#ifndef DBGMCU_TIM1_STOP
#define DBGMCU_TIM1_STOP    0x00000001U
#define DBGMCU_TIM2_STOP    0x00000001U
#define DBGMCU_TIM8_STOP    0x00000002U
#endif

/* ========================================================================
 * SYSCFG
 * ======================================================================== */

static inline void SYSCFG_EXTILineConfig(uint8_t port, uint8_t pin) {
    (void)port; (void)pin;
}

/* ========================================================================
 * GPIO Pin Source defines
 * ======================================================================== */

#ifndef GPIO_PinSource0
#define GPIO_PinSource0     0
#define GPIO_PinSource1     1
#define GPIO_PinSource2     2
#define GPIO_PinSource3     3
#define GPIO_PinSource4     4
#define GPIO_PinSource5     5
#define GPIO_PinSource6     6
#define GPIO_PinSource7     7
#define GPIO_PinSource8     8
#define GPIO_PinSource9     9
#define GPIO_PinSource10    10
#define GPIO_PinSource11    11
#define GPIO_PinSource12    12
#define GPIO_PinSource13    13
#define GPIO_PinSource14    14
#define GPIO_PinSource15    15
#endif

/* GPIO AF mapping function (F4-style, no-op on F1 since AF is via AFIO remap) */
static inline void GPIO_PinAFConfig(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t af) {
    (void)GPIOx; (void)pin; (void)af;
}

/* GPIO Pin defines (SPL-style: GPIO_Pin_0 etc. → already available via HAL) */

/* ========================================================================
 * GIT commit hash stub
 * ======================================================================== */

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH     "000000"
#endif

/* System clock frequency */
#ifndef SYSTEM_CORE_CLOCK
#define SYSTEM_CORE_CLOCK   72000000UL
#endif

/* ========================================================================
 * ADC V_ZERO
 * ======================================================================== */

#ifndef ADC_V_ZERO
#define ADC_V_ZERO          (4095.0f / 2.0f)
#endif

/* ========================================================================
 * CAN BTR bit-field macros (function-like, as used by VESC ChibiOS code)
 * CMSIS defines these as constants, VESC uses them as functions to shift values.
 * ======================================================================== */

#undef CAN_BTR_BRP
#undef CAN_BTR_TS1
#undef CAN_BTR_TS2
#undef CAN_BTR_SJW

#define CAN_BTR_BRP(n)      (((uint32_t)(n)) << CAN_BTR_BRP_Pos)
#define CAN_BTR_TS1(n)      (((uint32_t)(n)) << CAN_BTR_TS1_Pos)
#define CAN_BTR_TS2(n)      (((uint32_t)(n)) << CAN_BTR_TS2_Pos)
#define CAN_BTR_SJW(n)      (((uint32_t)(n)) << CAN_BTR_SJW_Pos)

/* ADC sensor indices (hall sensors) */
#ifndef ADC_IND_SENS1
#define ADC_IND_SENS1       0
#endif
#ifndef ADC_IND_SENS2
#define ADC_IND_SENS2       1
#endif
#ifndef ADC_IND_SENS3
#define ADC_IND_SENS3       2
#endif

#ifdef __cplusplus
}
#endif

#endif /* SPL_COMPAT_H */
