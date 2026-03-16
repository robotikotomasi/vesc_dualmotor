Import("env")

# Add extra source files from libraries
env.BuildSources(
    "$BUILD_DIR/HAL_Driver",
    "$PROJECT_DIR/library/STM32F1xx_HAL_Driver/Src",
    [
        "+<stm32f1xx_hal.c>",
        "+<stm32f1xx_hal_adc.c>",
        "+<stm32f1xx_hal_adc_ex.c>",
        "+<stm32f1xx_hal_cortex.c>",
        "+<stm32f1xx_hal_dma.c>",
        "+<stm32f1xx_hal_flash.c>",
        "+<stm32f1xx_hal_flash_ex.c>",
        "+<stm32f1xx_hal_gpio.c>",
        "+<stm32f1xx_hal_gpio_ex.c>",
        "+<stm32f1xx_hal_iwdg.c>",
        "+<stm32f1xx_hal_pwr.c>",
        "+<stm32f1xx_hal_rcc.c>",
        "+<stm32f1xx_hal_rcc_ex.c>",
        "+<stm32f1xx_hal_tim.c>",
        "+<stm32f1xx_hal_tim_ex.c>",
        "+<stm32f1xx_hal_uart.c>",
    ]
)

# FreeRTOS core
env.BuildSources(
    "$BUILD_DIR/FreeRTOS",
    "$PROJECT_DIR/library/FreeRTOS/Source",
    [
        "+<tasks.c>",
        "+<queue.c>",
        "+<list.c>",
        "+<timers.c>",
        "+<event_groups.c>",
        "+<portable/GCC/ARM_CM3/port.c>",
        "+<portable/MemMang/heap_4.c>",
        "+<CMSIS_RTOS_V2/cmsis_os2.c>",
    ]
)
