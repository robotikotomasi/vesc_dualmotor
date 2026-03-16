Import("env")

# STM32F1xx HAL Driver sources
env.BuildSources(
    "$BUILD_DIR/HAL_Driver",
    "$PROJECT_DIR/lib/STM32F1xx_HAL_Driver/Src",
    [
        "+<stm32f1xx_hal.c>",
        "+<stm32f1xx_hal_adc.c>",
        "+<stm32f1xx_hal_adc_ex.c>",
        "+<stm32f1xx_hal_can.c>",
        "+<stm32f1xx_hal_cortex.c>",
        "+<stm32f1xx_hal_crc.c>",
        "+<stm32f1xx_hal_dma.c>",
        "+<stm32f1xx_hal_flash.c>",
        "+<stm32f1xx_hal_flash_ex.c>",
        "+<stm32f1xx_hal_gpio.c>",
        "+<stm32f1xx_hal_gpio_ex.c>",
        "+<stm32f1xx_hal_i2c.c>",
        "+<stm32f1xx_hal_iwdg.c>",
        "+<stm32f1xx_hal_pwr.c>",
        "+<stm32f1xx_hal_rcc.c>",
        "+<stm32f1xx_hal_rcc_ex.c>",
        "+<stm32f1xx_hal_spi.c>",
        "+<stm32f1xx_hal_tim.c>",
        "+<stm32f1xx_hal_tim_ex.c>",
        "+<stm32f1xx_hal_uart.c>",
    ]
)

# FreeRTOS core
env.BuildSources(
    "$BUILD_DIR/FreeRTOS",
    "$PROJECT_DIR/lib/FreeRTOS/Source",
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

# Compatibility layer
env.BuildSources(
    "$BUILD_DIR/compat",
    "$PROJECT_DIR/compat",
    ["+<*.c>"]
)

# Main application sources
env.BuildSources(
    "$BUILD_DIR/app_src",
    "$PROJECT_DIR/Src",
    [
        "+<main.c>",
        "+<hw_setup.c>",
        "+<irq_handlers.c>",
        "+<events.c>",
        "+<timeout.c>",
        "+<terminal.c>",
        "+<flash_helper.c>",
        "+<conf_general.c>",
        "+<conf_custom.c>",
        "+<confgenerator.c>",
        "+<bms.c>",
    ]
)

# Motor control
env.BuildSources(
    "$BUILD_DIR/motor",
    "$PROJECT_DIR/Src/motor",
    [
        "+<mc_interface.c>",
        "+<mcpwm.c>",
        "+<mcpwm_foc.c>",
        "+<foc_math.c>",
        "+<virtual_motor.c>",
    ]
)

# Communication
env.BuildSources(
    "$BUILD_DIR/comm",
    "$PROJECT_DIR/Src/comm",
    [
        "+<commands.c>",
        "+<packet.c>",
        "+<log.c>",
        "+<comm_can.c>",
    ]
)

# Drivers
env.BuildSources(
    "$BUILD_DIR/driver",
    "$PROJECT_DIR/Src/driver",
    [
        "+<ledpwm.c>",
        "+<timer.c>",
        "+<eeprom.c>",
        "+<i2c_bb.c>",
        "+<spi_bb.c>",
        "+<servo_dec.c>",
        "+<pwm_servo.c>",
    ]
)

# NRF driver
env.BuildSources(
    "$BUILD_DIR/nrf",
    "$PROJECT_DIR/Src/driver/nrf",
    [
        "+<nrf_driver.c>",
        "+<rf.c>",
        "+<rfhelp.c>",
        "+<spi_sw.c>",
    ]
)

# Encoders
env.BuildSources(
    "$BUILD_DIR/encoder",
    "$PROJECT_DIR/Src/encoder",
    [
        "+<encoder.c>",
        "+<encoder_cfg.c>",
        "+<enc_abi.c>",
        "+<enc_ad2s1205.c>",
        "+<enc_as504x.c>",
        "+<enc_as5x47u.c>",
        "+<enc_bissc.c>",
        "+<enc_mt6816.c>",
        "+<enc_sincos.c>",
        "+<enc_tle5012.c>",
        "+<enc_ts5700n8501.c>",
        "+<enc_pwm.c>",
    ]
)

# IMU
env.BuildSources(
    "$BUILD_DIR/imu",
    "$PROJECT_DIR/Src/imu",
    [
        "+<imu.c>",
        "+<ahrs.c>",
        "+<mpu9150.c>",
        "+<bmi160_wrapper.c>",
        "+<icm20948.c>",
        "+<lsm6ds3.c>",
    ]
)

env.BuildSources(
    "$BUILD_DIR/imu_bmi",
    "$PROJECT_DIR/Src/imu/BMI160_driver",
    ["+<bmi160.c>"]
)

env.BuildSources(
    "$BUILD_DIR/imu_fusion",
    "$PROJECT_DIR/Src/imu/Fusion",
    [
        "+<FusionAhrs.c>",
        "+<FusionBias.c>",
        "+<FusionCompass.c>",
    ]
)

# Utilities
env.BuildSources(
    "$BUILD_DIR/util",
    "$PROJECT_DIR/Src/util",
    [
        "+<buffer.c>",
        "+<crc.c>",
        "+<digital_filter.c>",
        "+<mempools.c>",
        "+<utils_math.c>",
        "+<utils_sys.c>",
        "+<worker.c>",
    ]
)

env.BuildSources(
    "$BUILD_DIR/lzo",
    "$PROJECT_DIR/Src/util/lzo",
    ["+<minilzo.c>"]
)

# Applications
env.BuildSources(
    "$BUILD_DIR/applications",
    "$PROJECT_DIR/Src/applications",
    [
        "+<app.c>",
        "+<app_adc.c>",
        "+<app_ppm.c>",
        "+<app_nunchuk.c>",
        "+<app_uartcomm.c>",
        "+<app_pas.c>",
        "+<app_custom.c>",
        "+<app_custom_template.c>",
    ]
)

# libcanard
env.BuildSources(
    "$BUILD_DIR/canard",
    "$PROJECT_DIR/Src/libcanard",
    [
        "+<canard.c>",
        "+<canard_driver.c>",
    ]
)

env.BuildSources(
    "$BUILD_DIR/canard_dsdl",
    "$PROJECT_DIR/Src/libcanard/dsdl/vesc",
    ["+<vesc_RTData.c>"]
)

# QMLUI
env.BuildSources(
    "$BUILD_DIR/qmlui",
    "$PROJECT_DIR/Src/qmlui",
    ["+<qmlui.c>"]
)

env.BuildSources(
    "$BUILD_DIR/qmlui_app",
    "$PROJECT_DIR/Src/qmlui/app",
    ["+<qmlui_example_app.c>"]
)

env.BuildSources(
    "$BUILD_DIR/qmlui_hw",
    "$PROJECT_DIR/Src/qmlui/hw",
    ["+<qmlui_example_hw.c>"]
)

# Hardware config (from hwconf)
env.BuildSources(
    "$BUILD_DIR/hwconf",
    "$PROJECT_DIR/Src/hwconf",
    [
        "+<hw.c>",
        "+<shutdown.c>",
    ]
)
