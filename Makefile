######################################
# Hoverboard ESC Dual Motor - STM32F103RCT6
# VESC firmware with HAL/CMSIS + FreeRTOS RTOS v2
######################################

######################################
# target
######################################
TARGET = hoverboard_esc

######################################
# building variables
######################################
DEBUG = 0
OPT = -Os

#######################################
# paths
#######################################
BUILD_DIR = build

######################################
# source
######################################
# C sources

# CMSIS startup / system
C_SOURCES = \
lib/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c

# STM32F1xx HAL Driver
C_SOURCES += \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_crc.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_iwdg.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
lib/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c

# FreeRTOS
C_SOURCES += \
lib/FreeRTOS/Source/tasks.c \
lib/FreeRTOS/Source/queue.c \
lib/FreeRTOS/Source/list.c \
lib/FreeRTOS/Source/timers.c \
lib/FreeRTOS/Source/event_groups.c \
lib/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c \
lib/FreeRTOS/Source/portable/MemMang/heap_4.c \
lib/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c

# Compatibility layer
C_SOURCES += \
compat/compat_impl.c

# Main application sources
C_SOURCES += \
Src/main.c \
Src/hw_setup.c \
Src/irq_handlers.c \
Src/events.c \
Src/timeout.c \
Src/terminal.c \
Src/flash_helper.c \
Src/conf_general.c \
Src/conf_custom.c \
Src/confgenerator.c \
Src/bms.c

# Motor control
C_SOURCES += \
Src/motor/mc_interface.c \
Src/motor/mcpwm.c \
Src/motor/mcpwm_foc.c \
Src/motor/foc_math.c \
Src/motor/virtual_motor.c

# Communication
C_SOURCES += \
Src/comm/commands.c \
Src/comm/packet.c \
Src/comm/log.c \
Src/comm/comm_can.c

# Drivers
C_SOURCES += \
Src/driver/ledpwm.c \
Src/driver/timer.c \
Src/driver/eeprom.c \
Src/driver/i2c_bb.c \
Src/driver/spi_bb.c \
Src/driver/servo_dec.c \
Src/driver/pwm_servo.c

# NRF driver
C_SOURCES += \
Src/driver/nrf/nrf_driver.c \
Src/driver/nrf/rf.c \
Src/driver/nrf/rfhelp.c \
Src/driver/nrf/spi_sw.c

# Encoders
C_SOURCES += \
Src/encoder/encoder.c \
Src/encoder/encoder_cfg.c \
Src/encoder/enc_abi.c \
Src/encoder/enc_ad2s1205.c \
Src/encoder/enc_as504x.c \
Src/encoder/enc_as5x47u.c \
Src/encoder/enc_bissc.c \
Src/encoder/enc_mt6816.c \
Src/encoder/enc_sincos.c \
Src/encoder/enc_tle5012.c \
Src/encoder/enc_ts5700n8501.c \
Src/encoder/enc_pwm.c

# IMU
C_SOURCES += \
Src/imu/imu.c \
Src/imu/ahrs.c \
Src/imu/mpu9150.c \
Src/imu/bmi160_wrapper.c \
Src/imu/icm20948.c \
Src/imu/lsm6ds3.c \
Src/imu/BMI160_driver/bmi160.c \
Src/imu/Fusion/FusionAhrs.c \
Src/imu/Fusion/FusionBias.c \
Src/imu/Fusion/FusionCompass.c

# Utilities
C_SOURCES += \
Src/util/buffer.c \
Src/util/crc.c \
Src/util/digital_filter.c \
Src/util/mempools.c \
Src/util/utils_math.c \
Src/util/utils_sys.c \
Src/util/worker.c \
Src/util/lzo/minilzo.c

# Applications
C_SOURCES += \
Src/applications/app.c \
Src/applications/app_adc.c \
Src/applications/app_ppm.c \
Src/applications/app_nunchuk.c \
Src/applications/app_uartcomm.c \
Src/applications/app_pas.c \
Src/applications/app_custom.c \
Src/applications/app_custom_template.c

# libcanard
C_SOURCES += \
Src/libcanard/canard.c \
Src/libcanard/canard_driver.c \
Src/libcanard/dsdl/vesc/vesc_RTData.c

# QMLUI
C_SOURCES += \
Src/qmlui/qmlui.c \
Src/qmlui/app/qmlui_example_app.c \
Src/qmlui/hw/qmlui_example_hw.c

# Hardware config (from hwconf)
C_SOURCES += \
Src/hwconf/hw.c

# ASM sources
ASM_SOURCES = \
lib/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xe.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# (STM32F103 has no FPU)

# float-abi

# mcu
MCU = $(CPU) -mthumb

# macros for gcc
AS_DEFS =

C_DEFS = \
-DSTM32F103xE \
-DUSE_HAL_DRIVER \
-DHOVERBOARD_DUAL_MOTOR \
-DHW_HAS_DUAL_MOTORS \
-DHW_HEADER=\"hw_hoverboard.h\" \
-DCOMM_USE_USB=0 \
-DCAN_ENABLE=0 \
-DAPPCONF_CONTROLLER_ID=0 \
-DHW_SOURCE=\"hw_setup.c\"

# includes (compat MUST come FIRST to override ChibiOS headers)
C_INCLUDES = \
-Icompat \
-IInc \
-ISrc \
-ISrc/hwconf \
-ISrc/motor \
-ISrc/comm \
-ISrc/driver \
-ISrc/driver/nrf \
-ISrc/driver/lora \
-ISrc/encoder \
-ISrc/imu \
-ISrc/imu/BMI160_driver \
-ISrc/imu/Fusion \
-ISrc/util \
-ISrc/util/lzo \
-ISrc/applications \
-ISrc/libcanard \
-ISrc/libcanard/dsdl \
-ISrc/libcanard/dsdl/vesc \
-ISrc/qmlui \
-ISrc/qmlui/app \
-ISrc/qmlui/hw \
-ISrc/blackmagic \
-Ilib/STM32F1xx_HAL_Driver/Inc \
-Ilib/STM32F1xx_HAL_Driver/Inc/Legacy \
-Ilib/FreeRTOS/Source/include \
-Ilib/FreeRTOS/Source/portable/GCC/ARM_CM3 \
-Ilib/FreeRTOS/Source/CMSIS_RTOS_V2 \
-Ilib/CMSIS/Core/Include \
-Ilib/CMSIS/Device/ST/STM32F1xx/Include

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections \
-Wno-unused-parameter \
-Wno-missing-field-initializers \
-Wno-unused-function \
-Wno-unused-variable \
-Wno-discarded-qualifiers \
-Wno-sign-compare \
-Wno-implicit-function-declaration \
-Wno-pointer-sign \
-Wno-incompatible-pointer-types \
-Wno-int-conversion \
-Wno-format \
-Wno-unused-but-set-variable \
-Wno-maybe-uninitialized \
-Wno-type-limits \
-Wno-old-style-declaration \
-Wno-address

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103RCTX_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
-Wl,--gc-sections \
-Wl,--no-warn-rwx-segment

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
