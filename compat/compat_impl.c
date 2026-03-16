/*
 * Compatibility implementations
 * Provides stub implementations for ChibiOS HAL driver instances
 * and other compatibility functions.
 */

#include "ch.h"
#include "hal.h"
#include "shutdown.h"
#include <string.h>

/* ========================================================================
 * CAN driver instances
 * ======================================================================== */
CANDriver can_driver_1;
CANDriver can_driver_2;

/* ========================================================================
 * USB driver instances
 * ======================================================================== */
SerialUSBDriver SDU1;
USBDriver usb_driver_1;

/* ========================================================================
 * Serial driver instances
 * ======================================================================== */
SerialDriver sd1;
SerialDriver sd2;
SerialDriver sd3;

/* ========================================================================
 * SPI driver instances
 * ======================================================================== */
SPIDriver spid1;
SPIDriver spid2;

/* ========================================================================
 * I2C driver instances
 * ======================================================================== */
I2CDriver i2cd1;
I2CDriver i2cd2;

/* ========================================================================
 * ICU driver instances
 * ======================================================================== */
ICUDriver icud3;
ICUDriver icud4;

/* ========================================================================
 * ADC value array for DMA
 * ======================================================================== */
volatile uint16_t ADC_Value[16] = {0};

/* ========================================================================
 * Shutdown stubs
 * ======================================================================== */
void shutdown_init(void) {
}

void shutdown_reset_timer(void) {
}

bool shutdown_button_pressed(void) {
    return false;
}

float shutdown_get_inactivity_time(void) {
    return 0.0f;
}

void do_shutdown(void) {
}

/* ========================================================================
 * Hardware setup stubs (called from hw.h defaults)
 * ======================================================================== */
void hw_setup_adc_channels(void) {
}

void hw_start_i2c(void) {
}

void hw_stop_i2c(void) {
}

void hw_try_restore_i2c(void) {
}
