/*
 * This file is part of libgreat
 *
 * LPC43xx-specific I2C drivers
 */

#ifndef __LIBGREAT_PLATFORM_I2C_H__
#define __LIBGREAT_PLATFORM_I2C_H__

#include <toolchain.h>
#include <drivers/platform_clock.h>

typedef struct i2c i2c_t;

/**
 * I2C numbers for each supported UART on the LPC43xx.
 */
typedef enum {
	I2C0 = 0,
	I2C1 = 1,
} i2c_number_t;

/**
 * General I2C constants.
 */
enum {
	NUM_I2CS = 2
};


/**
 * Register layout for LPC43xx I2Cs.
 */
typedef volatile struct ATTR_PACKED {

	uint32_t control_set;
	uint32_t status;
	uint32_t data;
	uint32_t peripheral_address_0;
	uint32_t duty_cycle_high_half_word;
	uint32_t duty_cycle_low_half_word;
	uint32_t control_clear;
	uint32_t monitor_mode_control;
	uint32_t peripheral_address_1;
	uint32_t peripheral_address_2;
	uint32_t peripheral_address_3;
	uint32_t data_buffer
	uint32_t peripheral_address_mask_0;
	uint32_t peripheral_address_mask_1;
	uint32_t peripheral_address_mask_2;
	uint32_t peripheral_address_mask_3;

} platform_i2c_registers_t;


/**
 * Platform-specific, per-I2C-instance data.
 */
typedef struct {

	// The clock that controls the relevant I2C.
	platform_branch_clock_t *clock;

} platform_i2c_t;

#endif
