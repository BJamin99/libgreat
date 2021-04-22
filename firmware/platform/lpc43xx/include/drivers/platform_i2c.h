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

	//uint32_t control_set;
	struct {
		uint32_t                : 2;
		uint32_t assert_ack     : 1;
		uint32_t interrupt      : 1;
		uint32_t stop           : 1;
		uint32_t start          : 1;
        uint32_t i2c_enable     : 1;
        uint32_t                : 25
	};
	//uint32_t status;
	struct {
		uint32_t                : 3;
		uint32_t status         : 5;
		uint32_t                : 24
	};
	//uint32_t data;
	struct {
		uint32_t data           : 8;
		uint32_t                : 24
	};
	//uint32_t peripheral_address_0;
	struct {
		uint32_t general_call_0 : 1;
		uint32_t address_0      : 7;
		uint32_t                : 24
	};
	//uint32_t duty_cycle_high_half_word;
   	struct {
   		uint32_t duty_cycle_high_half_word : 16;
   		uint32_t                           : 16
   	};
	//uint32_t duty_cycle_low_half_word;
   	struct {
   		uint32_t duty_cycle_low_half_word  : 16;
   		uint32_t                           : 16
   	};
	//uint32_t control_clear;
	struct {
		uint32_t                : 2;
		uint32_t assert_ack_clr : 1;
		uint32_t interrupt_clr  : 1;
		uint32_t                : 1;
		uint32_t start_clr      : 1;
        uint32_t i2c_disable    : 1;
        uint32_t                : 25
	};
	//uint32_t monitor_mode_control;
	struct {
		uint32_t monitor_mode_enable : 1;
		uint32_t scl_output_enable   : 1;
		uint32_t interrupt_match     : 1;
		uint32_t                     : 29
	};
	//uint32_t peripheral_address_1;
	struct {
		uint32_t general_call_1 : 1;
		uint32_t address_1      : 7;
		uint32_t                : 24
	};
	//uint32_t peripheral_address_2;
	struct {
		uint32_t general_call_2 : 1;
		uint32_t address_2      : 7;
		uint32_t                : 24
	};
	//uint32_t peripheral_address_3;
	struct {
		uint32_t general_call_3 : 1;
		uint32_t address_3      : 7;
		uint32_t                : 24
	};
	//uint32_t data_buffer
	struct {
		uint32_t data_buffer    : 8;
		uint32_t                : 24
	};
	//uint32_t peripheral_address_mask_0;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_0 : 7;
		uint32_t                : 24
	};
	//uint32_t peripheral_address_mask_1;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_1 : 7;
		uint32_t                : 24
	};
	//uint32_t peripheral_address_mask_2;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_2 : 7;
		uint32_t                : 24
	};
	//uint32_t peripheral_address_mask_3;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_3 : 7;
		uint32_t                : 24
	};

} platform_i2c_registers_t;


/**
 * Platform-specific, per-I2C-instance data.
 */
typedef struct {

	// The clock that controls the relevant I2C.
	platform_branch_clock_t *clock;

} platform_i2c_t;

#endif
