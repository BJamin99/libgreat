/*
 * This file is part of libgreat
 *
 * LPC43xx-specific I2C drivers
 */

#ifndef __LIBGREAT_PLATFORM_I2C_H__
#define __LIBGREAT_PLATFORM_I2C_H__

#include <toolchain.h>
#include <drivers/platform_clock.h>

//Platform Capabilities
#define PLAT_CONTROLLER true
#define PLAT_PERIPHERAL true
#define PLAT_PERIP_MASK true

//Controller Transmitter Mode
#define STAT_CODE_START_TRANS 0x08 >> 3
#define STAT_CODE_REPEAT_START_TRANS 0x10 >> 3
#define STAT_CODE_SLA_W_TRANS_ACK 0x18 >> 3
#define STAT_CODE_SLA_W_TRANS_NACK 0x20 >> 3
#define STAT_CODE_CTRL_DAT_TRANS_ACK 0x28 >> 3
#define STAT_CODE_CTRL_DAT_TRANS_NACK 0x30 >> 3
//Controller Transmitter/Receiver Mode
#define STAT_CODE_ARB_LOST 0x38 >> 3
//Controller Receiver Mode
#define STAT_CODE_SLA_R_TRANS_ACK 0x40 >> 3
#define STAT_CODE_SLA_R_TRANS_NACK 0x48 >> 3
#define STAT_CODE_CTRL_DAT_RECV_ACK 0x50 >> 3
#define STAT_CODE_CTRL_DAT_RECV_NACK 0x58 >> 3
//Peripheral Receiver Mode
#define STAT_CODE_SLA_W_RECV_ACK 0x60 >> 3
#define STAT_CODE_ARB_LOST_SLA_W_RECV_ACK 0x68 >> 3
#define STAT_CODE_GC_RECV_ACK 0x70 >> 3
#define STAT_CODE_ARB_LOST_GC_RECV_ACK 0x78 >> 3
#define STAT_CODE_PERIP_DAT_RECV_ACK 0x80 >> 3
#define STAT_CODE_PERIP_DAT_RECV_NACK 0x88 >> 3
#define STAT_CODE_GC_DAT_RECV_ACK 0x90 >> 3
#define STAT_CODE_GC_DAT_RECV_NACK 0x98 >> 3
#define STAT_CODE_PERIP_STOP_REPEAT_START 0xA0 >> 3
//Peripheral Transmitter Mode
#define STAT_CODE_SLA_R_RECV_ACK 0xA8 >> 3
#define STAT_CODE_ARB_LOST_SLA_R_RECV_ACK 0xB0 >> 3
#define STAT_CODE_PERIP_DAT_TRANS_ACK 0xB8 >> 3
#define STAT_CODE_PERIP_DAT_TRANS_NACK 0xC0 >> 3
#define STAT_CODE_PERIP_LAST_DAT_ACK 0xC8 >> 3
//Miscellaneious
#define STAT_CODE_NO_RELEVANT_STATE_INFO 0xF8 >> 3
#define STAT_CODE_BUS_ERROR 0x00

// I2C generic status codes; platform interrupt routines need to translate platform
// specific status/state codes to libgreat generic status/state codes
// (e.g. for interrupt handling).  This is defined in i2c.h
typedef enum i2c_stat_code i2c_stat_code_t;

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
	NUM_I2CS = 2,
	// Number of peripherals per I2C
	NUM_PERIPHS = 4
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
        uint32_t                : 25;
	};
	//uint32_t status;
	struct {
		uint32_t                : 3;
		uint32_t status         : 5;
		uint32_t                : 24;
	};
	//uint32_t data;
	struct {
		uint32_t data           : 8;
		uint32_t                : 24;
	};
	//uint32_t peripheral_address_0;
	struct {
		uint32_t general_call_0 : 1;
		uint32_t address_0      : 7;
		uint32_t                : 24;
	};
	//uint32_t duty_cycle_high_half_word;
   	struct {
   		uint32_t duty_cycle_high_half_word : 16;
   		uint32_t                           : 16;
   	};
	//uint32_t duty_cycle_low_half_word;
   	struct {
   		uint32_t duty_cycle_low_half_word  : 16;
   		uint32_t                           : 16;
   	};
	//uint32_t control_clear;
	struct {
		uint32_t                : 2;
		uint32_t assert_ack_clr : 1;
		uint32_t interrupt_clr  : 1;
		uint32_t                : 1;
		uint32_t start_clr      : 1;
        uint32_t i2c_disable    : 1;
        uint32_t                : 25;
	};
	//uint32_t monitor_mode_control;
	struct {
		uint32_t monitor_mode_enable : 1;
		uint32_t scl_output_enable   : 1;
		uint32_t interrupt_match     : 1;
		uint32_t                     : 29;
	};
	//uint32_t peripheral_address_1;
	struct {
		uint32_t general_call_1 : 1;
		uint32_t address_1      : 7;
		uint32_t                : 24;
	};
	//uint32_t peripheral_address_2;
	struct {
		uint32_t general_call_2 : 1;
		uint32_t address_2      : 7;
		uint32_t                : 24;
	};
	//uint32_t peripheral_address_3;
	struct {
		uint32_t general_call_3 : 1;
		uint32_t address_3      : 7;
		uint32_t                : 24;
	};
	//uint32_t data_buffer
	struct {
		uint32_t data_buffer    : 8;
		uint32_t                : 24;
	};
	//uint32_t peripheral_address_mask_0;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_0 : 7;
		uint32_t                : 24;
	};
	//uint32_t peripheral_address_mask_1;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_1 : 7;
		uint32_t                : 24;
	};
	//uint32_t peripheral_address_mask_2;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_2 : 7;
		uint32_t                : 24;
	};
	//uint32_t peripheral_address_mask_3;
	struct {
		uint32_t                : 1;
		uint32_t address_mask_3 : 7;
		uint32_t                : 24;
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

//static platform_i2c_registers_t *get_i2c_registers(i2c_number_t i2c_number);
//int platform_i2c_init(i2c_t *i2c);
i2c_stat_code_t platform_i2c_get_stat(i2c_t *i2c);
//int platform_i2c_set_up_interrupt(i2c_t *i2c);
uint32_t platform_i2c_turn_on_ack(i2c_t *i2c);
uint32_t platform_i2c_turn_off_ack(i2c_t *i2c);
uint32_t platform_i2c_turn_on_interrupt(i2c_t *i2c);
uint32_t platform_i2c_turn_off_interrupt(i2c_t *i2c);
uint32_t platform_i2c_stop_controller(i2c_t *i2c);
uint32_t platform_i2c_start_controller(i2c_t *i2c, bool restart);
uint32_t platform_i2c_enable(i2c_t *i2c);
uint32_t platform_i2c_disable(i2c_t *i2c);
uint32_t platform_i2c_write_byte(i2c_t *i2c, uint8_t byte);
uint32_t platform_i2c_read_byte(i2c_t *i2c, uint8_t *byte);
uint32_t platform_i2c_set_7bit_address(i2c_t *i2c, uint32_t perip_num, uint8_t address, bool gc);
uint32_t platform_i2c_set_7bit_addres_mask(i2c_t *i2c, uint32_t perip_num, uint8_t mask);
uint32_t platform_i2c_set_scl_high_duty_cycle(i2c_t *i2c, uint16_t duty);
uint32_t platform_i2c_set_scl_low_duty_cycle(i2c_t *i2c, uint16_t duty);
uint32_t platform_i2c_monitor_mode_enable(i2c_t *i2c, bool scl_enable, bool match);
uint32_t platform_i2c_monitor_mode_disable(i2c_t *i2c);
uint32_t platform_i2c_get_parent_clock_frequency(i2c_t *i2c);

