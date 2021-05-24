/*
 * This file is part of libgreat
 *
 * Generic I2C driver header.
 */


#include <toolchain.h>
#include <drivers/platform_i2c.h>
#include <drivers/memory/ringbuffer.h>


#ifndef __LIBGREAT_I2C_H__
#define __LIBGREAT_I2C_H__

/**
 * I2C generic status codes; platform interrupt routines need to translate platform
 * specific status/state codes to libgreat generic status/state codes
 * (e.g. for interrupt handling).  These are based off the LPC43xx.
 */
typedef enum i2c_stat_code {
        //Controller Transmitter Mode
        START_TRANS,
        REPEAT_START_TRANS,
        SLA_W_TRANS_ACK,
        SLA_W_TRANS_NACK,
        CTRL_DAT_TRANS_ACK,
        CTRL_DAT_TRANS_NACK,
        //Controller Transmitter/Receiver Mode
        ARB_LOST,
        //Controller Receiver Mode
        SLA_R_TRANS_ACK,
        SLA_R_TRANS_NACK,
        CTRL_DAT_RECV_ACK,
        CTRL_DAT_RECV_NACK,
        //Peripheral Receiver Mode
        SLA_W_RECV_ACK,
        ARB_LOST_SLA_W_RECV_ACK,
        GC_RECV_ACK,
        ARB_LOST_GC_RECV_ACK,
        PERIP_DAT_RECV_ACK,
        PERIP_DAT_RECV_NACK,
        GC_DAT_RECV_ACK,
        GC_DAT_RECV_NACK,
        PERIP_STOP_REPEAT_START,
        //Peripheral Transmitter Mode
        SLA_R_RECV_ACK,
        ARB_LOST_SLA_R_RECV_ACK,
        PERIP_DAT_TRANS_ACK,
        PERIP_DAT_TRANS_NACK,
        PERIP_LAST_DAT_ACK,
        //Miscellaneious
        NO_RELEVANT_STATE_INFO,
        BUS_ERROR,
        UNKNOWN_STAT_CODE
} i2c_stat_code_t;

/**
 * Object representing a I2C device.
 */
typedef struct i2c {

	/**
	 * User configuration fields.
	 * These should be set before a call to uart_init.
	 */
	i2c_number_t              number;

	// The size of the buffer to be allocated for buffered reads/writes.
	// If this is set to 0, only synchronous reads and writes are supported.
	size_t buffer_size;

	/**
	 * i2c timeout value for waiting for ACK/NACK
	 * currently this value is platform dependent (i.e. this isn't an actual "time" measurement)
	 * (current lpc43xx code increments a counter in a while loop until it reaches this number)
	 * TODO firm-up the definition of this
	 */
	uint32_t timeout;

	/**
	 * last status code for i2c
	 * To keep things moving, the i2c interrupt handler sets SIC which returns the status code to NO_RELEVANT_STATE_INFO
	 * no platform level code will set this back to NO_RELEVANT_STATE_INFO
	 * This can be used so "application" level code can see the last handled status.
	 * If handling of the status is complete, suggest using NO_RELEVANT_STATE_INFO to clear the status
	 */
	volatile i2c_stat_code_t status;

	
	// PERIPHERAL ADDRESSES
	// Currently peripheral addresses must be specifed as part of initialization.
	// While this isn't flexible, it currently seems reasonable.
	//
	// number of peripheral addresses
	size_t num_perip_address;
	// array of peripheral addresses
	uint8_t *perip_address;
	// default byte to tx if no data in tx_buffer when read received
	// TODO make a perip struct that contains address/general call/and default data.
	uint8_t perip_default_tx_data;
	/**
	 * Private fields -- for platform driver use only. :)
	 */

	// I2C registers.
	platform_i2c_registers_t *reg;

	// Platform-specific data.
	platform_i2c_t platform_data;

	// Pointer to a ringbuffer used for asynchronous reads and writes.
	// TODO may need to setup separate buffers for the controller and each peripheral
	// TODO may need variable to store the current target address for the controller
	//		(e.g. in case of arbitration lost and/or repeated start)
	ringbuffer_t rx_buffer;
	ringbuffer_t tx_buffer;

	// For Controller reads, need to know how many more bytes to read
	volatile size_t rx_len;

} i2c_t;


/**
 * I2C implementation functions.
 */


/**
 * Sets up a platform I2C for use.
 *
 * @param i2c A I2C structure with configuration fields pre-populated. See above.
 */
int i2c_initialize(i2c_t *i2c, uint16_t duty_cycle);


/**
 * Perform a I2C transmit, but block until the transmission is accepted.
 */
void i2c_transmit_synchronous(i2c_t *i2c, uint8_t byte);

/**
 * retrieve the I2C status
 */
i2c_stat_code_t i2c_get_status(i2c_t *i2c);

int i2c_controller_write(i2c_t *i2c, uint8_t address, size_t data_len, uint8_t *data);
int i2c_start(i2c_t *i2c);
int i2c_controller_read(i2c_t *i2c, uint8_t address, size_t data_len, uint8_t *data);
int i2c_set_scl_low_duty_cycle(i2c_t *i2c, uint16_t duty_cycle);
int i2c_set_scl_high_duty_cycle(i2c_t *i2c, uint16_t duty_cycle);
int i2c_stop(i2c_t *i2c);
int i2c_start(i2c_t *i2c);
int i2c_tx_byte(i2c_t *i2c, uint8_t byte);
uint8_t i2c_rx_byte(i2c_t *i2c, bool ack);

#endif
