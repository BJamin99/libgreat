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
	 * Private fields -- for driver use only. :)
	 */

	// I2C registers.
	platform_i2c_registers_t *reg;

	// Platform-specific data.
	platform_i2c_t platform_data;

	// Pointer to a ringbuffer used for asynchronous reads and writes.
	// May be null if only synchronous reads and writes are supported.
	ringbuffer_t rx_buffer;
	ringbuffer_t tx_buffer;

} i2c_t;

/**
 * I2C implementation functions.
 */


/**
 * Sets up a platform I2C for use.
 *
 * @param i2c A I2C structure with configuration fields pre-populated. See above.
 */
int i2c_init(i2c_t *i2c);


/**
 * Platform-specific functions.
 */

/**
 * Performs platform-specific initialization for the given I2C.
 */
int platform_i2c_init(i2c_t *i2c);

/**
 * @return the frequency of the clock driving this I2C, in Hz
 */
uint32_t platform_i2c_get_parent_clock_frequency(i2c_t *i2c);


/**
 * Performs platform-specific initialization for the system's I2C interrupt.
 */
int platform_i2c_set_up_interrupt(i2c_t *i2c);


/**
 * Perform a I2C transmit, but block until the transmission is accepted.
 */
void i2c_transmit_synchronous(i2c_t *i2c, uint8_t byte);



/**
 * Reads all available data from the asynchronous receive buffer --
 * essentially any data received since the last call to a read function.
 *
 * @param buffer The buffer to read into.
 * @param count The maximum amount of data to read. Often, this is the size of
 *              your buffer.
 * @return The total number of bytes read.
 */
size_t i2c_read(i2c_t *i2c, void *buffer, size_t count);

#endif
