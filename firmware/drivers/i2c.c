/*
 * This file is part of libgreat
 *
 * Generic I2C drivers.
 */


#include <math.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <toolchain.h>

#include <drivers/i2c.h>
#include <drivers/arm_system_control.h>

typedef enum {
	IRQ_RECEIVE_DATA_AVAILABLE = 0x2,
} pending_interrupt_t;


// FIXME HACK: statically allocate I2C buffers
static uint8_t i2c_rx_buffer[256];
static uint8_t i2c_tx_buffer[256];

/**
 * Sets up a platform I2C for use.
 *
 * @param i2c A I2C structure with configuration fields pre-populated. See i2c.h.
 */
int i2c_init(i2c_t *i2c)
{
	// Perform the platform-specific initialization for the given I2C.
	int rc = platform_i2c_init(i2c);
	if (rc) {
		return rc;
	}

	// If we're going to allocate a buffer, we can perform asynchronous reads and writes.
	// Accordingly, we'll set up interrupts to fill / drain the rx and tx buffers.
	if (i2c->buffer_size) {

		// Allocate memory for our Rx buffer..
		// FIXME: malloc this!
		ringbuffer_init(&i2c->rx_buffer, i2c_rx_buffer, i2c->buffer_size);

		// ... and for our tx buffer.
		// FIXME: malloc this!
		ringbuffer_init(&i2c->tx_buffer, i2c_tx_buffer, i2c->buffer_size);

		// If we couldn't allocate either buffer, disable synchronous operations and return a warning code.
		if (!i2c->rx_buffer.buffer || !i2c->tx_buffer.buffer) {
			pr_warning("i2c: warning: could not allocate memory for our async operations buffer!\n");
			pr_warning("i2c: asynchronous operations disabled -- all reads/writes will be synchronous!");
			i2c->buffer_size = 0;

			free(i2c->rx_buffer.buffer);
			free(i2c->tx_buffer.buffer);

			return ENOMEM;
		}

		// Set up an interrupt to handle I2C interrupt events.
		rc = platform_i2c_set_up_interrupt(i2c);
		if (rc) {
			return rc;
		}
	}
	
	return 0;
}

int i2c_set_perip_addr(i2c_t *i2c, uint8_t 

void i2c_data_ready_interrupt(i2c_t *i2c)
{
	// FIXME reg is for platform code use only.  This should call a platform function.
	uint8_t rx_data = i2c->reg->data_buffer;
	ringbuffer_enqueue_overwrite(&i2c->rx_buffer, rx_data);
}



/**
 * Function called as the main handler for a I2C interrupt.
 */
void i2c_interrupt(i2c_t *i2c)
{
	// If there are no I2C interrupts pending, there's nothing to do.
	// Return early.
	// FIXME reg is for platform code use only.  This should call a platform function.
	if (i2c->reg->status = 0xf8) {
		return;
	}

	// If this is a "new data received" event, handle it.
	// FIXME reg is for platform code use only.  This should call a platform function.
	if(i2c->reg->status == IRQ_RECEIVE_DATA_AVAILABLE) {
		i2c_data_ready_interrupt(i2c);
	}
}



/**
 * Reads all available data from the asynchronous receive buffer --
 * essentially any data received since the last call to a read function.
 *
 * @param buffer The buffer to read into.
 * @param count The maximum amount of data to read. Often, this is the size of
 *              your buffer.
 * @return The total number of bytes read.
 */
size_t i2c_read(i2c_t *i2c, void *buffer, size_t count)
{
	uint8_t *byte_buffer = buffer;
	size_t data_read = 0;

	// Special case: if we're in synchronous mode, read and return
	// a single byte -- this handles the case where this is called after
	// an allocation fail-out.
	if (i2c->buffer_size == 0) {
		// TODO: implement
		return 0;
	}

	// Read count bytes from the buffer, or as much as is available.
	for (unsigned i = 0; i < count; ++i) {
		if (ringbuffer_empty(&i2c->rx_buffer)) {
			break;
		}

		byte_buffer[i] = ringbuffer_dequeue(&i2c->rx_buffer);
		data_read++;
	}

	// Return the actual amount read.
	return data_read;
}



/**
 * Perform a I2C controller transmit.
 * 
 * i2c is the i2c object to use
 * address is the 7-bit peripheral address (<=127)
 *
 */
int i2c_controller_transmit(i2c_t *i2c, uint8_t address, )
{
	if (!i2c) {
		return ENODEV;
	}

	if (address > 127) {
		return ENXIO;
	}
    
    // Shift address leaving LSB W/R bit unset to signal a write
    address = address << 1

	// Enter Controller Transmitter mode; the control register I2EN bit needs to be set
	// FIXME reg is for platform code use only.  This should call a platform function.
	&i2c->reg->i2c_enable = true;


}
