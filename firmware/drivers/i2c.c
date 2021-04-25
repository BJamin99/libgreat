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

		// Check for peripheral addresses.  If there is a peripheral address, assert ack
		if (&i2c->num_perip_address > 0) {
			for(int i=0; i<NUM_PERIPS; i++) {
				if (i >= &i2c->num_perip_address) {
					rc = platform_i2c_set_7bit_address(i2c, i, &i2c->perip_address[&i2c->num_perip_address-1], false);
					if(rc) {
						return rc;
					}
				}

				rc = platform_i2c_set_7bit_address(i2c, i, &i2c->perip_address[i], true);
				if(rc) {
					return rc;
				}
			}
			rc = platform_i2c_turn_on_ack(i2c);
			if(rc) {
				return rc;
			}
		}
	}

	rc = platform_i2c_enable(i2c);
	if(rc) {
		return rc;
	}
	
	return 0;
}


/**
 * Function called as the main handler for a I2C interrupt.
 */
void i2c_interrupt(i2c_t *i2c)
{
	status = platform_i2c_get_stat(i2c);
	switch(&i2c->reg->status) {
		//Controller States
		case i2c_stat_code.CTRL_DAT_TRANS_ACK: 
			if(ringbuffer_empty(&i2c->tx_buffer)){
				platform_i2c_turn_on_ack(i2c);
				platform_i2c_stop_controller(i2c);				
				break;
			}
		case i2c_stat_code.START_TRANS: 
		case i2c_stat_code.REPEAT_START_TRANS: 
		//Controller Transmitter State
		case i2c_stat_code.SLA_W_TRANS_ACK: 
			if(ringbuffer_data_available(&i2c->tx_buffer)){
				platform_i2c_write_byte(ringbuffer_dequeue(&i2c->txbuffer));
				platform_i2c_turn_on_ack(i2c);
			}
			break;
		case i2c_stat_code.CTRL_DAT_TRANS_NACK:
		case i2c_stat_code.SLA_W_TRANS_NACK:
		case i2c_stat_code.BUS_ERROR:
		case i2c_stat_code.SLA_R_TRANS_NACK:
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_stop_controller(i2c);
			break;
		//Controller Transmitter/Receiver Mode
		case i2c_stat_code.ARB_LOST:
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_start_controller(i2c);
			break;
		//Controller Receiver Mode
		case i2c_stat_code.SLA_R_TRANS_ACK:
			platform_i2c_turn_on_ack(i2c);
			break;
		case i2c_stat_code.CTRL_DAT_RECV_ACK:
			uint8_t rx_byte;
			platform_i2c_read_byte(i2c, &rx_byte);
			ringbuffer_enqueue(&i2c->rx_buffer, rx_byte);
			if(--i2c->rx_len == 0) {
				platform_i2c_turn_off_ack(i2c);
			}
			else {
				platform_i2c_turn_on_ack(i2c);
			}
			break;
		case i2c_stat_code.CTRL_DAT_RECV_NACK: 
			uint8_t rx_byte;
			platform_i2c_read_byte(i2c, &rx_byte);
			ringbuffer_enqueue(&i2c->rx_buffer, rx_byte);
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_stop_controller(i2c);
			// TODO do we need to signal data receive complete to trigger transfer to comms?
			break;
		//Peripheral Receiver Mode
		case i2c_stat_code.SLA_W_RECV_ACK:
		case i2c_stat_code.GC_RECV_ACK:
			platform_i2c_turn_on_ack(i2c);
			break;
		case i2c_stat_code.ARB_LOST_SLA_W_RECV_ACK:
		case i2c_stat_code.ARB_LOST_GC_RECV_ACK:
			platform_i2c_start_controller(i2c);
			platform_i2c_turn_on_ack(i2c);
			break;
		case i2c_stat_code.PERIP_DAT_RECV_ACK:
			uint8_t rx_byte;
			platform_i2c_read_byte(i2c, &rx_byte);
			ringbuffer_enqueue(&i2c->rx_buffer, rx_byte);
			if(ringbuffer_data_available(&i2c->rx_buffer) == &i2c->rx_len) {
				platform_i2c_turn_off_ack(i2c);
			}
			else {
				platform_i2c_turn_on_ack(i2c);
			}
			break;
		case i2c_stat_code.PERIP_DAT_RECV_NACK:
		case i2c_stat_code.GC_DAT_RECV_NACK:
		case i2c_stat_code.PERIP_STOP_REPEAT_START:
		case i2c_stat_code.PERIP_DAT_TRANS_NACK:
		case i2c_stat_code.PERIP_LAST_DAT_ACK:
			platform_i2c_turn_on_ack(i2c);
			break;
		case i2c_stat_code.GC_DAT_RECV_ACK:
			uint8_t rx_byte;
			platform_i2c_read_byte(i2c, &rx_byte);
			ringbuffer_enqueue(&i2c->rx_buffer, rx_byte);
			platform_i2c_turn_off_ack(i2c);
			break;
		//Peripheral Transmitter Mode
		case i2c_stat_code.SLA_R_RECV_ACK:
		case i2c_stat_code.PERIP_DAT_TRANS_ACK:
			//We rely on host having filled peripheral buffer
			//TODO the model here can either be rely on the peripheral buffer to be filled;
			//     have a default "read" buffer for a perpiheral;
			//	   or require back and forth with the host
			uint8_t tx_byte;
			if(ringbuffer_empty(&i2c->tx_buffer)) {
				tx_byte = i2c->perip_default_tx_data;
			}
			else {
				ringbuffer_dequeue(&i2c->tx_buffer, &tx_byte);
			}
			platform_i2c_write_byte(i2c, tx_byte);
			platform_i2c_turn_on_ack(i2c);
			break;
		case i2c_stat_code.ARB_LOST_SLA_R_RECV_ACK: 
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_start_controller(i2c);
			break;
		//Miscellaneious
		case i2c_stat_code.NO_RELEVANT_STATE_INFO:
		case i2c_stat_code.UNKNOWN_STAT_CODE:
		default:
	}
	platform_i2c_turn_off_interrupt(i2c);
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
 * Perform an I2C controller write.
 * 
 * i2c is the i2c object to use
 * address is the 7-bit peripheral address (<=127)
 *
 */
int i2c_controller_write(i2c_t *i2c, uint8_t address, size_t data_len, uint8_t *data)
{
	if (!i2c) {
		return ENODEV;
	}

	if (address > 127) {
		return ENXIO;
	}

	if (data_len > (&i2c->tx_buffer->size - ringbuffer_data_available(&i2c->tx_buffer))) {
		return ENOMEM;
	}
    
    // Shift address leaving LSB W/R bit unset to signal a write
    address = address << 1

	// Load address plus data into tx ring buffer
	ringbuffer_enqueue(&i2c->tx_buffer, address);
	for(int i=0; i<data_len; i++) {
		ringbuffer_enqueue(&i2c->tx_buffer, data[i]);
	}

	// Trigger a start condition to initiate master controller mode
	// data transmission done through interrupt handler
	platform_i2c_start_controller(i2c);
	while(!ringbuffer_data_available(&i2c->tx_buffer));
	return 0;
}

/**
 * Perform an I2C controller read.
 * 
 * i2c is the i2c object to use
 * address is the 7-bit peripheral address (<=127)
 *
 */
int i2c_controller_read(i2c_t *i2c, uint8_t address, size_t data_len, uint8_t *data)
{
	if (!i2c) {
		return ENODEV;
	}

	if (address > 127) {
		return ENXIO;
	}

	if (data_len > (&i2c->rx_buffer->size - ringbuffer_data_available(&i2c->rx_buffer))) {
		return ENOMEM;
	}
    
    // Shift address leaving LSB W/R bit set to signal a read
    address = address << 1 || 0x01

    // Load address into the tx_buffer (this is where the interrupt will grab it).
    ringbuffer_enqueue(&i2c->tx_buffer, address);

	// Trigger a start condition to initiate master controller mode
	// data transmission/reception done through interrupt handler
	platform_i2c_start_controller(i2c);
	while(!ringbuffer_data_available(&i2c->tx_buffer));

	// Remove data from rx ring buffer
	for(int i=0; i<data_len; i++) {
		ringbuffer_dequeue(&i2c->rx_buffer, &data[i]);
	}

	return 0;
}


 //TODO i2c_peripheral_load_tx_buffer
 //TODO i2c_peripheral_read_rx_buffer
 //TODO i2c_monitor_mode_start
 //TODO i2c_monitor_mode_read_rx_buffer
 //TODO i2c_monitor_mode_stop
