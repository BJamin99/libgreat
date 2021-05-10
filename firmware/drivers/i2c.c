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
int i2c_initialize(i2c_t *i2c)
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
		if (i2c->num_perip_address > 0) {
			for(uint32_t i=0; i<NUM_PERIPHS; i++) {
				if (i >= i2c->num_perip_address) {
					rc = platform_i2c_set_7bit_address(i2c, i, i2c->perip_address[i2c->num_perip_address-1], false);
					if(rc) {
						return rc;
					}
				}

				rc = platform_i2c_set_7bit_address(i2c, i, i2c->perip_address[i], true);
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
	
	return platform_i2c_enable(i2c);
}

/**
 * Function called to disable i2c
 */
int i2c_disable(i2c_t *i2c)
{
	return platform_i2c_disable(i2c);
}

/**
 * Function called to send i2c stop
 */
int i2c_stop(i2c_t *i2c)
{
	return platform_i2c_stop_controller(i2c);
}

/**
 * Function called to send i2c start
 */
int i2c_start(i2c_t *i2c)
{
	return platform_i2c_start_controller(i2c, true);
}

/**
 * Function called to send i2c byte
 */
int i2c_tx_byte(i2c_t *i2c, uint8_t byte)
{
	return platform_i2c_write_byte(i2c, byte);
}

/**
 * Function called to read i2c byte
 */
uint8_t i2c_rx_byte(i2c_t *i2c, bool ack)
{
	uint8_t byte;
	if(ack){
		platform_i2c_turn_on_ack(i2c);
	} else {
		platform_i2c_turn_off_ack(i2c);
	}
	platform_i2c_read_byte(i2c, &byte);
	return byte;
}

/**
 * Function called to set i2c SCL high duty cycle
 */
int i2c_set_scl_high_duty_cycle(i2c_t *i2c, uint16_t duty_cycle)
{
	return platform_i2c_set_scl_high_duty_cycle(i2c, duty_cycle);
}

/**
 * Function called to set i2c SCL low duty cycle
 */
int i2c_set_scl_low_duty_cycle(i2c_t *i2c, uint16_t duty_cycle)
{
        return platform_i2c_set_scl_low_duty_cycle(i2c, duty_cycle);
}
	

/**
 * Function called as the main handler for a I2C interrupt.
 */
void i2c_interrupt(i2c_t *i2c)
{
	uint8_t byte=0;
	i2c_stat_code_t status = platform_i2c_get_stat(i2c);
	switch(status) {
		//Controller States
		case CTRL_DAT_TRANS_ACK: 
			if(ringbuffer_empty(&i2c->tx_buffer)){
				platform_i2c_turn_on_ack(i2c);
				platform_i2c_stop_controller(i2c);				
				break;
			}
			__attribute__((fallthrough));
		case START_TRANS: 
		case REPEAT_START_TRANS: 
		//Controller Transmitter State
		case SLA_W_TRANS_ACK: 
			if(ringbuffer_data_available(&i2c->tx_buffer)){
				platform_i2c_write_byte(i2c, ringbuffer_dequeue(&i2c->tx_buffer));
				platform_i2c_turn_on_ack(i2c);
			}
			break;
		case CTRL_DAT_TRANS_NACK:
		case SLA_W_TRANS_NACK:
		case BUS_ERROR:
		case SLA_R_TRANS_NACK:
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_stop_controller(i2c);
			break;
		//Controller Transmitter/Receiver Mode
		case ARB_LOST:
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_start_controller(i2c, false);
			break;
		//Controller Receiver Mode
		case SLA_R_TRANS_ACK:
			platform_i2c_turn_on_ack(i2c);
			break;
		case CTRL_DAT_RECV_ACK:
			platform_i2c_read_byte(i2c, &byte);
			ringbuffer_enqueue(&i2c->rx_buffer, byte);
			if(--i2c->rx_len == 0) {
				platform_i2c_turn_off_ack(i2c);
			}
			else {
				platform_i2c_turn_on_ack(i2c);
			}
			break;
		case CTRL_DAT_RECV_NACK: 
			platform_i2c_read_byte(i2c, &byte);
			ringbuffer_enqueue(&i2c->rx_buffer, byte);
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_stop_controller(i2c);
			// TODO do we need to signal data receive complete to trigger transfer to comms?
			break;
		//Peripheral Receiver Mode
		case SLA_W_RECV_ACK:
		case GC_RECV_ACK:
			platform_i2c_turn_on_ack(i2c);
			break;
		case ARB_LOST_SLA_W_RECV_ACK:
		case ARB_LOST_GC_RECV_ACK:
			platform_i2c_start_controller(i2c, false);
			platform_i2c_turn_on_ack(i2c);
			break;
		case PERIP_DAT_RECV_ACK:
			platform_i2c_read_byte(i2c, &byte);
			ringbuffer_enqueue(&i2c->rx_buffer, byte);
			if(ringbuffer_data_available(&i2c->rx_buffer) == i2c->rx_len) {
				platform_i2c_turn_off_ack(i2c);
			}
			else {
				platform_i2c_turn_on_ack(i2c);
			}
			break;
		case PERIP_DAT_RECV_NACK:
		case GC_DAT_RECV_NACK:
		case PERIP_STOP_REPEAT_START:
		case PERIP_DAT_TRANS_NACK:
		case PERIP_LAST_DAT_ACK:
			platform_i2c_turn_on_ack(i2c);
			break;
		case GC_DAT_RECV_ACK:
			platform_i2c_read_byte(i2c, &byte);
			ringbuffer_enqueue(&i2c->rx_buffer, byte);
			platform_i2c_turn_off_ack(i2c);
			break;
		//Peripheral Transmitter Mode
		case SLA_R_RECV_ACK:
		case PERIP_DAT_TRANS_ACK:
			//We rely on host having filled peripheral buffer
			//TODO the model here can either be rely on the peripheral buffer to be filled;
			//     have a default "read" buffer for a perpiheral;
			//	   or require back and forth with the host
			if(ringbuffer_empty(&i2c->tx_buffer)) {
				byte = i2c->perip_default_tx_data;
			}
			else {
				byte = ringbuffer_dequeue(&i2c->tx_buffer);
			}
			platform_i2c_write_byte(i2c, byte);
			platform_i2c_turn_on_ack(i2c);
			break;
		case ARB_LOST_SLA_R_RECV_ACK: 
			platform_i2c_turn_on_ack(i2c);
			platform_i2c_start_controller(i2c, false);
			break;
		//Miscellaneious
		case NO_RELEVANT_STATE_INFO:
		case UNKNOWN_STAT_CODE:
		default:
			break;
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
// TODO this is a leftover from using UART as the base for I2C.  Shouldn't be needed.
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

	if (data_len > (i2c->tx_buffer.size - ringbuffer_data_available(&i2c->tx_buffer))) {
		return ENOMEM;
	}
    
    // Shift address leaving LSB W/R bit unset to signal a write
    address = address << 1;

	// Load address plus data into tx ring buffer
	ringbuffer_enqueue(&i2c->tx_buffer, address);
	for(uint32_t i=0; i<data_len; i++) {
		ringbuffer_enqueue(&i2c->tx_buffer, data[i]);
	}

	// Trigger a start condition to initiate master controller mode
	// data transmission done through interrupt handler
	platform_i2c_start_controller(i2c, false);
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

	if (data_len > (i2c->rx_buffer.size - ringbuffer_data_available(&i2c->rx_buffer))) {
		return ENOMEM;
	}
    
    // Shift address leaving LSB W/R bit set to signal a read
    address = (address << 1) + 1;

    // Load address into the tx_buffer (this is where the interrupt will grab it).
    ringbuffer_enqueue(&i2c->tx_buffer, address);

	// Trigger a start condition to initiate master controller mode
	// data transmission/reception done through interrupt handler
	// TODO remove potential for infinite loop (e.g. timeout?)
	// TODO perhaps need some flags/signals from interrupts?
	platform_i2c_start_controller(i2c, false);
	while(!ringbuffer_data_available(&i2c->rx_buffer));

	// Remove data from rx ring buffer
	for(uint32_t i=0; i<data_len; i++) {
		data[i] = ringbuffer_dequeue(&i2c->rx_buffer);
	}

	return 0;
}

 // i2c_monitor_mode_start
int i2c_monitor_mode_start(i2c_t *i2c, bool scl_enable, bool match_all) {
	return platform_i2c_monitor_mode_enable(i2c, scl_enable, match_all);
}

 // i2c_monitor_mode_read_rx_buffer
int i2c_monitor_mode_read_rx_buffer(i2c_t *i2c, size_t data_len, uint8_t *data, size_t *actual_len) {
	if (!i2c) {
		return ENODEV;
	}

	// Remove data from rx ring buffer
	uint32_t i;
	size_t avail_data = ringbuffer_data_available(&i2c->rx_buffer);
	for(i = 0; i<data_len && i<avail_data; i++) {
		data[i] = ringbuffer_dequeue(&i2c->rx_buffer);
	}
	*actual_len = i;

	return 0;
}

 // i2c_monitor_mode_stop
int i2c_monitor_mode_stop(i2c_t *i2c) {
	return platform_i2c_monitor_mode_disable(i2c);
}

 //TODO i2c_peripheral_load_tx_buffer
 //TODO i2c_peripheral_read_rx_buffer
