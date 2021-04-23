/*
 * This file is part of libgreat
 *
 * LPC43xx-specific I2C code
 */

#include <debug.h>

#include <drivers/scu.h>
#include <drivers/platform_i2c.h>
#include <drivers/arm_vectors.h>
#include <drivers/platform_clock.h>

//
// Stores a reference to each active I2C object.
// Used to grab the relevant I2C object for its interrupt.
//
i2c_t *active_i2c_objects[NUM_I2CS];


typedef struct {

	// Identifies the I2C's location in the SCU.
	uint8_t group;
	uint8_t pin;

	// The SCU function number that switches a given pin to I2C mode.
	uint8_t function;

} i2c_pin_t;

typedef struct {

	// Transmit / receive.
	i2c_pin_t scl;
	i2c_pin_t sda;

} i2c_pins_t;


/**
 * Mapping that provides the location of the default pins for each I2C.
 * Note I2C0 has its own pins and are not multiplexed GPIO
 */
i2c_pins_t i2c_pins[] = {
	NULL,
	{
		.scl = { .group = 2, .pin = 4, .function = 1 },
		.sda = { .group = 2, .pin = 3, .function = 1 }
	},
};


// Imports from the local I2C driver. These aren't part of the public API,
// so they're not defined in platform_i2c.h.
void i2c_interrupt(i2c_t *i2c);


/**
 * @return the register bank for the given I2C
 */
static platform_i2c_registers_t *get_i2c_registers(i2c_number_t i2c_number)
{
	switch(i2c_number) {
		case 0: return (platform_i2c_registers_t *)0x400A1000;
		case 1: return (platform_i2c_registers_t *)0x400E0000;
	}

	pr_error("platform_i2c: tried to set up a non-existant I2C %d!\n", i2c_number);
	return 0;
}



static platform_branch_clock_t *get_clock_for_i2c(i2c_number_t i2c_number)
{
	platform_clock_control_register_block_t *ccu = get_platform_clock_control_registers();

	switch(i2c_number)  {
		case 0: return &ccu->apb1->i2c0;
		case 1: return &ccu->apb3->i2c1;
	}

	pr_error("platform_i2c: cannot find a clock for I2C %d!\n", i2c_number);
	return NULL;
}


/**
 * Performs platform-specific initialization for the given I2C
 */
int platform_i2c_init(i2c_t *i2c)
{
	i2c_pins_t pins = i2c_pins[i2c->number];

	// Fetch the registers for the relevant I2C
	i2c->reg = get_i2c_registers(i2c->number);
	if (!i2c->reg) {
		return EINVAL;
	}

	// Figure out the clock that drives the relevant I2C
	i2c->platform_data.clock = get_clock_for_i2c(i2c->number);
	if (!i2c->platform_data.clock) {
		return EINVAL;
	}

	// ... and ensure that it's on.
	platform_enable_branch_clock(i2c->platform_data.clock, false);

	// Connect our I2C pins to the I2C hardware (only needed for I2C1)
	if (pins) {
		platform_scu_configure_pin_i2c(pins.scl.group, pins.scl.pin, pins.scl.function);
		platform_scu_configure_pin_i2c(pins.sda.group, pins.sda.pin, pins.sda.function);
	}

	return 0;
}


/**
 * Low-level I2C interrupt handlers.
 * These grab the active I2C object for the relevant interrupt, and call the main ISR.
 */

static void platform_i2c0_interrupt(void)
{
	i2c_interrupt(active_i2c_objects[0]);
}

static void platform_i2c1_interrupt(void)
{
	i2c_interrupt(active_i2c_objects[1]);
}



/**
 * Performs platform-specific initialization for the system's I2C interrupt.
 */
int platform_i2c_set_up_interrupt(i2c_t *i2c)
{
	uint32_t irq_number;

	// Look-up table of per-I2C interrupt handlers.
	const vector_table_entry_t irq_handlers[] = {
		platform_i2c0_interrupt, platform_i2c1_interrupt
	};

	const uint32_t irq_numbers[] = {
		I2C0_IRQ, I2C1_IRQ
	};

	// Store the current I2C object, so we can find it during our interrupt handler.
	active_i2c_objects[i2c->number] = i2c;

	// Enable the relevant interrupt in the NVIC.
	irq_number = irq_numbers[i2c->number];
	vector_table.irqs[irq_number] = irq_handlers[i2c->number];
	platform_enable_interrupt(irq_number);

	return 0;
}

uint32_t platform_i2c_turn_on_ack(i2c_t *i2c) {
	// turn on ack
	return 0;
}

uint32_t platform_i2c_turn_off_ack(i2c_t *i2c) {
	//turn off ack
	return 0;
}

uint32_t platform_i2c_turn_on_interrupt(i2c_t *i2c) {
	//turn on interrupt
	return 0;
}

uint32_t platform_i2c_turn_off_interrupt(i2c_t *i2c) {
	//turn off interrupt
	return 0;
}

uint32_t platform_i2c_stop_controller(i2c_t *i2c) {
	//send stop
	return 0;
}

// issues a start condition to enter controller mode.
// if restart is true and the interface is already in controller mode,
// issues a stop and then a start.  Otherwise a repeated start is issued
uint32_t platform_i2c_start_controller(i2c_t *i2c, bool restart) {
	//start/repeated start
	return 0;
}

uint32_t platform_i2c_enable(i2c_t *i2c) {
	//enable interface
	return 0;
}

uint32_t platform_i2c_disable(i2c_t *i2c) {
	//disable interface
	return 0;
}

uint32_t platform_i2c_write_byte(i2c_t *i2c, uint32_t byte) {
    return 0;
}

uint32_t platform_i2c_read_byte(i2c_t *i2c, uint32_t *byte) {
	return 0;
}

uint32_t platform_i2c_set_7bit_address(i2c_t *i2c, uint32_t perip_num, uint8_t address) {
    return 0;
}

uint32_t platform_i2c_set_7bit_addres_mask(i2c_t *i2c, uint32_t perip_num, uint8_t mask) {
	return 0;
}

uint32_t platform_i2c_set_scl_high_duty_cycle(i2c_t *i2c, uint32_t duty) {
	return 0;
}

uint32_t platform_i2c_set_scl_low_duty_cycle(i2c_t *i2c, uint32_t duty) {
	return 0;
}

uint32_t platform_i2c_monitor_mode_enable(i2c_t *i2c) {
	return 0;
}

uint32_t platform_i2c_monitor_mode_disable(i2c_t *i2c) {
	return 0;
}

/**
 * @return the frequency of the clock driving this I2C, in Hz
 */
uint32_t platform_i2c_get_parent_clock_frequency(i2c_t *i2c)
{
	return platform_get_branch_clock_frequency(i2c->platform_data.clock);
}
