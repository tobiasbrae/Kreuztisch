/*
*	File: port_configuration.c
*	
*	implements functions to define an manipulate a Port-Pin	
*
*	Author: Tobias Braechter
*	Date: 16.11.2017
*
*
*/

#include <stdint.h>

#ifndef PORT_CONFIGURATION_H_
#define PORT_CONFIGURATION_H_

	#define DDR 0
	#define PORT 1
	#define PIN 2

	typedef struct  
	{
		volatile uint8_t *ddr;
		volatile uint8_t *port;
		volatile uint8_t *pin;
		uint8_t bit;
	}portpin;
	
	/*
	*	sets the parameters of the given port pin
	*	arg0: the portpin to be configured
	*	arg1: the DDR-register to be set
	*	arg2: the PORT-register to be set
	*	arg3: the PIN-register to be set
	*	arg4: the bit to be set
	*/
	void setPortPin(portpin target, uint8_t *ddr, uint8_t *port, uint8_t *pin, uint8_t bit);

	/*
	*	sets the given port pin to the given value
	*	arg0: the port pin to write to
	*	arg1: the value to be written
	*/
	void setPin(portpin target, uint8_t reg, uint8_t value);
	
	/*
	*	toggles the given port pin
	*	arg0: the port pin to be toggled
	*/
	void togglePin(portpin target, uint8_t reg);
	
	/*
	*	reads the given port pin input
	*	arg0: the port pin to be read from
	*	ret: returns the value of the given port-pin
	*/
	uint8_t readPin(portpin target, uint8_t reg);
	
	/*
	*	copies the value of the given source port pin to the given target port pin
	*	arg0: the source port pin
	*	arg1: the target port pin
	*/
	void copyPin(portpin source, uint8_t regSource, portpin target, uint8_t regTarget);
	

	/*
	*	sets the given bit from the given register to the given value
	*	arg0: the target register
	*	arg1: the target bit
	*	arg2: the value to be written
	*/
	void setBit(volatile uint8_t* reg, uint8_t bit, uint8_t value);

	/*
	*	copies the value of the given source register-pin to the given target
	*	arg0: the source register
	*	arg1: the source bit
	*	arg2: the target register
	*	arg3: the target bit
	*/
	void copyBit(volatile uint8_t* regFrom, uint8_t bitFrom, volatile uint8_t* regTo, uint8_t bitTo);

	/*
	*	reads the value of the given bit in the given register
	*	arg0: the register to read
	*	arg1: the pin to read
	*	ret: the value of the given register bit
	*/
	uint8_t readBit(volatile uint8_t* reg, uint8_t bit);

#endif /* PORT_CONFIGURATION_H_ */