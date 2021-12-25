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

#include "port_configuration.h"

void setPortPin(portpin target, uint8_t *ddr, uint8_t *port, uint8_t *pin, uint8_t bit)
{
	target.ddr = ddr;
	target.port = port;
	target.pin = pin;
	target.bit = bit;	
}

volatile uint8_t *getReg(portpin target, uint8_t reg)
{
	if(reg == DDR)
		return target.ddr;
	else if(reg == PIN)
		return target.pin;	
	return target.port;
}

void setPin(portpin target, uint8_t reg, uint8_t value)
{
	if(value == 1)
		*getReg(target, reg) |= (1 << target.bit);
	else
		*getReg(target, reg) &= ~(1 << target.bit);
}

void togglePin(portpin target, uint8_t reg)
{
	setPin(target, reg, !readPin(target, reg));
}

uint8_t readPin(portpin target, uint8_t reg)
{
	return (*getReg(target, reg) >> target.bit) & 1;
}

void copyPin(portpin source, uint8_t regSource, portpin target, uint8_t regTarget)
{
	setPin(source, regSource, readPin(target, regTarget));
}


void setBit(volatile uint8_t* reg, uint8_t bit, uint8_t value)
{
	if(value == 0)
		*reg &= ~(1 << bit);
	else if(value == 1)
		*reg |= (1 << bit);
}

void copyBit(volatile uint8_t* regFrom, uint8_t bitFrom, volatile uint8_t* regTo, uint8_t bitTo)
{
	setBit(regTo, bitTo, readBit(regFrom, bitFrom));	
}

uint8_t readBit(volatile uint8_t* reg, uint8_t bit)
{
	return (*reg >> bit) & 1;	
}