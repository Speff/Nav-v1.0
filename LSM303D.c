/*
 * LSM303D.c
 *
 * Created: 12/12/2015 9:23:38 PM
 *  Author: Sahil
 */ 

#include "LSM303D.h"

void enableSPI(void){
	//PRR0 &= ~(1<<PRSPI);						// PRSPI - Disable SPI power saving
	SPCR = (1<<SPE) | (1<<MSTR) |		// SPE- Enable SPI, MSTR-Set SPI Master
			(1<<CPOL) | (1<<CPHA) | (1<<SPR0);	// CPOL - Clock high on IDLE, CPHA - Sample on rising edge, SPR0 - CLK/16
}

void disableSPI(void){
	SPCR &= ~(1<<SPE);		// SPE - Disable SPI
	//PRR0 |= (1<<PRSPI);		// PRSPI - Enable SPI power save mode
}

uint8_t writeByteToSPI(uint8_t data){
	SPDR = data;

	while(!(SPSR & (1<<SPIF)));
	
	return(SPDR);
}