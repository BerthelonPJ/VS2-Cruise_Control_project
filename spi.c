/*
 * spi.c
 *
 * Created: 8/24/2018 10:31:43 AM
 *  Author: Jan van Deventer
 * The purpose of these function is to introduce the students to SPI.
 * In this case, we start using it in between boards rather than in between components on a board.
 */ 

#include <avr/io.h> // input output header file for this AVR chip.
#include "spi.h"

void SPI_MasterInit(void)
{
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS); //Master out Slave in is an output, Clock is an output, Slave select at output(this limits the master to be both but OK for here)
	DDR_SPI &= ~(1<<DD_MISO);	//Master in Slave out is an input
	PORTB |= (1<<DD_SS);		//set the slave select high: idle 
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

void SPI_SlaveInit(void)
{
	DDR_SPI |= (1<<DD_MISO);	/* Set MISO output */
	DDR_SPI &= ~(1<<DD_SS);		//Set slave select as an input
	/* Enable SPI */
	SPCR = (1<<SPE)|(1<<SPIE);	//Enable SPI and its interrupt
}

void SPI_MasterTransmit(char cData)
{
PORTB &= ~(1<<DD_SS); //start transmission by pulling low the Slave Select line
SPDR = cData;	// Load the data and start the transmission
while(!(SPSR & (1<<SPIF)));	/* Wait for transmission complete */
PORTB |= (1<<DD_SS); //Return to idle mode
}

char SPI_SlaveReceive(void)
{
/* Wait for reception complete */
while(!(SPSR & (1<<SPIF)));
/* Return data register */
return SPDR;
}