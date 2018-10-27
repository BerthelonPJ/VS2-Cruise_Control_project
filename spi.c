/*
 * spi.c
 *
 * Created: 8/24/2018 10:31:43 AM
 *  Author: Jan van Deventer
 * The purpose of these function is to introduce the on board accelerometer which we interact with via SPI.
 * Updated on August 29, 2018.
 */

#include <avr/io.h> // input output header file for this AVR chip.
#include "spi.h"

void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS); //Master out Slave in (MOSI) is an output, Clock is an output, Slave select at output(this limits the master to be both but OK for here)
	DDR_SPI &= ~(1<<DD_MISO);	//Master in Slave out (MISO) is an input
	PORTB |= (1<<DD_SS);		//set the slave select high: idle
	/* Enable SPI, Master, set clock rate fck/64 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
}

void SPI_MasterTransmit(uint8_t rwAddress,uint8_t cData)
{
	/*NOTE: The command byte of SPI bus is:
	(MSB) R/W A A A A A A 0 (LSB)
	|  |         | |
	|  |         |  - Don't care bit
	|   ------------- Register address,
	---------------- Read/Write bit (1=write, 0=read)*/

	/* Start transmission */
	SPDR = rwAddress;
	PORTB &= ~(1<<DD_SS); //start transmission by pulling low the Slave Select line
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));

	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));

	PORTB |= (1<<DD_SS); //Return to idle mode
}

accelSPI acc(void){
	accelSPI temp;	// temp is an instance of an accelSPI structure (with local scope)

	SPI_MasterTransmit(0b00001100,0xff);//send command to the acc (X axis data read)
	// X-axis register is 0x6 = 0b0110 shifted left by 1, MSB is read so 0
	// The second byte is a dummy value
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF)));
	/* Return data register */
	temp.x=SPDR; // with compensation for individual board

	SPI_MasterTransmit(0b00001110,0xff);//send command to the acc (Y axis data read)
	// Y-axis register is 0x7 = 0b0111 shifted left by 1, MSB is read so 0
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF)));
	/* Return data register */
	temp.y=SPDR; // with compensation for individual board

	SPI_MasterTransmit(0b00010000,0xff);//send command to the acc (Z axis data read)
	// Z-axis register is 0x6 = 0b1000 shifted left by 1, MSB is read so 0
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF)));
	temp.z=SPDR;

	return temp;
}
