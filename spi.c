/*! \file spi.c \brief SPI interface driver. */
//*****************************************************************************
//
// File Name	: 'spi.c'
// Title		: SPI interface driver
// Author		: Pascal Stang - Copyright (C) 2000-2002 - Modified by Mikael Larsmark
// Created		: 11/22/2000
// Revised		: 06/06/2002
// Version		: 0.6
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
// NOTE: This code is currently below version 1.0, and therefore is considered
// to be lacking in some functionality or documentation, or may not be fully
// tested.  Nonetheless, you can expect most functions to work.
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>

#include "spi.h"

// Define the SPI_USEINT key if you want SPI bus operation to be
// interrupt-driven.  The primary reason for not using SPI in
// interrupt-driven mode is if the SPI send/transfer commands
// will be used from within some other interrupt service routine
// or if interrupts might be globally turned off due to of other
// aspects of your program
//
// Comment-out or uncomment this line as necessary
//#define SPI_USEINT

// global variables
volatile u08 spiTransferComplete;

// SPI interrupt service handler
#ifdef SPI_USEINT
SIGNAL(SIG_SPI)
{
	spiTransferComplete = TRUE;
}
#endif

// access routines
void spiInit()
{
	// setup SPI I/O pins
	sbi(PORTB, 1);		// set SCK hi
	sbi(DDRB, 1);		// set SCK as output
	cbi(DDRB, 3);		// set MISO as input
	sbi(DDRB, 2);		// set MOSI as output
	sbi(DDRB, 0);		// SS must be output for Master mode to work
	
	// setup SPI interface :
	// master mode
	sbi(SPCR, MSTR);
	// clock = f/128
	sbi(SPCR, SPR0);
	sbi(SPCR, SPR1);
	// select clock phase negative-going in middle of data
	cbi(SPCR, CPOL);
	cbi(SPCR, CPHA);
	// Data order MSB first
	cbi(SPCR,DORD);
	// enable SPI
	sbi(SPCR, SPE);

	// clear status
	inb(SPSR);
	spiTransferComplete = TRUE;
}

void spiSendByte(u08 data)
{
	// send a byte over SPI and ignore reply
	#ifdef SPI_USEINT
		while(!spiTransferComplete);
	#else
		while(!(inb(SPSR) & (1<<SPIF)));
	#endif

	spiTransferComplete = FALSE;
	outb(SPDR, data);
}

u08 spiTransferByte(u08 data)
{
/*	// make sure interface is idle
	#ifdef SPI_USEINT
		while(!spiTransferComplete);
	#else
		while(!(inb(SPSR) & (1<<SPIF)));
	#endif
*/
	// send the given data
	spiTransferComplete = FALSE;
	outb(SPDR, data);

	// wait for transfer to complete
	#ifdef SPI_USEINT
		while(!spiTransferComplete);
	#else
		while(!(inb(SPSR) & (1<<SPIF)));
		// *** reading of the SPSR and SPDR are crucial
		// *** to the clearing of the SPIF flag
		// *** in non-interrupt mode
		//inb(SPDR);
		// set flag
		spiTransferComplete = TRUE;
	#endif
	// return the received data
	return inb(SPDR);
}

u16 spiTransferWord(u16 data)
{
	u16 rxData = 0;

	// send MS byte of given data
	rxData = (spiTransferByte((data>>8) & 0x00FF))<<8;
	// send LS byte of given data
	rxData |= (spiTransferByte(data & 0x00FF));

	// return the received data
	return rxData;
}
