/***************************************************************************
 *   Copyright (C) 2016 by Mikael Larsmark, Conex Engineering              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>

#include "fxas21002c.h"
#include "init.h"
#include "usart.h"
#include "spi.h"
#include "delay.h"

#define DEBUG_FXAS21002C

unsigned char fxas21002c_found = 0;

/*! \brief Initializes the FXAS21002C gyro
 *  \return 1 if init was successful, 0 if it failed */
unsigned char fxas21002c_init(void) {
	unsigned char retval=0xFF;

	//Set the direction of the configured PIN for the chip select to output
	FXAS21002C_CS_DDR |= (1<<FXAS21002C_CS_PIN);

	FXAS21002C_CS_HIGH;
	
	delay_ms(100);

	FXAS21002C_CS_LOW;
	spiTransferByte((1<<7) | 0x0C);
	retval = spiTransferByte(0x00);
	FXAS21002C_CS_HIGH;
	
	if (retval == 0xD7) {
		#ifdef DEBUG_FXAS21002C
		printf("FXAS21002C >> Device found\n");
		#endif
		
		fxas21002c_found = 1;
	}
	else {
		#ifdef DEBUG_FXAS21002C
		printf("FXAS21002C >> Not deteced, SPI CS not connected?\n");
		#endif
		
		fxas21002c_found = 0;
		return(0);
	}
	
	return(1);
}

/*! \brief Check if the FXAS21002C is found on the SPI bus
 *  \return 1 if the FXAS21002C is found, 0 if not found */
unsigned char fxas21002c_device_found(void) {
	return(fxas21002c_found);
}