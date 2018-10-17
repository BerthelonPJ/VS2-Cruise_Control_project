/***************************************************************************
 *   Copyright (C) 2009 by Mikael Larsmark, Luleå University of Technology *
 *   larsmark@ltu.se                                                       *
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

#include "init.h"
#include "usart.h"

/*! This define is calculated so that timer 0 generates interrupts every ~1 ms */
#define OCR0_1MS	16

/*! Initialize all the ports used for the board. Change here if you wish to change
    the direction of some port. Also sets the interrupt for the "common" button input */
void init_ports(void) {
	DDRA = 0xFF;
	DDRB = 0xF6;
	DDRC = 0x07;
	DDRD = 0xA4;
	DDRE = 0x02;
	DDRF = 0x0F;
	DDRG = 0x03;
	
	//Trigger on rising edge
	EICRB |= (0<<ISC61) | (1<<ISC60);
	EIMSK |= (1<<INT6);
	
	//Set pull-ups for the DIP-switch
	PORTE |= (1<<2);
	PORTE |= (1<<3);
}

/*! Initialize the usart which is connected to the USB port
   This function will also enable so that printf works as std io */
void init_usb_usart(unsigned char baudrate) {
	usart1_init(baudrate);
	
	//Make printf work as output to USB
	fdevopen((void*)usart1_transmit, (void*)usart1_receive_loopback);
}

/*! Initialize the PWM output for the backlight level adjustment */
void init_backlight(void) {
	
	TCCR1A = (0<<COM1C1) | (1<<COM1C0) | (0<<WGM13) | (1<<WGM12) | (0<<WGM11)| (1<<WGM10);
	TCCR1B = (0<<CS32)|(0<<CS31)|(1<<CS30);
}

/*! Initialize timer 0 to have 1ms interrupt interval */
void init_timer_0(void) {
	TCCR0A = 0;
	TIMSK0 |= (1<<OCIE0A);	/* enable output compare interrupt */
	TCCR0A  = (1<<WGM01)|(1<<CS02)|(0<<CS01)|(1<<CS00); /* CTC, prescale = 1024 */
	TCNT0  = 0;
	OCR0A   = OCR0_1MS;		/* match in aprox 1 ms,  */
}