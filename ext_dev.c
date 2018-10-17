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

#include "ext_dev.h"

/*! \brief Turn a led on
 * This function is used to turn on the leds
 * \param led_nr The index of which led you wish to turn on (1-5) */
void ext_dev_led_on(unsigned char led_nr) {
	if (led_nr == 1)
		PORTG |= (1<<0);
	else if (led_nr == 2)
		PORTG |= (1<<1);
	else if (led_nr == 3)
		PORTC |= (1<<0);
	else if (led_nr == 4)
		PORTC |= (1<<1);
	else if (led_nr == 5)
		PORTC |= (1<<2);
}

/*! \brief Turn a led off
 * This function is used to turn off the leds
 * \param led_nr The index of which led you wish to turn off (1-5) */
void ext_dev_led_off(unsigned char led_nr) {
	if (led_nr == 1)
		PORTG &= ~(1<<0);
	else if (led_nr == 2)
		PORTG &= ~(1<<1);
	else if (led_nr == 3)
		PORTC &= ~(1<<0);
	else if (led_nr == 4)
		PORTC &= ~(1<<1);
	else if (led_nr == 5)
		PORTC &= ~(1<<2);
}

/*! \brief Read the DIP switch status 
 *  This function will read the DIP switch located on the development board and return
 *  its value.
 * \return Dip switch value (0-3) */
unsigned char ext_dev_read_dip(void) {
	unsigned char temp = 0x03;
	
	if (PINE & (1<<3))
		temp &= ~(1<<0);
	if (PINE & (1<<2))
		temp &= ~(1<<1);
	
	return(temp);
}

/*! \brief Turn the relay on
 *  This function will turn on the relay which is located on the development board, which
 *  can be used to control external devices. A LED located on the board will also light up
 *  with a red light if the relay is turned on */
void ext_dev_relay_on(void) {
	PORTD |= (1<<7);
}

/*! \brief Turn the relay off
 *  This function will turn off the relay which is located on the development board, which
 *  can be used to control external devices. A LED located on the board will also light up
 *  with a red light if the relay is turned on, and will be turned off when the relay is off */
void ext_dev_relay_off(void) {
	PORTD &= ~(1<<7);
}

/*! \brief Read the status of the buttons S1-S5
 *  This function will read the status of the buttons you have selected to read. 
 *  \param button Which buttons you wish to read, either BUTTON_ALL or BUTTON_S1, BUTTON_S2 .. BUTTON_S5
 *  \return 1 if button is pressed, 0 if not */
unsigned char ext_dev_read_button(unsigned char button) {
	unsigned char temp = 0;
	
	if (button == BUTTON_ALL)
		temp = ((PINC & 0xF8)>>3);
	else if (button == BUTTON_S1)
		temp = ((PINC & (1<<3))>>3);
	else if (button == BUTTON_S2)
		temp = ((PINC & (1<<4))>>4);
	else if (button == BUTTON_S3)
		temp = ((PINC & (1<<5))>>5);
	else if (button == BUTTON_S4)
		temp = ((PINC & (1<<6))>>6);
	else if (button == BUTTON_S5)
		temp = ((PINC & (1<<7))>>7);

	return(temp);
}

void ext_dev_lcd_backlight(unsigned char level) {
	float percent = 255 - (level * 2.55f);
				
	if (level >= 100)
		OCR1C = 0x00;
	else
		OCR1C = (unsigned char)(percent);
}