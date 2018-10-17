/*! \file ext_dev.h \brief EXT devices, such as leds etc
 * \author Mikael Larsmark
 * \date 2009-10-28
 */
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


#ifndef _EXT_DEV_H_
#define _EXT_DEV_H_

#define BUTTON_ALL	0
#define BUTTON_S1		1
#define BUTTON_S2		2
#define BUTTON_S3		3
#define BUTTON_S4		4
#define BUTTON_S5		5

void ext_dev_led_on(unsigned char led_nr);
void ext_dev_led_off(unsigned char led_nr);

unsigned char ext_dev_read_dip(void);

void ext_dev_relay_on(void);
void ext_dev_relay_off(void);

unsigned char ext_dev_read_button(unsigned char button);

void ext_dev_lcd_backlight(unsigned char level);

#endif

