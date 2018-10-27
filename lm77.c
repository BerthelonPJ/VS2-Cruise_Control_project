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

#include "lm77.h"
#include "i2c.h"

//TODO: Implement the critical limits etc for the lm77 temperature sensor

/*! \brief Reads the temperature from the LM77 sensor
 *  \return The temperature as a floating point value */
float lm77_read_temp(void) {
	unsigned char temp[2];

	temp[0] = 0;
	temp[1] = 0;

	i2cMasterReceiveNI(LM77_ADDR, 2, (unsigned char *)temp);

	temp[1] = (temp[1]>>3) | ((temp[0] & 0x07)<<5);
	temp[0] = (temp[0]>>3);

	int int_temp = (temp[0]<<8) | temp[1];

	//Check the sign bit - Not tested in minus degrees
	if (int_temp & 0x200) {
		int_temp = !int_temp + 1;

		return(int_temp * (-0.5f));
	}

	return(int_temp * 0.5f);
}
