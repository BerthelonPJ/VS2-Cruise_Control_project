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
#include <string.h>

#include "realtime_clock.h"
#include "i2c.h"

char rtc_registers[9] = {0,0,0,0,0,0,0,0,0};

/*! \brief Initialize the RTC
 * This function is used to initialize the real time clock with a specified time and date. In the example below
 * the date and time we wish to set is 2009-11-04, 13:50:30.
 * \param seconds The number of seconds 0-59 (example = 30)
 * \param minutes The number of minutes 0-59 (example = 50)
 * \param hours The number of hours 0-23 (example = 13)
 * \param date The day of the month, 1-31 (example = 4)
 * \param month The current month 1-12 (example = 11)
 * \param year The current year 0-99 (example = 9) 
 * \return The answer of the I2C communication */
unsigned char rtc_init(unsigned char seconds, unsigned char minutes, unsigned char hours, unsigned char date, unsigned char month, unsigned char year) {	
	rtc_registers[0] = 0;
	
	rtc_registers[1] = (seconds / 10)<<4;
	rtc_registers[1] += (seconds % 10);
	rtc_registers[2] = (minutes / 10)<<4;
	rtc_registers[2] += (minutes % 10);
	rtc_registers[3] = (hours / 10)<<4;
	rtc_registers[3] += (hours % 10);
	rtc_registers[5] = (date / 10)<<4;
	rtc_registers[5] += (date % 10);
	rtc_registers[6] = (month / 10)<<4;
	rtc_registers[6] += (month % 10);
	rtc_registers[7] = (year / 10)<<4;
	rtc_registers[7] += (year % 10);
	
	unsigned char res = i2cMasterSendNI(RTC_ADDR,8, (unsigned char *)rtc_registers);
	
	for (int i=0;i<9;i++)
		rtc_registers[i] = 0;
	
	return(res);
}

/*! \brief Get the time from the RTC
 * This function will return the current time of the I2C realtime clock.
 * \param time A pointer to a character array of size 8
 * \return The answer of the I2C communication */
unsigned char rtc_get_time(char *time) {
	unsigned char res = i2cMasterReceiveNI(RTC_ADDR,8,(unsigned char *)rtc_registers);

	//Convert the data to proper format
	unsigned char hours = ((rtc_registers[3] & 0x30)>>4)*10 + (rtc_registers[3] & 0x0F);
	unsigned char minutes = ((rtc_registers[2] & 0x70) >> 4) * 10 + (rtc_registers[2] & 0x0F);
	unsigned char seconds = ((rtc_registers[1] & 0x70) >> 4) * 10 + (rtc_registers[1] & 0x0F);
	
	//Copy the time into the *time pointer
	sprintf(time, "%02i:%02i:%02i",hours,minutes,seconds);
	
	return(res);
}

/*! \brief Get the date from the RTC
 * This function will return the current date of the I2C realtime clock.
 * \param time A pointer to a character array of size 8
 * \return The answer of the I2C communication */
unsigned char rtc_get_date(char *date) {
	unsigned char res = i2cMasterReceiveNI(RTC_ADDR,8,(unsigned char *)rtc_registers);

	//Convert the data to proper format
	unsigned char day = ((rtc_registers[5] & 0x30)>>4)*10 + (rtc_registers[5] & 0x0F);
	unsigned char month = ((rtc_registers[6] & 0x70) >> 4) * 10 + (rtc_registers[6] & 0x0F);
	unsigned char year = ((rtc_registers[7] & 0x70) >> 4) * 10 + (rtc_registers[7] & 0x0F);
	
	//Copy the time into the *time pointer
	sprintf(date, "%02i-%02i-%02i",year,month,day);
	
	return(res);
}
