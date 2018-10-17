/*! \file iis328dq.h \brief IIS328DQ 3-axis accelerometer driver
 * \author Mikael Larsmark, SM2WMV
 * \date 2016-04-26
 */
/***************************************************************************
 *   Copyright (C) 2016 by Mikael Larsmark, Luleå University of Technology *
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


#ifndef _IIS328DQ_H_
#define _IIS328DQ_H_

#define IIS328DQ_CS_DDR		DDRB
#define IIS328DQ_CS_PORT	PORTB
#define IIS328DQ_CS_PIN		0

//! Pull the chip select low (Active)
#define IIS328DQ_CS_LOW		IIS328DQ_CS_PORT &= ~(1<<IIS328DQ_CS_PIN)
//! Pull the chip select high (Not Active)
#define IIS328DQ_CS_HIGH	IIS328DQ_CS_PORT |= (1<<IIS328DQ_CS_PIN)

unsigned char iis328dq_init(void);

unsigned char iis328dq_device_found(void);

#endif
