/*! \file fxas21002c.h \brief FXAS21002C 3-axis gyro driver
 * \author Mikael Larsmark, SM2WMV
 * \date 2016-04-26
 */
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


#ifndef _FXAS21002C_H_
#define _FXAS21002C_H_

#define FXAS21002C_CS_DDR	DDRE
#define FXAS21002C_CS_PORT	PORTE
#define FXAS21002C_CS_PIN	5

//! Pull the chip select low (Active)
#define FXAS21002C_CS_LOW		FXAS21002C_CS_PORT &= ~(1<<FXAS21002C_CS_PIN)
//! Pull the chip select high (Not Active)
#define FXAS21002C_CS_HIGH		FXAS21002C_CS_PORT |= (1<<FXAS21002C_CS_PIN)

unsigned char fxas21002c_init(void);

unsigned char fxas21002c_device_found(void);

#endif
