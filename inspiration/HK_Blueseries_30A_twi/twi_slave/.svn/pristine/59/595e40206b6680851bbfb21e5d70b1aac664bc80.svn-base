/*************************************************************************
 * TWI Slave library functions                                           *
 *                                                                       *
 * Copyright (C) 2011 by Anton 'TwisteR' Dubniak <twister@tfsoft.org.ua> *
 *                                                                       *
 * This program is free software: you can redistribute it and/or modify  *
 * it under the terms of the GNU General Public License as published by  *
 * the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                   *
 *                                                                       *
 * This program is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 * GNU General Public License for more details.                          *
 *                                                                       *
 * You should have received a copy of the GNU General Public License     *
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *************************************************************************/

#ifndef _TWI_SLAVE_H
#define _TWI_SLAVE_H

//*********************************************************************
// This library should be located in 'twi_slave' subdir of your project
//*********************************************************************
#include "../parameters.h"
#include <stdbool.h>

//************************************************************
// These constants should be defined in project's parameters.h
//************************************************************
// TWI_DEBUG
// TWI_BUFFER_SIZE
// TWI_OWN_ADDR

// TWI flags
#define TWI_BUSY	0
#define TWI_NEW_DATA	1
// reserved for future use
#define TWI_LAST_TX_OK	2
#define TWI_GEN_CALL	3

// Flag manipulation
#define SET_TWI_FLAG(flag)	twi_status |= (uint8_t) _BV(flag)
#define CLR_TWI_FLAG(flag)	twi_status &= (uint8_t)~_BV(flag)
#define GET_TWI_FLAG(flag)	(twi_status & (uint8_t) _BV(flag)) != 0

void	twi_slave_init(uint8_t, bool);
int8_t	twi_slave_get_data(uint8_t *);
int8_t	twi_slave_send_data(uint8_t *, uint8_t);

#endif // _TWI_SLAVE_H

