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

#include <stdio.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "twi_slave.h"

static uint8_t twi_buf[TWI_BUFFER_SIZE];
static uint8_t twi_status;

/*******************************************************************************
 Call this function to set the device's address and init the TWI hardware.

 Arguments:
	1-st	: device's own address on a TWI bus
	2-nd	: TRUE to accept TWI General calls, FALSE to ignore them
*******************************************************************************/
void twi_slave_init(uint8_t twi_own_addr, bool accept_gen_calls)
{
	// set our own TWI address and TWI General Call recognition
	TWAR = (twi_own_addr << 1) | (accept_gen_calls << TWGCE);

	// set the TWI control register
	TWCR =	_BV(TWINT)|	// clear the interrupt flag
		_BV(TWEA) |	// enable TWI Acknowledge bit
		_BV(TWEN) |	// activate the TWI interface
		_BV(TWIE);	// enable TWI

#ifdef DEBUG
	printf("\ntwi_slave_init: addr=0x%X, GCE=%d\n", twi_own_addr, 
		accept_gen_calls);
#endif
}

/*******************************************************************************
 Use this function to receive data from the TWI interface. Make sure, that the
 *msg buffer is large enough to hold at least TWI_BUFFER_SIZE bytes.

 Arguments:
	1-st	: pointer to the buffer for incoming data.

 Return value:
	If no error occurs, this function returns the number of bytes received.
	Otherwise an error code is returned:
	0	if there is no new data since the last function call
	-1	if TWI hardware is busy
*******************************************************************************/
int8_t twi_slave_get_data(uint8_t *msg)
{
	uint8_t bytes_read = 0;

	// are we busy right now?
	if (GET_TWI_FLAG(TWI_BUSY))
	{
		return -1;
	}

	// are there any new data?
	if (GET_TWI_FLAG(TWI_NEW_DATA))
	{
		// copy data from twi_buf to *msg
		for (bytes_read = 0; bytes_read < TWI_BUFFER_SIZE; bytes_read++)
		{
			msg[bytes_read] = twi_buf[bytes_read];
#ifdef DEBUG
			printf("get: twi_buf[%d] = %d\n", bytes_read, twi_buf[bytes_read]);
#endif
		}
		// there are no new data anymore
		CLR_TWI_FLAG(TWI_NEW_DATA);
	}

	return bytes_read;
}

/*******************************************************************************
 Use this function to transmit data to the TWI interface.

 Arguments:
	1-st	: pointer to the data to be transmitted
	2-nd	: how many bytes should be sent

 Return value:
	If no error occurs, this function returns the number of bytes, queued
	for transfer. Otherwise an error code is returned:
	-1	if TWI hardware is busy
	-2	if message is larger than space in buffer or zero-length message
*******************************************************************************/
int8_t twi_slave_send_data(uint8_t *msg, uint8_t msg_size)
{
	uint8_t i = 0;

	// are we busy right now?
	if (GET_TWI_FLAG(TWI_BUSY))
	{
		return -1;
	}

	// is there enough buffer space?
	if ((msg_size <= 0) || (msg_size > TWI_BUFFER_SIZE))
	{
		return -2;
	}

	// copy our message to TWI buffer
	for (i = 0; i < msg_size; i++)
	{
		twi_buf[i] = msg[i];
#ifdef DEBUG
		printf("send: twi_buf[%d] = %d\n", i, msg[i]);
#endif
	}

	// TWI module is busy now
	SET_TWI_FLAG(TWI_BUSY);
	// set the TWI control register
	TWCR =	_BV(TWINT)|	// clear the interrupt flag
		_BV(TWEA) |	// enable TWI Acknowledge bit
		_BV(TWEN) |	// activate the TWI interface
		_BV(TWIE);	// enable TWI interupt

	return msg_size;
}

/*******************************************************************************
  Interrupt service routine. Do not call this function from main application.
*******************************************************************************/
ISR(TWI_vect)
{
	static uint8_t twi_buf_ptr;

	switch (TW_STATUS)
	{
	//
	// Slave Transmitter mode
	//
		// SLA+R received, ACK returned
		case TW_ST_SLA_ACK:
#ifdef DEBUG
			printf("TRANSMIT: ");
#endif
			twi_buf_ptr = 0;

		// data transmitted, ACK received
		case TW_ST_DATA_ACK:
			// check for buffer overflow
			if (twi_buf_ptr < TWI_BUFFER_SIZE)
			{
				TWDR = twi_buf[twi_buf_ptr++];
#ifdef DEBUG
				printf("0x%X, ", TWDR);
#endif
				TWCR =	_BV(TWINT)| // clear the interrupt flag
					_BV(TWEA) | // enable TWI Acknowledge
					_BV(TWEN) | // enable the TWI interface
					_BV(TWIE);  // enable TWI interupt
				SET_TWI_FLAG(TWI_BUSY);
			}
			else
			{
#ifdef DEBUG
				printf("NACK\n");
#endif
				// nothing to transmit, return NACK
				TWCR =	_BV(TWINT)| // clear the interrupt flag
					_BV(TWEN) | // enable the TWI interface
					_BV(TWIE);  // enable TWI interupt
				// TWI interface is not busy now
				CLR_TWI_FLAG(TWI_BUSY);
			}
		break;

	//
	// Slave Receiver mode
	//
		// SLA+W received, ACK returned
		// Now we should reset the buffer pointer and get ready to store
		// incoming data
		case TW_SR_SLA_ACK:
#ifdef DEBUG
			printf("RECEIVE: ");
#endif
			twi_buf_ptr = 0;
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			SET_TWI_FLAG(TWI_BUSY);
		break;

		// data received, ACK returned
		case TW_SR_DATA_ACK:
			// check for buffer overflow
			if (twi_buf_ptr < TWI_BUFFER_SIZE)
			{
				twi_buf[twi_buf_ptr++] = TWDR;
#ifdef DEBUG
				printf("0x%X, ", TWDR);
#endif
				TWCR =	_BV(TWINT)| // clear the interrupt flag
					_BV(TWEA) | // enable TWI Acknowledge
					_BV(TWEN) | // enable the TWI interface
					_BV(TWIE);  // enable TWI interupt
			}
			else
			{
#ifdef DEBUG
				printf("NACK\n");
#endif
				// buffer is full, return NACK
				TWCR =	_BV(TWINT)| // clear the interrupt flag
					_BV(TWEN) | // enable the TWI interface
					_BV(TWIE);  // enable TWI interupt
				// TWI interface is not busy now
				CLR_TWI_FLAG(TWI_BUSY);
			}

			SET_TWI_FLAG(TWI_NEW_DATA);

		break;

	//
	// Transmission finishing / Error recovery
	//
		// data transmitted, NACK received 
		case TW_ST_DATA_NACK:
		// stop or repeated start condition received while selected 
		case TW_SR_STOP:
		// data received, NACK returned 
		case TW_SR_DATA_NACK:
		// last data byte transmitted, ACK received
		case TW_ST_LAST_DATA:
		// illegal start or stop condition
		case TW_BUS_ERROR:
#ifdef DEBUG
			printf("STOP [0x%X]\n", TW_STATUS);
#endif
			TWCR =	_BV(TWINT)| // clear the interrupt flag
				_BV(TWSTO)| // transmit a stop condition
				_BV(TWEA) | // enable TWI Acknowledge
				_BV(TWEN) | // enable the TWI interface
				_BV(TWIE);  // enable TWI interupt

			CLR_TWI_FLAG(TWI_BUSY);
			twi_buf_ptr = 0;
		break;

		default:
#ifdef DEBUG
			printf("RESTART [0x%X]\n", TW_STATUS);
#endif
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			CLR_TWI_FLAG(TWI_BUSY);
	}
}
