// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2010, Roboterclub Aachen e.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Roboterclub Aachen e.V. nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ROBOTERCLUB AACHEN E.V. ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOTERCLUB AACHEN E.V. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */
// ----------------------------------------------------------------------------

#ifndef	DEFAULTS_H
#define	DEFAULTS_H

#define BOOTLOADER_TIMEOUT		1500	//15s
#define BUFFER_SIZE_RX 250
#define	COMMAND_MASK			0x0F
#define	REQRESPONSE_MASK		0x30
#define	START_OF_MESSAGE_MASK	0x80

typedef enum
{
	// every bootloader type has this commands
	IDENTIFY		= 1,
	SET_ADDRESS		= 2,
	DATA			= 3,
	START_APP		= 4,
	CHIP_ERASE		= 5,
	CHANGE_ID		= 6,

	REQUEST					= 0x00,
	SUCCESSFULL_RESPONSE	= 0x10,
	WRONG_NUMBER_REPSONSE	= 0x20,
	ERROR_RESPONSE			= 0x30,

	NO_MESSAGE		= 0x0F
} command_t;

// ----------------------------------------------------------------------------
// create pagesize identifier
#if	SPM_PAGESIZE == 32
	#define	PAGESIZE_IDENTIFIER		(0)
#elif SPM_PAGESIZE == 64
	#define	PAGESIZE_IDENTIFIER		(1)
#elif SPM_PAGESIZE == 128
	#define	PAGESIZE_IDENTIFIER		(2)
#elif SPM_PAGESIZE == 256
	#define	PAGESIZE_IDENTIFIER		(3)
#else
	#error	Strange value for SPM_PAGESIZE. Check the define!
#endif

// -----------------------------------------------------------------------------
// set current version of the bootloader
#define	BOOTLOADER_VERSION		2

// -----------------------------------------------------------------------------
// CAN settings
#ifndef	CAN_BITRATE
	#define	CAN_BITRATE		125
#endif

// ----------------------------------------------------------------------------
// Set a few AVR specific defines

#if defined(__AVR_AT90CAN32__)
	
	#define	RWW_PAGES	96
//	#define	RAMSTART	0x0100
	#define	SIG_FAMILY	0x95
	#define	SIG_DEVICE	0x81
	#define	TIMER_INTERRUPT_FLAG_REGISTER	TIFR1
	
#elif defined(__AVR_AT90CAN64__)
	
	#define	RWW_PAGES	224
//	#define	RAMSTART	0x0100
	#define	SIG_FAMILY	0x96
	#define	SIG_DEVICE	0x81
	#define	TIMER_INTERRUPT_FLAG_REGISTER	TIFR1
	
#elif defined(__AVR_AT90CAN128__)
	
	#define	RWW_PAGES	480
//	#define	RAMSTART	0x0100
	#define	SIG_FAMILY	0x97
	#define	SIG_DEVICE	0x81
	#define	TIMER_INTERRUPT_FLAG_REGISTER	TIFR1
	
#else
	#error	chosen AVR command is not supported yet!
#endif

// ----------------------------------------------------------------------------
// Timereinstellung fuer aktuelle Taktfrequenz auswaehlen (500ms)
//
// TIMER_PRELOAD = 65536 - (0.5s * F_CPU) / 1024

#if F_CPU == 16000000UL
	#define	TIMER_PRESCALER		(1<<CS12)|(1<<CS10)		// Prescaler = 1024
	#define	TIMER_PRELOAD		57724
#else
	#error	choosen F_CPU not supported yet!
#endif


#endif	// DEFAULTS_H
