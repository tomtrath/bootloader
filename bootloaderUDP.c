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
/**
 * \brief	UDP Bootloader
 * 
 * 
 * \version	$Id$
 * \author	Fabian Greif <fabian.greif@rwth-aachen.de>
 * \author	Adrian Weiler
 * \author	Thomas Trathnigg
 */
// ----------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "utils.h"
#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"
#include "net.h"
#include "homecan.h"
#include "bootloader.h"


// ----------------------------------------------------------------------------
// globale Variablen
static uint8_t rxbuf[BUFFER_SIZE_RX+1];

static uint8_t mymac[6] = {0x00,0x04,0xA3,0x00,0x00,0x03};

static uint8_t myip[4] = {192,168,1,12};

static uint16_t flashpage = 0;
static uint8_t page_buffer_pos = 0;
static uint8_t page_buffer[SPM_PAGESIZE];

volatile uint8_t deviceID = 0xFF;
static volatile uint16_t timer;
static volatile uint8_t timeout;

// -----------------------------------------------------------------------------
// Watchdog Timer als erstes im Programm deaktivieren
// siehe http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html

void
disable_watchdog(void) \
		__attribute__((naked)) \
		__attribute__((section(".init3")));

void
disable_watchdog(void)
{
	// Save MCUSR so that the main program can access if later
	GPIOR0 = MCUSR;
	MCUSR = 0;
	wdt_disable();
}

// ----------------------------------------------------------------------------
/**
 * \brief	starts the application program
 */
void
boot_jump_to_application(void)
{
	TCCR0A = 0; //disable timer0

	cli();

	// relocate interrupt vectors
	uint8_t reg = MCUCR & ~((1 << IVCE) | (1 << IVSEL));
	
	MCUCR = reg | (1 << IVCE);
	MCUCR = reg;
	
	// reset SPI interface to power-up state
	SPCR = 0;
	SPSR = 0;
	
#if FLASHEND > 0xffff
	__asm__ __volatile__(
			"push __zero_reg__" "\n\t"
			"push __zero_reg__" "\n\t"
			"push __zero_reg__" "\n\t");
#else
	__asm__ __volatile__(
			"push __zero_reg__" "\n\t"
			"push __zero_reg__" "\n\t");
#endif
	
	// when the functions executes the 'ret' command to return to
	// its origin the AVR loads the return address from the stack. Because we
	// pushed null it instead jumps to address null which starts the main
	// application.
}

// ----------------------------------------------------------------------------
/**
 * \brief	write a complete page to the flash memorey
 * 
 * \param	page	page which should be written
 * \param	*buf	Pointer to the buffer with the data
 *
 * \see		avr-libc Documentation > Modules > Bootloader Support Utilities
 */
void
boot_program_page(uint16_t page, uint8_t *buf)
{
	uint32_t adr = page * SPM_PAGESIZE;
	
	boot_page_erase(adr);
	boot_spm_busy_wait();	  // Wait until the memory is erased.
	
	for (uint16_t i=0; i < SPM_PAGESIZE; i+=2)
	{
		// Set up little-endian word.
		uint16_t w = *buf++;
		w += (*buf++) << 8;
		
		boot_page_fill(adr + i, w);
	}
	
	boot_page_write(adr);		// Store buffer in flash page.
	boot_spm_busy_wait();		// Wait until the memory is written.
	
	// Reenable RWW-section again. We need this if we want to jump back
	// to the application after bootloading.
	boot_rww_enable();
}

void readDeviceIDFromEEPROM(void) {
	deviceID = eeprom_read_byte(EEPROM_DEVICE_ID);
}

//
ISR(TIMER0_COMP_vect )
{
	timer++;
	if (timer>BOOTLOADER_TIMEOUT+(uint16_t)deviceID) {
		timeout = 1;
	}
}

// ----------------------------------------------------------------------------
int
main(void) __attribute__((OS_main));

int
main(void)
{
	enum { 
		IDLE,
		COLLECT_DATA,
		RECEIVED_PAGE
	} state = IDLE;
	uint8_t next_message_number = -1;
	
	// Relocate interrupt vectors to boot area
	MCUCR = (1 << IVCE);
	MCUCR = (1 << IVSEL);
	
	disable_watchdog();

	readDeviceIDFromEEPROM();
	
	//setup timer0, interrupt every
	TCCR0A = (1<<WGM01) | (0<<WGM00) | (0<<COM0A1) | (0<<COM0A0) | (1<<CS02) | (0<<CS01) | (1<<CS00); //clk/1024, CTC mode, pin output deactivated
	OCR0A = 156; //set to 10ms intervall for overflow interrupt
	TIMSK0 = (1<<OCIE0A) | (0<<TOIE0);	//enable overflow interrupt
	timer = 0;
	timeout = 0;

	enc28j60Init(mymac);

	// Magjack leds configuration, see enc28j60 datasheet, page 11
	// LEDB=yellow LEDA=green
	// 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
	enc28j60PhyWrite(PHLCON,0x476);

	//init the ethernet/ip layer:
	init_udp_or_www_server(mymac,myip);

    //init CAN port LED
    PORTE = 1<<PE2;
    DDRE = 1<<PE2;

	sei();

	while (1)
	{
		uint16_t page;
		static uint8_t next_message_data_counter;
		uint16_t plen = 0;

		//receive a new message
		plen = enc28j60PacketReceive(BUFFER_SIZE_RX, rxbuf);
		packetloop_arp_icmp_tcp(rxbuf,plen);
		if (timeout) boot_jump_to_application();
		if (plen!=0) {
			PORTE ^= 1<<PE2;
			if (rxbuf[IP_PROTO_P]==IP_PROTO_UDP_V){
				//check if BOOTLOADER port and correct packet size
				if (rxbuf[UDP_DST_PORT_H_P]==HOMECAN_UDP_PORT_BOOTLOADER>>8 && rxbuf[UDP_DST_PORT_L_P]==(HOMECAN_UDP_PORT_BOOTLOADER&0xFF) && rxbuf[UDP_LEN_L_P]-UDP_HEADER_LEN==8) {
					//check if packet is addressed to this device
					if (rxbuf[UDP_DATA_P]==deviceID) {
						uint8_t* packet = &rxbuf[UDP_DATA_P];
						uint8_t* packet_data = &rxbuf[UDP_DATA_P+4];
						uint8_t packetlen = rxbuf[UDP_LEN_L_P]-UDP_HEADER_LEN;
						uint8_t packetlen_data = packetlen - 4;

						// check if the message is a request, otherwise reject it
						if ((packet[1] & REQRESPONSE_MASK) != REQUEST)
							continue;

						packet[1] &= COMMAND_MASK;

						// check message number
						next_message_number++;
						if (packet[2] != next_message_number)
						{
							if (packet[1]==IDENTIFY) {
								//IDENTIFY always allowed, reset message_number state to start value
								next_message_number = 0;
							} else {
								// wrong message number => send NACK
								packet[2] = next_message_number;
								next_message_number--;
								packet[1]|= WRONG_NUMBER_REPSONSE;
								make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
								continue;
							}
						}

						// process command
						switch (packet[1])
						{
						case IDENTIFY:
							//Stop Bootloader timer
							TCCR0A = (1<<WGM01) | (0<<WGM00) | (0<<COM0A1) | (0<<COM0A0) | (0<<CS02) | (0<<CS01) | (0<<CS00);
							// version and command of the bootloader
							packet_data[0] = BOOTLOADER_VERSION;
							packet_data[1] = PAGESIZE_IDENTIFIER;

							// number of writeable pages
							packet_data[2] = HIGH_BYTE(RWW_PAGES);
							packet_data[3] = LOW_BYTE(RWW_PAGES);

							packet[1] = IDENTIFY | SUCCESSFULL_RESPONSE;
							make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
							break;

							// --------------------------------------------------------------------
							// set the current address in the page buffer
						case SET_ADDRESS:
							page = (packet_data[0] << 8) | packet_data[1];

							if (packetlen_data == 4 &&
									packet_data[2] < (SPM_PAGESIZE / 4) &&
									page < RWW_PAGES)
							{
								flashpage = page;
								page_buffer_pos = packet_data[3];

								state = COLLECT_DATA;

								packet[1] = SET_ADDRESS | SUCCESSFULL_RESPONSE;
								make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
							}
							else {
								goto error_response;
							}
							break;

							// --------------------------------------------------------------------
							// collect data
						case DATA:
							if (packetlen_data != 4 ||
									page_buffer_pos >= (SPM_PAGESIZE / 4) ||
									state == IDLE) {
								state = IDLE;
								goto error_response;
							}

							// check if the message starts a new block
							if (packet[3] & START_OF_MESSAGE_MASK)
							{
								packet[3] &= ~START_OF_MESSAGE_MASK;		// clear flag
								next_message_data_counter = packet[3];
								state = COLLECT_DATA;
							}

							if (packet[3] != next_message_data_counter) {
								//state = IDLE;
								//request again using WRONG_NUMBER_REPSONSE
								packet[3] = next_message_data_counter;
								packet[1] |= WRONG_NUMBER_REPSONSE;
								make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
								continue;
							}
							next_message_data_counter--;

							// copy data
							memcpy(page_buffer + page_buffer_pos * 4, &packet_data[0], 4);
							page_buffer_pos++;

							if (packet[3] == 0)
							{
								if (page_buffer_pos == (SPM_PAGESIZE / 4))
								{
									packet_data[0] = flashpage >> 8;
									packet_data[1] = flashpage & 0xff;

									if (flashpage >= RWW_PAGES) {
										packetlen_data = 2;
										goto error_response;
									}

									boot_program_page( flashpage, page_buffer );
									page_buffer_pos = 0;
									flashpage += 1;

									// send ACK
									packet[1] = DATA | SUCCESSFULL_RESPONSE;
									make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
								}
								else {
									packet[1] = DATA | SUCCESSFULL_RESPONSE;
									make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
								}
							}
							break;

							// --------------------------------------------------------------------
							// start the flashed application program
						case START_APP:
							packet[1] = START_APP | SUCCESSFULL_RESPONSE;
							make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);

							// wait for the mcp2515 to send the message
							_delay_ms(200);

							// start application
							boot_jump_to_application();
							break;

							// --------------------------------------------------------------------
							/*
					case CHIP_ERASE:
						// erase complete flash except the bootloader region
						for (uint16_t i = 0; i < RWW_PAGES; i++ )
						{
							uint32_t adr = i * SPM_PAGESIZE;

							boot_page_erase( adr );
							boot_spm_busy_wait();
						}
						boot_rww_enable();

						packet[1] = CHIP_ERASE | SUCCESSFULL_RESPONSE;
						make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen-4,HOMECAN_UDP_PORT_BOOTLOADER);

						break;
							 */
							// --------------------------------------------------------------------
							// change the deviceID
						case CHANGE_ID:
							deviceID = packet_data[0];
							eeprom_write_byte((uint8_t *)EEPROM_DEVICE_ID,deviceID);
							packet[1] = CHANGE_ID | SUCCESSFULL_RESPONSE;
							make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
							break;

							error_response:
						default:
							packet[1] |= ERROR_RESPONSE;
							make_udp_reply_from_request_udpdat_ready(rxbuf,packetlen,HOMECAN_UDP_PORT_BOOTLOADER);
							break;
						}
					}
				}
			}
		}
	}
}
