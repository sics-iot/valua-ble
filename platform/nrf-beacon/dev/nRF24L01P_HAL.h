/*
 * Copyright (c) 2015, Robert Bosch Center for Cyber Physical Systems, IISc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 *
 * -----------------------------------------------------------------
 *
 * Author  : Gurinder Singh Gil <gsgill112@gmail.com>
 * Created : 2015
 * Updated : $Date: 2015/05/05 20:23:02 $
 *           $Revision: 1.0 $
 */


#ifndef NRF24L01P_HAL_H_
#define NRF24L01P_HAL_H_

/**********************************************************************************************
 * INCLUDES
 *********************************************************************************************/
#include "nRF24L01.h"

/**********************************************************************************************
 * CONSTANTS
 *********************************************************************************************/
//#define MOSI - gsgill 
//#define MISO
//#define SCK
//#define CS

/**********************************************************************************************
 * VARIABLES
 *********************************************************************************************/
uint8_t MAC[6];

/**********************************************************************************************
 * FUNCTIONS - API
 *********************************************************************************************/

/*
 * Initialization function for nRf Radio 
 */
void nrf24_init(void);

/*
 * Checks if Data is available for rx 
 */
uint8_t nrf24_dataReady(void);

/*
 * Chck if radio is sending data 
 */
uint8_t nrf24_isSending(void);

/*
 * Get the status of the radio 
 */
uint8_t nrf24_getStatus(void);

/*
 * Check if Rx fifo is empty
 */
uint8_t nrf24_rxFifoEmpty(void);

/*
 * Reads payload data 
 */
uint8_t nrf24_getData(void *data, uint16_t len);

/*
 * Send the payload data
 */
uint8_t nrf24_send(void *data, uint16_t len);

/*
 * Enable/Disable Rx power 
 */
void nrf24_rxPower(uint8_t EN);

/*
 * Enable/Disable Tx power
 */
void nrf24_txPower(uint8_t EN);

/*
 * Returns the payload length 
 */
uint8_t nrf24_payloadLength(void);

/*
 * Set RX Address 
 */
void nrf24_setRxAddr(uint8_t *addr);

/*
 * Set TX Address 
 */
void nrf24_setTxAddr(uint8_t *addr);

/*
 * Set MAC Address
 */
void nrf24_ble_setMacId(uint8_t mac[]);

/*
 * configuration of the radio 
 */
void nrf24_config(uint8_t channel, uint8_t len);

/*
 * configure the spi bus to be used 
 */
void nrf24_configSpi(void);
//void nrf24_configSpi(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs); //no need as we are not using software SPI

/*
 * Radio presence Self-Test 
 */
uint8_t nrf24_available(void);

#endif /* NRF24L01P_HAL_H_ */
