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
 *           (Reference : http://dmitry.gr/index.php?r=05.Projects&proj=11.%20Bluetooth%20LE%20fakery by Dmitry)
 * Created : 2015
 * Updated : $Date: 2015/06/08 17:23:02 $
 *           $Revision: 1.0 $
 */


#ifndef NRF24L01P_BLE_H_
#define NRF24L01P_BLE_H_

/**********************************************************************************************
 * INCLUDES
 *********************************************************************************************/

/**********************************************************************************************
 * CONSTANTS AND STRUCT
 *********************************************************************************************/
extern uint8_t MAC[6];
struct adv_header{
	uint8_t header;
	uint8_t dataLength;
	uint8_t MAC[6];
};

/**********************************************************************************************
 * FUNCTIONS - API
 *********************************************************************************************/

/* 
 * Function for CRC computation
 */
void ble_CRC(const uint8_t* data, uint8_t len, uint8_t* dst);

/* 
 * Function for data Whitening
 */
void ble_Whiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff);

/* 
 * Function for encoding BLE Packet
 */
void ble_PacketEncode(uint8_t* data, uint8_t len, uint8_t chan);

/* 
 * Function or starting BLe Whitning  
 */
static inline uint8_t ble_WhitenStart(uint8_t chan);

/* 
 * Function for swapping the bits as BLE transmits in opposit order
 */
uint8_t swapbits(uint8_t input);


#endif /* NRF24L01P_BLE_H_ */

