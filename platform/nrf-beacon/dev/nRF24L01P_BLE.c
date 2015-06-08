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


#ifndef NRF24L01P_BLE_C_
#define NRF24L01P_BLE_C_

/**********************************************************************************************
 * INCLUDES
 *********************************************************************************************/
#include "nRF24L01P_BLE.h"
 
/**********************************************************************************************
 * Global VARIABLES 
 *********************************************************************************************/
extern uint8_t MAC[6];

/**********************************************************************************************
 * @fn          ble_CRC
 * 
 * @brief       function to compute 24-Bit CRC for the payload starting value of crc is 0x555555 
 *              CRC is 3bytes 
 * @param       data*(payload data), len(data length), dst*(destination) 
 *  
 * @return      None
 *********************************************************************************************/
void ble_CRC(const uint8_t* data, uint8_t len, uint8_t* dst){
uint8_t v, t, d;

	while(len--){
	
		d = *data++;
		for(v = 0; v < 8; v++, d >>= 1){
		
			t = dst[0] >> 7;
			
			dst[0] <<= 1;
			if(dst[1] & 0x80) dst[0] |= 1;
			dst[1] <<= 1;
			if(dst[2] & 0x80) dst[1] |= 1;
			dst[2] <<= 1;
			
		
			if(t != (d & 1)){
			
				dst[2] ^= 0x5B;
				dst[1] ^= 0x06;
			}
		}	
	}
}

/**********************************************************************************************
 * @fn          ble_Whiten
 * 
 * @brief       
 * 
 * @param       
 *  
 * @return      
 *********************************************************************************************/
void ble_Whiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff){
uint8_t  m;
	
	while(len--){
	
		for(m = 1; m; m <<= 1){
		
			if(whitenCoeff & 0x80){
				
				whitenCoeff ^= 0x11;
				(*data) ^= m;
			}
			whitenCoeff <<= 1;
		}
		data++;
	}
}

/**********************************************************************************************
 * @fn          ble_PacketEncode
 * 
 * @brief       
 * 
 * @param       
 *  
 * @return      
 *********************************************************************************************/
void ble_PacketEncode(uint8_t* data, uint8_t len, uint8_t chan){
	uint8_t i, dataLen = len - 3;
	
	btLeCrc(packet, dataLen, packet + dataLen);
	for(i = 0; i < 3; i++, dataLen++) packet[dataLen] = swapbits(packet[dataLen]);
	btLeWhiten(packet, len, btLeWhitenStart(chan));
	for(i = 0; i < len; i++) packet[i] = swapbits(packet[i]);
}

/**********************************************************************************************
 * @fn          ble_WhitenStart
 * 
 * @brief       
 * 
 * @param       
 *  
 * @return      
 *********************************************************************************************/
static inline uint8_t ble_WhitenStart(uint8_t chan){
	return swapbits(chan) | 2;
}

/**********************************************************************************************
 * @fn          ble_swapbits
 * 
 * @brief       
 * 
 * @param       
 *  
 * @return      
 *********************************************************************************************/
uint8_t ble_swapbits(uint8_t input){
uint8_t v = 0;
	
	if(input & 0x80) v |= 0x01;
	if(input & 0x40) v |= 0x02;
	if(input & 0x20) v |= 0x04;
	if(input & 0x10) v |= 0x08;
	if(input & 0x08) v |= 0x10;
	if(input & 0x04) v |= 0x20;
	if(input & 0x02) v |= 0x40;
	if(input & 0x01) v |= 0x80;

	return v;
}


endif /* NRF24L01P_BLE_C_ */

