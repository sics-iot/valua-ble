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


#ifndef NRF24L01P_HAL_C_
#define NRF24L01P_HAL_C_

/**********************************************************************************************
 * INCLUDES
 *********************************************************************************************/
#include "nRF24L01.h"
#include "dev/nRF24L01P_HAL.h"

/**********************************************************************************************
 * CONSTANTS
 *********************************************************************************************/
#define RX_PWRUP_STAT ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT))
#define TX_PWRUP_STAT ((1 << RX_DR) | (0 << TX_DS) | (1 << MAX_RT))
#define RX_PWRUP_CONF ((1 << PWR_UP) | (1 << PIRM_RX))
#define TX_PWRUP_CONF ((1 << PWR_UP) | (0 << PRIM_RX))
#define PWRDWN ((0 << PWR_UP) | (0 << PRIM_RX))
#define NRF_RST_CONF ((0 << MASK_RX_DR) | (0 << MASK_TX_DR) | (0 << MASK_MAX_RT) | (0 << CRCO) | (0 << PWR_UP) | (0 << PRIM_RX) | (0 << EN_CRC))
/**********************************************************************************************
 * Global VARIABLES
 *********************************************************************************************/
extern uint8_t MAC[6];

/**********************************************************************************************
 * @fn          nrf24_init          
 * 
 * @brief       Initialize the nRF24l01+ module  
 * 
 * @param       None 
 *  
 * @return      None
 *********************************************************************************************/
void nrf24_init(void){

}

/**********************************************************************************************
 * @fn          nrf24_getStatus
 * 
 * @brief       Get the sataus of the radio 
 * 
 * @param       None
 *  
 * @return      returns the Status of radio in HEX
 *********************************************************************************************/
uint8_t nrf24_getStatus(void){
	uint8_t status;
	status = csi00_strobe(NOP);
	return status;	
}

/**********************************************************************************************
 * @fn          nrf24_dataReady
 * 
 * @brief       Checks if data is available in Rx
 * 
 * @param       None
 *  
 * @return      1 : Data Ready 
 *              0 : Data not Ready
 *********************************************************************************************/
uint8_t nrf24_dataReady(void){
	uint8_t status;
	status = nrf24_getStatus();
	if(status & (1 << RX_DR )) return 1;
	else return 0;
}

/**********************************************************************************************
 * @fn          nrf24_isSending
 * 
 * @brief       Check if Radio is sending Data, Data available in Tx 
 * 
 * @param       None 
 *  
 * @return      1 : Sending Data 
 *              0 : Radio is free 
 *********************************************************************************************/
uint8_t nrf24_isSending(void){
	uint8_t status;
	status = nrf24_getStatus();
	if (status & ((1 << TX_DS) | (1 << MAX_RT))) return 1;
	else return 0;
}

/**********************************************************************************************
 * @fn          nrf24_rxFifoEmpty 
 * 
 * @brief       check weather the Rx FIFO is empty or not 
 * 
 * @param       None
 *  
 * @return      1 : Rx FIFO is Empty
 *              0 : Rx FIFO is not Empty
 *********************************************************************************************/
uint8_t nrf24_rxFifoEmpty(void){
	uint8_t fifoStat;
	fifoStat = csi00_read(FIFO_STATUS);
	return (fifoStat & (1 << RX_EMPTY));
}

/**********************************************************************************************
 * @fn          nrf24_getData
 * 
 * @brief       Reads the payload data into an buffer : core data I/O fn
 * 
 * @param       *payoad_data, payload_length
 *  
 * @return      0 : Sucess
 *              1 : Faliure      
 *********************************************************************************************/
uint8_t nrf24_getData(void *data, uint16_t len){

}

/**********************************************************************************************
 * @fn          nrf24_send
 * 
 * @brief       Writes the data to the Tx buffer : core data I/O fn
 * 
 * @param       *payload_data, payload_length
 *  
 * @return      0 : Sucess
 *              1 : Faliure
 *********************************************************************************************/
uint8_t nrf24_send(void *data, uint16_t len){

}

/**********************************************************************************************
 * @fn          nrf24_rxPower
 * 
 * @brief       Enable or Disable Rx power 
 * 
 * @param       1 : Enable 
 *              0 : Disable 
 *  
 * @return      None
 *********************************************************************************************/
void nrf24_rxPower(uint8_t EN){
	if (EN == 1){
		csi00_write(STATUS, RX_PWRUP_STAT);
		csi00_write(CONFIG, RX_PWRUP_CONF);
	}
	else if (EN == 0){
		csi00_write(CONFIG, PWRDWN);
	}
}

/**********************************************************************************************
 * @fn          nrf24_txPower
 * 
 * @brief       Enable or Disable Tx power
 * 
 * @param       1 : Enable
 *              0 : Disable
 *  
 * @return      None 
 *********************************************************************************************/
void nrf24_txPower(uint8_t EN){
	if (EN == 1){
		csi00_write(STATUS, TX_PWRUP_STAT);
		csi00_write(CONFIG, TX_PWRUP_CONF);
	}
	else if (EN == 0){
		csi00_write(CONFIG, PWRDWN);
	}
}

/**********************************************************************************************
 * @fn          nrf24_payloadLength 
 * 
 * @brief       Returns length of teh payload
 * 
 * @param       None
 *  
 * @return      payload_length
 *********************************************************************************************/
uint8_t nrf24_payloadLength(void){

}

/**********************************************************************************************
 * @fn          nrf24_setRxAddr
 * 
 * @brief       Set Reciever's Address 
 * 
 * @param       *addr
 *  
 * @return      None
 *********************************************************************************************/
void nrf24_setRxAddr(uint8_t *addr){

}

/**********************************************************************************************
 * @fn          nrf24_setTxAddr
 * 
 * @brief       Set Transmitter's Address 
 * 
 * @param       *addr
 *  
 * @return      None
 *********************************************************************************************/
void nrf24_setTxAddr(uint8_t *addr){

}

/**********************************************************************************************
 * @fn          nrf24_ble_setMacId
 * 
 * @brief       Set BLE MAC ID to global MAC variable
 * 
 * @param       uint8_t mac[6]
 *  
 * @return      None 
 *********************************************************************************************/
void nrf24_ble_setMacId(uint8_t mac[]){
	uint8_t ctr;
	for(ctr = 0 ; ctr < 5 ; ctr++){
		MAC[ctr] = mac[ctr];
	}
}

/**********************************************************************************************
 * @fn          nrf24_config 
 * 
 * @brief       configure channels to be used and the payload_length
 * 
 * @param       Radio channels to be used ( -*- channel info ), payload_length  
 *  
 * @return      None
 *********************************************************************************************/
void nrf24_config(uint8_t channel, uint8_t len){

}

/**********************************************************************************************
 * @fn          nrf24_configSpi
 * 
 * @brief       configure the CS and CSN SPI pins for communication (use only if 3wire mode is configured in rl78/spi driver)
 * 
 * @param       spi_cs pin, spi_csn pin
 *  
 * @return      None
 *********************************************************************************************/
void nrf24_configSpi(uint8_t spi_cs, uint8_t spi_csn){

}

/**********************************************************************************************
 * @fn          nrf24_available
 * 
 * @brief       Self-Test for Radio presence detection 
 * 
 * @param       None
 *  
 * @return      0 : Radio present
 *              1 : Radio not Present 
 *********************************************************************************************/
uint8_t nrf24_available(void){

}

// -*- blank tempelate to be used for new function def
/**********************************************************************************************
 * @fn          
 * 
 * @brief       
 * 
 * @param       
 *  
 * @return      
 *********************************************************************************************/

#endif /* NRF24L01P_HAL_C_ */
