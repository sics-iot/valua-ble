/*
 * Copyright (c) 2015, SICS Swedish ICT
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \author Zhitao He <zhitao@sics.se>, Gurinder Singh Gill <gsgill112@gmail.com>
 */

#include "stdio.h"
#include "csi00.h"
#include "string.h" // for NULL
#include "platform-conf.h"     /* for f_CLK */
#include "sfrs.h"
#include "sfrs-ext.h"
#include "lib/ringbuf.h"

#pragma GCC optimize ("Os")

#define NBITS2(n)  ((n&2)?1:0)
#define NBITS4(n)  ((n&(0xC))?(2+NBITS2(n>>2)):(NBITS2(n)))
#define NBITS8(n)  ((n&0xF0)?(4+NBITS4(n>>4)):(NBITS4(n)))
#define NBITS16(n) ((n&0xFF00)?(8+NBITS8(n>>8)):(NBITS8(n)))
#define NBITS32(n) ((n&0xFFFF0000)?(16+NBITS16(n>>16)):(NBITS16(n)))
#define NBITS(n)   (n==0?0:NBITS32(n)+1)
#define LOG2(n)    (NBITS(n) - 1)

/* Maximum SPI baudrate is f_CLK/2 */
#define BAUDRATE (f_CLK/2)
#define f_MCK f_CLK
#define PRS_VALUE LOG2((f_CLK / f_MCK))
#define SDR_VALUE (f_MCK / 2 / BAUDRATE - 1)

extern uint8_t *csi00_tx_addr;
extern uint8_t *csi00_rx_addr;
extern uint16_t csi00_buf_len;

/* #define TXBUFSIZE 128 // size must be power of two */

/* int (*csi00_input_handler)(unsigned char c); */

/* static struct ringbuf txbuf; */
/* static uint8_t txbuf_data[TXBUFSIZE]; */

/* #define READ 0 */
/* #define WRITE 1 */
/* static volatile int transmitting; */
/* static volatile int rw; */

void
csi00_init(void)
{
  /* Refer to hardware manual chapter 17 for details */
	/* See Figure 17 - 45 Initial Setting Procedure for Master Transmission/Reception */

	/* Supply clock to serial array unit 0, where CSI00 is located */
  SAU0EN = 1;
	/* Set the operation clock */
  SPS0 = (PRS_VALUE << 4) | PRS_VALUE;                  /* Set input clock (CK00 and CK01) to f_MCK */
  /* Set the operation mode = CSI, interrupt source = transfer end  */
  SMR00 = 0x0020;
  /* Set a communication format: Tx+Rx, data & clock phase type 4, no error interrupt,
     MSB first, data length is 8-bit 
     Refer to nRF24L10P datasheet, 8.3 "SPI operation"
   */
  SCR00 = 0xF007;
  /* Set a transfer baud rate, by dividing f_MCK */
  SDR00 = SDR_VALUE << 9;
  /* Set the initial output level of SCK = 0 and SO = 0 */
  SO0 &= ~0x0101;
  /* Set the SOEmn bit to 1 and enable data output */
  SOE0 |= 0x0001;
  /* Set SCK pin: output mode (and must set output latch high, see fig. 4-11) */
  P3 |= (1<<0);
  PM3 &= ~(1<<0);
  /* Set SO pin: output mode (and must set output latch high, see fig. 4-11) */
  P5 |= (1<<1);
  PM5 &= ~(1<<1);
  /* Set SI pin: input mode */
  PM5 |= (1<<0);

  /* Enable operation by setting SSmn = 1 */
  SS0 |= 0x0001;

  /* Unmask CSI00 interrupt */
  /* CSIIF00 = 0; */
  /* CSIMK00 = 0; */
}

uint8_t
csi00_strobe(uint8_t cmd)
{
	CSN = 0;
	SIO00 = cmd;
	/* // busy wait until communication ends */
	/* while(TSF00); */
	// busy wait until port buffer cleared
	/* while(BFF00); */
	// busy wait until port buffer filled with received value
	/* while(!BFF00); */
	while(TSF00);
	clock_delay_usec(1);
	CSN = 1;
	return SIO00;
}

uint8_t
csi00_read(uint8_t addr)
{
	CSN = 0;

	/* write addr byte */
	SIO00 = addr;
	while(TSF00);

	/* write dummy byte to read data byte */
	SIO00 = 0xFE;
	while(TSF00);

  	CSN = 1;
	return SIO00;
}

void
csi00_write(uint8_t addr, uint8_t val)
{
	CSN = 0;

	/* write addr byte */
	SIO00 = addr;
	while(TSF00);

	/* write data byte */
	SIO00 = val;
	while(TSF00);

  	CSN = 1;
}


int
csi00_read_message(uint8_t addr, uint8_t *buf, uint8_t len)
{
	int i;

	CSN = 0;

	/* write addr byte */
	SIO00 = addr;
	while(TSF00);

	for(i = 0;i < len;i++) {
		/* write dummy byte to read data byte */
		SIO00 = 0xFD;
		while(TSF00);
		clock_delay_usec(1);
		/* read byte into buffer */
		buf[i] = SIO00;
	}

	CSN = 1;
	return i;
}

void
csi00_write_message(uint8_t addr, uint8_t *buf, uint8_t len)
{
	int i;

	CSN = 0;

	/* write addr byte */
	SIO00 = addr;
	while(TSF00);

	/* write data bytes */
	for(i = 0;i < len;i++) {
		SIO00 = buf[i];
		while(TSF00);
	}

	CSN = 1;
}

/* Function for writing commands to the command reg  */
/* returns 0 for sucess */
/*uint8_t csi00_cmd(uint8_t cmd){
	CSN = 0;

	SIO00 = cmd;
//	while(TSF00);
	while(BFF00);

	CSN = 1;
	return 0;
}
*/

/* Function to transfer (transmit or recieve) single or multiple bytes of data over spi */
/*   Return char
 *   0            :             SUCESS
 *   <HEX>        :             Reg OUT
 *   TODO - fix bugs in CSI00_tranfer_fn and add interrupt support to Tx function !
 */
uint8_t csi00_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t data_len){
	uint8_t dummy_data;

	csi00_tx_addr = tx_buf;
	csi00_rx_addr = rx_buf;
	csi00_buf_len = data_len;
	//debug stmt 
	#if 1
	iprintf("0x%20X ; ",*csi00_tx_addr);	
	#endif
	
	if(csi00_tx_addr != 0){
		SIO00 = *csi00_tx_addr;
		while(TSF00);
		csi00_tx_addr++;
	}else{
		SIO00 = 0xFF;
		while(TSF00);
	}
	csi00_buf_len--;                       //end of initial data Tx/Rx Routine

	while(csi00_buf_len != 0){
		//debug stmt 
		#if 1
		iprintf("0x%20X ; ",*csi00_tx_addr);	
		#endif
		if(csi00_buf_len > 0){
			if(csi00_rx_addr != 0){
				*csi00_rx_addr = SIO00;
				while(TSF00);
				csi00_rx_addr++;
			}else{
				dummy_data = SIO00;
				while(TSF00);
			} 				// read useful/dummy data 
			if(csi00_tx_addr !=0){
				SIO00 = *csi00_tx_addr;
				while(TSF00);
				csi00_tx_addr++;
			}else{
				SIO00 = 0xFF;
				while(TSF00);
			}
			csi00_buf_len--;
		}else if(csi00_buf_len == 0){
			if(csi00_rx_addr != 0){
				*csi00_rx_addr = SIO00;
				while(TSF00);
				csi00_rx_addr++;
			}else{
				dummy_data = SIO00;
				while(TSF00);
			}
		}
	}
	//debug stmt 
	#if 1
	iprintf("returned ;");	
	#endif
	return 0;
}


/* CSI00 ISR: TX/RX transfer complete */
void __attribute__ ((interrupt))
st0_handler(void)
{
	/* more data => send next byte */
  /* if(ringbuf_elements(&txbuf) == 0) { */
  /*   transmitting = 0; */
  /* } else { */
  /*   TXD2 = ringbuf_get(&txbuf); */
  /* } */

	/* if (rw == READ) { */
	/* 	/\* uint8_t byte = (uint8_t)(SDR00 & 0x00FF); *\/ */
	/* 	uint8_t byte = SIO00; */
	/* 	(void)csi00_input_handler(byte); */
	/* 	if (len--) { */
	/* 		SIO00 = 0xFF; // write dummy data to generate SCK clock pulses */
	/* 	} else { */
	/* 		CSN = 1; // terminate transaction */
	/* 	} */
	/* } else { */
	/* 	// write mode */
	/* 	if(ringbuf_elements(&txbuf) == 0) { */
	/* 		transmitting = 0; */
	/* 		CSN = 1; */
	/* 	} else { */
	/* 		SIO00 = ringbuf_get(&txbuf); */
	/* 	} */
	/* } */
}
