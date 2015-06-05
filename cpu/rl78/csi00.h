/*
 * Copyright (c) 2015, SICS Swedish ICT.
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

#ifndef CSI00_H__
#define CSI00_H__

#include "stdint.h" // for uint8_t, uint16_6

uint8_t *csi00_tx_addr;
uint8_t *csi00_rx_addr;
uint16_t csi00_buf_len;

/* extern int (*csi00_input_handler)(unsigned char c); */

/*
 *  Initialization of the SPI Module
 */
void csi00_init(void);
/* void csi00_set_input(int (*input)(unsigned char c)); */

/*
 *  Send one byte data over SPI bus
 */
uint8_t csi00_strobe(uint8_t cmd);

/*
 *  Read one byte of data from SPI bus
 */
uint8_t csi00_read(uint8_t addr);

/*
 * Write a byte of data on SPI bus
 */
void csi00_write(uint8_t addr, uint8_t val);

/*
 * Transfer function for reading and Writing multiple bytes of data --Modification needed--
 */
uint8_t csi00_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t data_len);

/*
 * Write multiple bytes of data on SPI bus
 */
void csi00_write_message(uint8_t addr, uint8_t *buf, uint8_t len);

/*
 * Read multiple bytes of data from SPI bus
 */
int csi00_read_message(uint8_t addr, uint8_t *buf, uint8_t len);

#endif /* CSI00_H__ */
