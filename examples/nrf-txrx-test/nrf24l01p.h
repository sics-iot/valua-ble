#ifndef NRF24L01P
#define NRF24L01P

/* nRF24L01+ commands */
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_TX_PAYLOAD_NO_ACK 0xB0

/* nRF24L01+ registers */
#define CONFIG 0x00
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define RX_ADDR_P0 0x0A
#define TX_ADDR 0x10
#define DYNPD 0x1C
#define FEATURE 0x1D

/* nRF24L01+ register bits */
#define BV(b) (1<<(b))
// register CONFIG
#define MASK_RX_DR BV(6)
#define MASK_TX_DS BV(5)
#define MASK_MAX_RT BV(4)
#define EN_CRC BV(3)
#define CRCO BV(2)
#define PWR_UP BV(1)
#define PRIM_RX BV(0)
// register STATUS
#define RX_DR BV(6)
#define TX_DS BV(5)
#define MAX_RT BV(4)
#define RX_P_NO BV(3)|BV(2)|BV(1)
#define TX_FULL BV(0)

extern struct radio_driver nrf24l01p_driver;

unsigned
getreg(unsigned addr);

void
setreg(unsigned addr, unsigned val);


#endif
