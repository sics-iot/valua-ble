# The nRF beacon hardware
The hardware consists of a Renesas RL78G14 MCU and an unpluggable Nordic Semiconductor nRF24L01+ radio module.
## MCU
RL78G14 is a family of 16-bit MCUs, which consists of many variants in terms of pin count (30...100) and ROM (16...512 KB) / RAM (2.5...48 KB) sizes. The nRF beacon rev 2.5 has a R5F104AE, a 30-pin RL78G14 chip that carries 64 KB ROM and 5.5 KB RAM. The CPU clock rate can be configured to maximum 32 MHz.
## Radio
The radio module carries a 2.4 GHz GFSK radio transceiver controlled from an SPI interface. The maximum bit rate is 2Mbps. The maximum TX power is 0dBm. The TX and RX FIFO's are 32 bytes long. 128 channel frequencies are selectable from the range [2400, 2400+127] MHz.
# Building Contiki for the nRF beacon
We use the GNU cross-compile toolchain supplied by KPIT:
[GNURL78 Tool Chain (ELF Format)](http://www.kpitgnutools.com/latestToolchain.php)

## Build LED blink example
	cd contiki/example/nrf-beacon-blink
	make
	make srec

The code can then be flashed to the beacon board using
[rl78flash](https://github.com/msalov/rl78flash),
but a [custom cable](https://github.com/msalov/rl78flash/blob/master/hw/rl78s-hw.png) must be made.
Obtain and build rl78flash:

	git clone https://github.com/msalov/rl78flash.git
	make -C rl78flash

## Flash the example
	rl78flash -vv -i -m3 /dev/ttyUSB0 -b500000 -a contiki/examples/nrf-beacon-blink/blink.nrf-beacon.srec

Connect a serial cable to the TxD2 pin, and open a terminal set to 115200 bps, 8-bits, no-parity to see the program's print output:

	Hi!
	Hello, blink
	node_id = 258
	CPU frequency = 16000000
	tick 0
	tick 1
	tick 2
	tick 3
	...
