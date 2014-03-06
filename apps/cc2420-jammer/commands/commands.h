#ifndef COMMANDS
#define COMMANDS

#define DEFAULT_TX_INTERVAL (CLOCK_SECOND / 16)
#define DEFAULT_MAX_TX_PACKETS 100
#define DEFAULT_PAYLOAD_LEN 20
/* exprimental value used to occupy near full bandwidth, assuming RIMTER_SECOND = 4096 * N */
/* "SFD gap" = 298 us out of 4395 us Droplet interval => actual free air time = 298 - 160 (preamble+SFD) = 138 us => free bandwidth = 138/4395 = 3.1% */
/* #define DEFAULT_RTIMER_INTERVAL (RTIMER_SECOND / 128) */
#define DEFAULT_RTIMER_INTERVAL ((RTIMER_SECOND + (225/2)) / 225 )

extern int mode;
extern clock_time_t tx_interval;
extern int max_tx_packets;
extern int payload_len;
extern uint16_t seqno;
extern struct etimer et;
extern struct rtimer rt;
extern rtimer_clock_t rtimer_interval;
extern long int sum_rssi;
extern long unsigned sum_lqi;
extern uint8_t len_hdr;

void
do_command(char ch);

struct command
{
	char ch;
	void (*f)(void);
};

void commands_set_callback(void (*f)(int));

/* void next_mode(void); */
/* void previous_mode(void); */
/* void next_mode(void); */
/* void previous_mode(void); */
/* void reboot(void); */
/* void power_up(void); */
/* void power_down(void); */
/* void tx_frequency_up(void); */
/* void tx_frequency_down(void); */
/* void rtimer_frequency_up(void); */
/* void rtimer_frequency_down(void); */
/* void rssi(void); */
/* void channel_up(void); */
/* void channel_down(void); */
/* void frequency_up(void); */
/* void frequency_down(void); */
/* void cca_mux_up(void); */
/* void sfd_mux_up(void); */
/* void view_rx_statistics(void); */
/* void view_failed_rx_statistics(void); */
/* void view_tx_power_level(void); */
/* void tx_packets_up(void); */
/* void tx_packets_down(void); */
/* void this_mode_again(void); */
/* void payload_len_up(void); */
/* void payload_len_down(void); */
/* void agc_lnamix_gainmode_up(void); */
/* void agc_vga_gain_up(void); */
/* void debug_hssd(void); */
/* void debug_analog(void); */
/* void reverse_phase(void); */
/* void reverse_syncword(void); */
/* void len_hdr_up(void); */

#endif
