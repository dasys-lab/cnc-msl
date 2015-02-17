/*****************************************************************************
*  "A Very Simple Application" from the uIP 0.9 documentation
*****************************************************************************/
#include <avr/io.h>
#include "app.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <avr/wdt.h>
#include "uip_arp.h"
#include <avr/eeprom.h>


// CAN
#include "mcp2515.h"
#include "global.h"
#include "defaults.h"

// UART
#include "uart.h"

#define BUFFERLENGTH 256

#define CNMC_QUOTE 0x84
#define CNMC_START 0x81
#define CNMC_END 0x82

bool uart0_quote = false;
int uart0_status = 0;
bool uart1_quote = false;
int uart1_status = 0;
char *bla;
char c;
u16_t temp;
struct uip_eth_addr eth_adr;
static char status[20];

tExtendedCAN *message;
tExtendedCAN m;
//static char recbuf[BUFFERLENGTH];
static char canbuf[20];

void poll_uart1(void);

void connection_init(void)
{
	int j,i;	
	u16_t ipaddr;
	char* init_buf = "init";

	
        uip_ipaddr(&ipaddr, 192,168,0,5);
	
	//open ports
        eth2can_status = uip_udp_listen(&ipaddr,10000);
	eth2can_uart0 = uip_udp_listen(&ipaddr,10001);
	eth2can_uart1 = uip_udp_listen(&ipaddr,10002);
	eth2can_can = uip_udp_listen(&ipaddr,10003);

	
	mcp2515_init();
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
	
	//send "init" on resart for arp reply
	uip_udp_sendto(init_buf, 4, eth2can_uart1);
	uip_arp_out();
	nic_send();

	send_init = 0;

}


void tcp_connection_app(void)
{
}

/*
 * Bearbeitet Pakete der 3 UDP Verbindungen
 */
void udp_connection_app(void) {
	char q;
	if(uip_newdata() || uip_rexmit())
	{
		// Daten der UART0 Verbindung einfach auf die UART Rausschreiben
		if(uip_udp_conn==eth2can_uart0) {
			bla = (char*)uip_appdata;
			uart_puts(bla);
		}
		// Daten der UART1 Verbindung einfach auf die UART Rausschreiben
		if(uip_udp_conn==eth2can_uart1) {
			bla = (char*)uip_appdata;
			q = 0;
			do {
				uart1_putc(*bla);
				if (*bla == CNMC_QUOTE && !q) q = 1;
				else q = 0;
				bla++;
			} while (!(*bla == CNMC_END && !q));
			uart1_putc(*bla);
//			uart1_puts(bla);
		} 
		// Can Daten in Messagestruktur Paken und auf Can Rausschreiben.
		else if(uip_udp_conn==eth2can_can) {
			message = (tExtendedCAN*)uip_appdata;
			mcp2515_send_extmessage(message);
		}
		// Status Protokoll
		else if(uip_udp_conn==eth2can_status) {
			switch(uip_appdata[0]) { 
				case 'P':
					uip_appdata[0]++;
					uip_udp_sendto(uip_appdata, 1, eth2can_status);
					uip_arp_out();
					nic_send();
					return;
				case 'C':
					status[0] = mcp2515_read_register(EFLG);
					status[1] = mcp2515_read_register(TEC);
					status[2] = mcp2515_read_register(REC);
					status[3] = 0x00;
					uip_udp_sendto(status, 4, eth2can_status);
					uip_arp_out();
					nic_send();
					return;
				case 'A':
					uip_appdata[0]++;
					uip_arp_init();
					uip_udp_sendto(uip_appdata, 1, eth2can_status);
					uip_arp_out();
					nic_send();
					return;
				case 'S':
					goto *((void**) 0);
					return;
				case 'R':
					cli();
					wdt_enable (WDTO_15MS);
					while (1);
					return;
				case 'I':
					mcp2515_init();
					mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
					break;
				case 'M':
					for(c=0;c<6;c++)
					{
						status[c] = eeprom_read_byte(AT_EEPROM_MAC_ADDRESS+c);
					}
					status[6]=0x00;
					uip_udp_sendto(status, 6, eth2can_status);
					uip_arp_out();
					nic_send();
					break;
				case 'E':
					for(c=0;c<6;c++) eeprom_write_byte(AT_EEPROM_MAC_ADDRESS+c, uip_appdata[c+1]);
					break;
				case 'X':
					send_init = 0;
					break;
			}
		}
	}
}


void poll_can(void) {
	if(mcp2515_check_message()) {
		mcp2515_get_extmessage(&m);
		uip_udp_sendto((char*)&m, 15, eth2can_can);
                uip_arp_out();
                nic_send();
	}
}

void poll_uart0(void) {
	static char recbuf[BUFFERLENGTH];
	unsigned int c;
	static uint8_t i = 0;
	static uint8_t len = 0;
	static char overflow = 0;

	c = uart_getc();

	if (c & UART_NO_DATA) {
		return;
	}

	// check for error
	if (c & UART_FRAME_ERROR) {
		return;
	}
	if (c & UART_OVERRUN_ERROR) {
		return;
	}
	if (c & UART_BUFFER_OVERFLOW) {
		return;
	}

	// echo back the received char

	// avoid inserting NULL-Byte in strings
	if (c == 0x00)
		return;
// \n\r -> leerpaket!
	if (c == '\n' || c == '\r') {
		if (overflow) {
			overflow = 0;
			len = 0;
			return;
		}
		if (len == 0)
			return;
		// add trailing NULL-byte
		recbuf[len] = 0x00;
		uip_udp_sendto(recbuf, len, eth2can_uart0);
		uip_arp_out();
		nic_send();
		len = 0;
	}
	else {
		if (len >= BUFFERLENGTH-1) {
			overflow = 1;
			return;
		}
		else {
			recbuf[len++] = c;
		}
	}

}


void poll_uart1(void) {
	static char recbuf[BUFFERLENGTH];
	unsigned int c;
	static uint8_t i = 0;
	static uint8_t len = 0;
	static char overflow = 0;

	c = uart1_getc();

	if (c & UART_NO_DATA) {
		return;
	}
	// check for error
	if (c & UART_FRAME_ERROR) {
		return;
	}
	if (c & UART_OVERRUN_ERROR) {
		return;
	}
	if (c & UART_BUFFER_OVERFLOW) {
		return;
	}

//VNCVNCVNC	

	if (uart1_status == 0) { // auf daten warten
		if (c == CNMC_START && !uart1_quote) {
			uart1_status = 1;
			len = 0;
			recbuf[len++] = c;
		}
	} else {
		recbuf[len++] = c;
		if (c == CNMC_END && !uart1_quote) {
			recbuf[len] = 0x00;
			uip_udp_sendto(recbuf, len, eth2can_uart1);
			uip_arp_out();
			nic_send();
			len = 0;
			uart1_status = 0;
		}
	 

	if (uart1_quote) uart1_quote = false;
	else if (c == CNMC_QUOTE) uart1_quote = true;

	}
//	if (len > 0)  {
//		uip_udp_sendto(recbuf, len, eth2can_uart1);
//		uip_arp_out();
//		nic_send();
//	}

	// echo back the received char

	// avoid inserting NULL-Byte in strings
//	if (c == 0x00)
//		return;

// hier \n bzw \r rausnehmen daimt keine leerpakete kommen
//	if (c == '\n' || c == '\r') {
//		if (overflow) {
//			overflow = 0;
//			len = 0;
//			return;
//		}
//		if (len == 0)
//			return;
//		// add trailing NULL-byte
//		recbuf[len] = 0x00;
//		uip_udp_sendto(recbuf, len, eth2can_uart1);
//		uip_arp_out();
//		nic_send();
//		len = 0;
//	}
//	else {
//		if (len >= BUFFERLENGTH-1) {
//			overflow = 1;
//			return;
//		}
//		else {
//			recbuf[len++] = c;
//		}
//	}

}

