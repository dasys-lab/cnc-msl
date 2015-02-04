/*****************************************************************************
*  Module Name:       uIP-AVR Port - main control loop shell
*  
*  Created By:        Louis Beaudoin (www.embedded-creations.com)
*
*  Original Release:  September 21, 2002 
*
*  Module Description:  
*  This main control loop shell provides everything required for a basic uIP
*  application using the RTL8019AS NIC
*
*   
*  November 16, 2003
*    Changed NIC interface from RTL8019 specific to general NIC calls
*    Calls the uip_arp_timer function every 10 seconds
*    
*  September 30, 2002
*    Added support for Imagecraft Compiler
*****************************************************************************/

#include "uip.h"
#include "nic.h"
#include "uip_arp.h"
#include <util/delay.h>
#include "compiler.h"
#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/signal.h>
#include<avr/io.h>

#include "uart.h"
#define UART_BAUD_RATE                  57600
#define UART_BAUD_RATE_KICKER           57600

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])



/*****************************************************************************
*  Periodic Timout Functions and variables
*
*  The periodic timeout rate can be changed depeding on your application
*  Modify these functions and variables based on your AVR device and clock
*    rate
*  The current setup will interrupt every 256 timer ticks when the timer
*    counter overflows.  timerCounter must count until 0.5 seconds have
*    alapsed
*****************************************************************************/

#define TIMER_PRESCALE    1024

#define TIMERCOUNTER_PERIODIC_TIMEOUT (F_CPU / TIMER_PRESCALE / 2 / 256)

static unsigned char timerCounter, blink;
static char do_can_poll;

void initTimer(void)
{
  TCCR0 = 0x07; // This is big shit. use bitshiftings to explain what there
            // is going on

  // interrupt on overflow
  TIMSK = (1 << TOIE0);
	
  timerCounter = 0;
  blink = 0;
}



#ifdef __IMAGECRAFT__
#pragma interrupt_handler SIG_OVERFLOW0:iv_TIMER0_OVF
#endif

SIGNAL(SIG_OVERFLOW0)
{
//  if(++blink%10==0) PORTF ^= (1 << PF3);
  timerCounter++;
}


SIGNAL (INT6_vect)
{ 
	++do_can_poll;
}


/*****************************************************************************
*  Main Control Loop
*
*  
*****************************************************************************/
int main(void)
{
  unsigned char arptimer=0;
  char* init_buf = "init";

  //for (int i=0;i<10;++i) delay_ms(50);
  
  // init NIC device driver
  nic_init();

  DDRF = (1<<PF3);
  PORTF |= (1 << PF3);

  // init uIP
  uip_init();

  // init ARP cache
  uip_arp_init();

   // init app
  connection_init();

  // init periodic timer
  initTimer();
  sei();

  //set can interupt
  do_can_poll = 5;
  EICRB |= (0<<ISC61) | (1<<ISC60);
  EIMSK |= (1<<INT6);

  sei();

  // init uart
  uart1_init(UART_BAUD_SELECT(UART_BAUD_RATE_KICKER, F_CPU));
  uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));

  while(1) {

	  //poll data from uart
	  poll_uart0();
	  poll_uart1();

	  //poll can
	  if (do_can_poll-- > 0) {
		  poll_can();
	  }

	  // look for a packet
	  uip_len = nic_poll();

	  if(uip_len == 0) {
		  if(send_init && timerCounter%10==0) {
			  //send "init" on resart
			  uip_udp_sendto(init_buf, 4, eth2can_status);
			  uip_arp_out();
			  nic_send();
		  }
		  
		  // if timed out, call periodic function for each connection
		  if(timerCounter > TIMERCOUNTER_PERIODIC_TIMEOUT) {
#if UIP_TCP
			  timerCounter = 0;

			  for(i = 0; i < UIP_CONNS; i++) {
				  uip_periodic(i);

				  // transmit a packet, if one is ready
				  if(uip_len > 0)  {
//					  uart1_puts_P(".");
					  uip_arp_out();
					  nic_send();
				  }
			  }
#endif
#if UIP_UDP_elm
//			  for(i = 0; i < UIP_UDP_CONNS; i++) {
//				  uip_udp_periodic(i);
				  /* If the above function invocation resulted in data that
				   *  should be sent out on the network, the global variable
				   *  uip_len is set to a value > 0. */
//				  if(uip_len > 0) {
//					  uip_arp_out();
//					  nic_send();
//				  }
//			  }
#endif /* UIP_UDP */

			  /* Call the ARP timer function every 10 seconds. */
			  if(++arptimer == 200)
			  {	
				  uip_arp_timer();
				  arptimer = 0;
			  }
		  }
	  }
	  else  // packet received
	  {
		  // process an IP packet
		  if(BUF->type == htons(UIP_ETHTYPE_IP))
		  {
			  // add the source to the ARP cache
			  // also correctly set the ethernet packet length before processing
			  uip_arp_ipin();
			  uip_input();

			  // transmit a packet, if one is ready
			  if(uip_len > 0)
			  {
				  uip_arp_out();
				  nic_send();
			  }
		  }
		  // process an ARP packet
		  else if(BUF->type == htons(UIP_ETHTYPE_ARP))
		  {
			  uip_arp_arpin();

			  // transmit a packet, if one is ready
			  if(uip_len > 0)
				  nic_send();
		  }
	  }
  }

  return 1;
}
