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

#include "compiler.h"

#include "temperatur.h"

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


#define T0OVRFLW 0
#define MainInt 1

#ifdef use_RTC

#define TIMERCOUNTER_PERIODIC_TIMEOUT 1

#else	

#define TIMER_PRESCALE    1024
#define TIMERCOUNTER_PERIODIC_TIMEOUT (F_OSC / TIMER_PRESCALE / 2 / 256)

#endif

// tIn könnte man Bitmaskiert nutzen um versch.
// Interrupts zu steuern.

static char tInt=0,minInt=0;
static char hsek=0;

static unsigned char timerCounter;
volatile unsigned int iSec, iMin, iHour, iDays;

void initTimer(void)
{

#ifdef use_RTC

	ASSR = (1<<AS0);  		// Timer 0 Asynchron (ext. 32.768KHz Quarz)
	TCCR0 = (1<<CS02);		// fOsc/64 = 500ms

#else

	TCCR0 = 0x07;  // timer0 prescale 1/1024 (7)

#endif

	TIMSK = (1<<TOIE0);		// Overflow Int enable
	
	timerCounter = 0;
	tInt = 0;
}



#ifdef __IMAGECRAFT__
#pragma interrupt_handler SIG_OVERFLOW0:iv_TIMER0_OVF
#endif

SIGNAL(SIG_OVERFLOW0)
{

	timerCounter++;
	
	tInt = 1;  // interrupt flag
	hsek += 1; // hsek = half second
	
	// Kontroll Led :-)
	PORTB ^= (1<<PB5);
	
	if(hsek==2){ // volle Sekunde?
		
		iSec += 1;
		if(iSec==60){
			iSec = 0;
			iMin += 1;
			minInt = 1; // Minuten Interrupt :-)
			if(iMin==60){
				iMin = 0;
				iHour += 1;
				if(iHour==24){
					iHour = 0;
					iDays += 1;
				}
			}
		}
	
	hsek=0;
	
	}
	
}



/*****************************************************************************
*  Main Control Loop
*
*  
*****************************************************************************/
int main(void)
{
  unsigned char i;
  unsigned char arptimer=0;

	// PORTB PB5 als Ausgang (in use LED)
	DDRB=(1<<PB5);
	PORTB=(1<<PB5);

	init_sensors();

  // init NIC device driver
  nic_init();

  // init uIP
  uip_init();

  // init app
  example1_init();

  // init ARP cache
  uip_arp_init();

  // init periodic timer
  initTimer();
  
  sei();

	// initialisierendes lesen der Temperatur(en)
	read_temp_sensors();

  while(1)
  {

	if(minInt==1){
		minInt=0;
		read_temp_sensors();
	}
    // look for a packet
    uip_len = nic_poll();
    if(uip_len == 0)
    {
      // if timed out, call periodic function for each connection
      //if(timerCounter > TIMERCOUNTER_PERIODIC_TIMEOUT)
	  if(tInt)
      {
        
		tInt = 0;
		//timerCounter = 0;
        
        for(i = 0; i < UIP_CONNS; i++)
        {
          uip_periodic(i);
		
          // transmit a packet, if one is ready
          if(uip_len > 0)
          {
            uip_arp_out();
            nic_send();
          }
        }

        /* Call the ARP timer function every 10 seconds. */
        if(++arptimer == 20)
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
