


#include "system/ais_utils.h"
#include "aisc167b/ais_led.h"
#include "can_stuff/ais_can.h"

static char flag = 1;

int JumpDown2CAN(void)
{
#define DELAY 150000

long delay;
//if (can_init(CAN_BAUD, 0)) while(1){led_swap_red();}

// also use >> can_debug(ResponseCmd.datalen,2);
 
while(flag){

// Fill data
//  cansend_data[0] = 0x11;


	if (can_send( MSG_CMDGRP_MOTOR_CONFOUT_ )) while(1){led_swap_red();};
	for(delay = 0; delay<DELAY; delay++);

	if (can_send( MSG_CMDGRP_MOTOR_STATUSOUT_ )) while(1){led_swap_red();};
	for(delay = 0; delay<DELAY; delay++);

	if (can_send( MSG_CMDGRP_CONTROLLER_PAROUT_ )) while(1){led_swap_red();};
	for(delay = 0; delay<DELAY; delay++);

	if (can_send( MSG__CMDGRP_MOTOR_ERR_ )) while(1){led_swap_red();};
	for(delay = 0; delay<DELAY; delay++);

	if (can_send( MSG_CMDGRP_SYSTEM_BASIC_OUT_ )) while(1){led_swap_red();};
	for(delay = 0; delay<DELAY; delay++);

	if (can_send( MSG_CMDGRP_SYSTEM_MAINTANANCE_OUT_ )) while(1){led_swap_red();};
	for(delay = 0; delay<DELAY; delay++);

	if (can_send( MSG_STDERR )) while(1){led_swap_red();};
	for(delay = 0; delay<DELAY; delay++);

//	while(1){led_swap_red();}

flag = 0;
while(1){  if ( can_receive())	while(1){led_swap_red();};
			for(delay = 0; delay<100000; delay++);
			led_swap_green();
			for(delay = 0; delay<DELAY; delay++);
		}
	
	}
	return (0);
}

