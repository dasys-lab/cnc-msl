// FIRMWARE 12.09.2007/10:00

#include <intrins.h>
#include "init.h"
#include "ais_typedef.h"
#include "system/ais_utils.h"
#include "aisc167b/ais_gpt.h"
#include "aisc167b/ais_led.h"
#include "system/ais_cmdbuff.h"
#include "system/ais_select.h"
#include "motorctrl/ais_motorconfig.h"
#include "motorctrl/ais_configmanager.h"
#include "motorctrl/ais_motorctrl.h"
#include "controller/ais_controller.h"
#include "motorctrl/ais_output.h"
#include "mpwr/ais_mpwr.h" // for tRAP functions stoppingmotors
#include "comasc/ais_asccom.h"

#include "comasc/ais_ascbuff.h"

#include "aisc167b/ais_eeprom.h"
#include "can_stuff/ais_can.h"
#include "aisc167b/ais_bioport.h"
#include <stdio.h>
#include <stdlib.h>

int JumpDown2CAN(void);



sbit DIR_DOUT = DP2^15;
sbit DOUT   = P2^15;
sbit DIR_D2OUT = DP2^14;
sbit D2OUT   = P2^14;


void strobeMS() {
  unsigned char i=0;

  DOUT  = 1; 
  for (i=0; i<50; i++) ;

  DOUT = 0; // TRIGGER
  for (i=0; i<10; i++) ;
  DOUT = 1;
  for (i=0; i<20; i++) ;
}

void strobeME() {
  unsigned char i=0;

  DOUT  = 1; 
  for (i=0; i<50; i++) ;

  DOUT = 0; // TRIGGER
  for (i=0; i<200; i++) ;
  DOUT = 1;
  for (i=0; i<20; i++) ;
}




//////////////////////////////////////////////////////////////////////////////
/// VMC Motorcontrol, main program loop
///
///
//////////////////////////////////////////////////////////////////////////////
int main(void) {
    // Length of last cycle in [µs]
    VMC_LONG_32 last_cycle_length = 0;

    // Desired Cycle Time for main Loop in ms
    VMC_LONG_32 desired_cycletime;

	VMC_INT_16 defConfigID;

    //============================ Initialization ================================
	// call init funtion to initialize all modules
    init();

   DIR_DOUT  		= 1;
   DIR_D2OUT 		= 1;   
   DOUT  			= 1; 
   D2OUT 			= 1;


    IEN = 1;

	asccom_senddata(ASC_SEND_ALL); // Send all init messages
	// enable all Interrupts

	

	// Set desired cycletime, in ms
	desired_cycletime = 9;  //0;  //9; //20;

	// load start config, if empty default values will be set
	set_default_configuration();

    //set_default_configuration();
 	// load default configuration as preset with Jumper on AISC167Board
	defConfigID = (char)((2 * get_state_sw2()) + get_state_sw1());	
	load_configuration(defConfigID);	

	//============================ Initialization End =========================

	//set_timeout(0);
    //============================ Main Loop ====================================

//JumpDown2CAN();

    while(1) {	
int can_group;

// Time START/END

//	if (can_send( MSG_STDERR )) while(1){led_swap_red();};

	    strobeMS();
        // Gets Time (in µs) at Cycle Start
        last_cycle_length = get_cycle_time();	

/////////////////////////////////////////////////////////////////////////////////
        // Load command from RS232 and stores it into the commandbuffer
        cmdb_load_command(_CMDB_CH_ASC0_, _CMDB_ONE_);
//@@@@@        cmdb_load_command(_CMDB_CH_ALL_, _CMDB_ALL_);

/////////////////////////////////////////////////////////////////////////////////
		while ( can_group = can_receive())
	 		{		// transfere all received messages into buffer
	if (can_send( MSG_CMDGRP_MOTOR_CONFOUT_ )) while(1){led_swap_red();};
			}

/////////////////////////////////////////////////////////////////////////////////
// commands from fast interfaces
       
		// Load message from CAN and stores it into the commandbuffer

        cmdb_load_command(_CMDB_CH_CAN_, _CMDB_ONE_);

//@@@@@       

        // Sends response from commandbuffer to RS232



		while (!can_answer())
		{
	if (can_send( MSG_CMDGRP_MOTOR_CONFOUT_ )) while(1){led_swap_red();};
		}


        // Selects the function to handle the command if applicable
//@@@@@        sel_proceed_cmd();
		sel_proceed_allcmd();

        // The control loop, read sensors, calculate runtime data and set actuators
       main_motorcontrol(last_cycle_length);  // needs 3,5 msec!!! with ZERO commands
		

        // Sends response from commandbuffer to RS232
       cmdb_send_command(_CMDB_ONE_);
 //@@@@@      cmdb_send_command(_CMDB_ALL_);


		asccom_senddata(ASC_SEND_ONE); // Trigger ASC0 to send all Data ASC_SEND_ALL
		// CALL ONLY ONCE per loop!!!!!! 
	
		// show led sequenze
		ledseq_show();


        // Wait to synchronize the cycletime with the desired one (HERE =>0)		
        sync_cycletime(desired_cycletime);			

//	if (can_send( MSG_CMDGRP_MOTOR_CONFOUT_ )) while(1){led_swap_red();};

	get_bumper_port();
	}

	//============================ Main Loop End====================================



}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// Hardware Trap - whenever Hardware class B error ocurs show steady red 
/// led and stop !
///
//////////////////////////////////////////////////////////////////////////////
void class_b_trap (void) interrupt 0x0A  {
  VMC_UCHAR_8 motorID; 
  VMC_UINT_16 count, loop; 

  led_set_green(0); // Green led off
  led_set_red(1);   // Red led on
  /* STOP MOTORS ! */
  for ( motorID = 0; motorID < _NUM_MOTORS_; motorID++)
   mpwr_motor_set(motorID, 0 );
  while (1) {
	for (count = 0; count < 5; count++ ) {
	  led_set_red(1);   // Red led on	
	  for (loop = 0; loop < 3000; loop++) ;
	  led_set_red(0);   // Red led on	
	  for (loop = 0; loop < 3000; loop++) ;
    }
    led_set_green(0); // Green led off
    led_set_red(1);   // Red led on
	for (loop = 0; loop < 10000; loop++) ;
  }        // end-less loop */
}



//////////////////////////////////////////////////////////////////////////////
/// Hardware Trap - whenever stack Error ocurs 
/// show steady red led and stop !
///
//////////////////////////////////////////////////////////////////////////////
void STKOF_trap (void) interrupt 0x04  {
  VMC_UCHAR_8 motorID; 
  VMC_UINT_16 count, loop; 

  led_set_green(0); // Green led off
  led_set_red(1);   // Red led on
  /* STOP MOTORS ! */
  for ( motorID = 0; motorID < _NUM_MOTORS_; motorID++)
   mpwr_motor_set(motorID, 0 );
  while (1) {
	for (count = 0; count < 4; count++ ) {
	  led_set_red(1);   // Red led on	
	  for (loop = 0; loop < 3000; loop++) ;
	  led_set_red(0);   // Red led on	
	  for (loop = 0; loop < 3000; loop++) ;
    }
    led_set_green(0); // Green led off
    led_set_red(1);   // Red led on
	for (loop = 0; loop < 10000; loop++) ;
  }        // end-less loop */
  
}


//////////////////////////////////////////////////////////////////////////////
/// Hardware Trap - whenever stack Error ocurs 
/// show steady red led and stop !
///
//////////////////////////////////////////////////////////////////////////////
void STKUF_trap (void) interrupt 0x06  {
  VMC_UCHAR_8 motorID; 
  VMC_UINT_16 count, loop; 

  led_set_green(0); // Green led off
  led_set_red(1);   // Red led on
  /* STOP MOTORS ! */
  for ( motorID = 0; motorID < _NUM_MOTORS_; motorID++)
   mpwr_motor_set(motorID, 0 );
  while (1) {
	for (count = 0; count < 6; count++ ) {
	  led_set_red(1);   // Red led on	
	  for (loop = 0; loop < 3000; loop++) ;
	  led_set_red(0);   // Red led on	
	  for (loop = 0; loop < 3000; loop++) ;
    }
    led_set_green(0); // Green led off
    led_set_red(1);   // Red led on
	for (loop = 0; loop < 10000; loop++) ;
  }        // end-less loop */
}



