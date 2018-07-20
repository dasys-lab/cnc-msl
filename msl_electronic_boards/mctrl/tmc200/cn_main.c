#include "main.h"
#include "cn_main.h"
#include "cn_led.h"
#include "cn_motor.h"
#include "cn_time.h"
#include "cn_serial.h"
#include "cn_pwm.h"
#include "cn_commands.h"
#include "cn_controller.h"
#include "cn_adc.h"
#include "cn_logging.h"
#include "cn_force_lookup.h"

#include "cc6.h"
#include "gpt1.h"

#include <math.h>
#include <stdio.h>

void cn_main(void) {

	//int i;
	//udword tmp_start, tmp_end;
	//char  bla[7]= "abcdef\n";
	
	WDT_Trigger();

	ledOn();
	cn_time_init();
	cn_init_pwm();
	cn_adc_init();
	cn_serial_init();
	initMotorSystem();
	cn_controller_init();
	init_logging();
	
	//cn_init_force_lookup();
	ledOff();

	WDT_Trigger();

	while(1) {

		cn_log(LOG_EVENT_BEGIN_OF_CYCLE, 0, 0);
		
		cn_mark_cycle_start();

	
        motor_current[0] = getMotorCurrent(MOTOR1);
		motor_current[1] = getMotorCurrent(MOTOR2);
		motor_current[2] = getMotorCurrent(MOTOR3); 

		// commit/update pid anD other parameters
		cn_controller_update_params();

		// update encoder values
		// best to do directly before control
		encoder_rel[0] = getAndResetMotorTicks(MOTOR1);
		encoder_rel[1] = getAndResetMotorTicks(MOTOR2);
		encoder_rel[2] = getAndResetMotorTicks(MOTOR3);

		// control
		cn_controller_main_control();
		//cn_controller_current_control();
		cn_controller_set_final_pwm();
		
		// trigger adc conversions for next iteration
		cn_adc_trigger_conversions();

		// watchdog resets after ~200ms without trigger
		// but doesn't work at the moment
		// because of custom dave startup asm
		WDT_Trigger();


		//for(i = 0; i < 1000; ++i) {
		//	ASC0_TransmitData(0xFF);
		//	while(ASC0_IsTransmitDone() == 0) {k++;}
		//}
		//cn_serial_send_data(bla, (ubyte) 7);
  	
		//tmp_end = cn_getRTC();
		//printf("ct: %lu\n", (tmp_end - tmp_start));

		// wait for cycle to end
		cn_sync_to_cycle();

	}

}