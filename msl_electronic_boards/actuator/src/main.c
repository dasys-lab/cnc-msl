#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "util.h"
//#include <stdio.h>
#include "pwm.h"
#include "motor.h"
#include "messages.h"
#include "timer.h"
#include "defaults.h"
#include <util/delay.h>
#include <avr/wdt.h>
#include "lightbarrier.h"
#include "vision.h"
#include "bundle.h"
#include "button.h"
#include "motion.h"
#include "adns3080.h"

int main(void)
{

	SET_OUTPUT(SS);

	request_position = 0;
	request_dir = 1;
	
	//uart1_init(UART_BAUD_SELECT(9600, F_CPU));
        // save reset-register and reset the value
        uint8_t reset_msg[1] = {MCUCSR};
        MCUCSR = 0x00;
	
	sei();

	timer_init();
	pwm_init();
	//adc_init();
	motor_init();
	
	//init kicker pos
	_delay_us(10);
	servo_set(50);

	lightbarrier_init();
	vision_init();
	bundle_init();
	button_init();
	motion_init();
	can_init();
	adns_init();
	
	// send reset message
        can_put_cmd(CMD_RESET, reset_msg, 1);
	
	
	//uart1_puts("init Multi");
	
	//enable watchdog
	//wdt_enable(WDTO_15MS);
 	wdt_enable(WDTO_30MS);

	for (;;) {
		//uart1_puts("init Multi");
		timer_trigger_callbacks();
		message_handler();
		
		uint32_t time = timer_get_ms();
	
		if( debugOF == 0 )
		{	
			check_motors(time);
			ballhandler_control();		
			//stop_control();
		
			//_delay_us(1000000);
			//debug("Init Multi");
		
			//test message for mcp reset
			//debug("alive");
			sendStatus(time);

			visionSendStatus(time);
			
			bundleSendStatus(time);
		
			//update motion burst and sended
			update_motion_burst(time);
			send_motion_burst(time);
		}
		else
		{
			getFrameBurst(img);
			wdt_reset();
			printFrame(img);	
			wdt_reset();
			_delay_ms(10);
		}
		//reset watchdog
		wdt_reset();
	}
	
	return 0;
}
