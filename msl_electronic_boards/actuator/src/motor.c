#include <avr/interrupt.h>
#include <avr/io.h>
#include "pwm.h"
#include "defaults.h"
#include "global.h"
#include <util/delay.h>
#include "adc.h"
#include "motor.h"
#include "messages.h"

uint32_t last_heartbeat = 0; //ms
#define PING_TIMEOUT	1000 //ms

int16_t left_v = 0;
int8_t left_d = 0;
int16_t left_v_desired = 0;
int8_t left_d_desired = 0;

int16_t right_v = 0;
int8_t right_d = 0;
int16_t right_v_desired = 0;
int8_t right_d_desired = 0;

void motor_init(void){

	//set ballhandler left
	SET_OUTPUT(MOTOR_LEFT_DIR); //PC0 direction, PC1 reset
	SET_OUTPUT(MOTOR_LEFT_RESET);
	SET_INPUT(MOTOR_LEFT_ERROR_1);//PC2 PC3 error state
	SET_INPUT(MOTOR_LEFT_ERROR_2);
	RESET(MOTOR_LEFT_DIR);
	SET(MOTOR_LEFT_RESET);
	
	//set ballhandler right
	SET_OUTPUT(MOTOR_RIGHT_DIR); //PC0 direction, PC1 reset
	SET_OUTPUT(MOTOR_RIGHT_RESET);
	SET_INPUT(MOTOR_RIGHT_ERROR_1);//PC2 PC3 error state
	SET_INPUT(MOTOR_RIGHT_ERROR_2);
	RESET(MOTOR_RIGHT_DIR);
	SET(MOTOR_RIGHT_RESET);
	
	//set stop mechanism
	SET_OUTPUT(MOTOR_STOP_DIR); //PC0 direction, PC1 reset
	SET_OUTPUT(MOTOR_STOP_RESET);
	SET_INPUT(MOTOR_STOP_ERROR_1);//PC2 PC3 error state
	SET_INPUT(MOTOR_STOP_ERROR_2);
	RESET(MOTOR_STOP_DIR);
	SET(MOTOR_STOP_RESET);
}

void check_motors(uint32_t time) {

	//char str[10];
	//sprintf(str, "V:%d", time);
	//sprintf(str, "V:%d", last_heartbeat);
	//debug(str);

	if( (time - last_heartbeat > PING_TIMEOUT) && !manual_mode ) {	
		pwm_set(1, 0);
		pwm_set(2, 0);
		pwm_set(3, 0);
	}
	
}

void     ballhandler_set(int16_t left, int16_t right) {

	// PWM > 0 wheel moves left
	// PWM < 0 wheel moves right
	//DIR is SET -> wheel moves right
	//DIR is NOT SET -> wheel moves left

	// direction change 	
	if (left > 0 && IS_SET(MOTOR_LEFT_DIR) ) {
		//case left forward
		left_d_desired = 0;
	}

	if (right > 0 && IS_SET(MOTOR_RIGHT_DIR) ) {
		//case right forward
		right_d_desired = 0;		
	}

	if (left < 0 && !IS_SET(MOTOR_LEFT_DIR) ) {
		//case left backward
		left_d_desired = 1;
	}

	if (right < 0 && !IS_SET(MOTOR_RIGHT_DIR)) {
		//case left forward
		right_d_desired = 1;
	}

	//set pwms
	//26V for motors
	//pwm_set(1, abs(left*45/10));
	//pwm_set(2, abs(right*45/10));
	
	//max voltage 15.4V
	//pwm_set(1, abs(left)*26/10);
	//pwm_set(2, abs(right)*26/10);

	//use controller loop
	left_v_desired = abs(left)*26/10;
	right_v_desired =abs(right)*26/10;
}

void 	ballhandler_control() {
	
	//control left
	bool switchLeft = false;
	if( left_d != left_d_desired )
		switchLeft = true;

	if( switchLeft ) {
		left_v -= PWM_STEP_SIZE;
		if( left_v < 0) {
			left_v = 0;
			TOGGLE(MOTOR_LEFT_DIR);
			left_d = left_d_desired;
		}
	} else {
		int16_t diff = left_v_desired - left_v;
		if( diff > 0) {
			left_v += PWM_STEP_SIZE;
			if( left_v > left_v_desired )
				left_v = left_v_desired;

		} else {
			left_v -= PWM_STEP_SIZE;
			if( left_v < left_v_desired )
				left_v = left_v_desired;
		}
	}

	//control right
	bool switchRight = false;
	if( right_d != right_d_desired )
		switchRight = true;

	if( switchRight ) {
		right_v -= PWM_STEP_SIZE;
		if( right_v < 0) {
			right_v = 0;
			TOGGLE(MOTOR_RIGHT_DIR);
			right_d = right_d_desired;
		}
	} else {
		int16_t diff = right_v_desired - right_v;
		if( diff > 0) {
			right_v += PWM_STEP_SIZE;
			if( right_v > right_v_desired )
				right_v = right_v_desired;

		} else {
			right_v -= PWM_STEP_SIZE;
			if( right_v < right_v_desired )
				right_v = right_v_desired;
		}
	}

	//set pwms
	pwm_set(1,left_v);
	pwm_set(2,right_v);

	_delay_us(1000);
}

void     stop_set(uint8_t pos) {
	request_position = pos;
}

void 	stop_control() {
	//norm pos
	uint16_t normPos = request_position*1024l/100l;
	
	//regler fuer stopmechanismus
	uint16_t acPos = adc_read_avg(1,3);
	
	//vel between 0 - 450

	int16_t diff = normPos - acPos;

	if( diff*request_dir < 0) {
		pwm_set(3, 0);
		_delay_us(WAIT_TIME_DIR_CHANGE);
		TOGGLE(MOTOR_STOP_DIR);
	}

	if (diff < 0) {
	    diff *= -1;
	    request_dir = -1;
	} else if (diff > 0) {
	    request_dir = 1;
	}

	if( diff > 260 )
		diff = 260;

	if( diff < 10 )
		diff = 0;
	
	//char str[10];
	//sprintf(str, "V:%d", diff);
	//sprintf(str, "V:%u", normPos);
	//debug(str);
	
	pwm_set(3,diff);
	//pwm_set(3,10);
}

void	 servo_set(uint8_t pos) {
	//uint16_t val = (pos*(512-237)/100)+237;
	uint16_t val = pos*2.75 + 237;
	
	//char str[30];
	//sprintf(str, "Val : %d", val);
	//debug(str);
	
	pwm_set(0, val);
}
