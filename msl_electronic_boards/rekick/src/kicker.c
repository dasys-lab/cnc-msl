#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "global.h"
#include "defaults.h"
#include "booster.h"
#include "messages.h"
#include "uart.h"
#include "timer.h"
#include "ports.h"
#include "kicker.h"

// Kick without rotating the servo
// uncomment this to decativate servo
// warning: this may harm the hardware. fix the servo in a kick-position
//#define DEACTIVATE_SERVO
#define MAX(x,y) ((x)>(y)?(x):(y))
#define MIN(x,y) ((x)<(y)?(x):(y))

#ifdef DEACTIVATE_SERVO
#	warning This FW is configured to deactivate the servo.
#endif

// Use this if we have a hardware interlock with an solenoid
#define USE_SOLENOID_INTERLOCK

// allow a variance from a forced voltage (in Volt)
#define EPSILON_FORCED_VOLTAGE	0

// time in which the kick task expires
#define KICK_TASK_EXPIRE	2000

#define TIME_BETWEEN_TWO_SHOTS	200 // ms

// the logic state for the signal to lock/unlock the solenoid is defined here
#define UNLOCK_ROTOR		SET(ROTOR_INTERLOCK)
#define LOCK_ROTOR			RESET(ROTOR_INTERLOCK)
#define IS_ROTOR_LOCKED		(! IS_SET(ROTOR_CHECKLOCK))

int16_t lastServoValue = -1;
uint8_t MOVE_OFFSET = 20;


struct KICK_STRUCT {
	uint32_t timestamp;
	uint32_t last_kick;
	uint8_t  release_time;
	uint8_t  at_voltage;
} kick_job = {0, 0, 0, 0};

#define ROTJOB_START 0
#define ROTJOB_ROT 1
#define ROTJOB_END 2
#define ROTJOB_WIG_IN_LEFT 3
#define ROTJOB_WIG_IN_RIGHT 4

struct ROTATE_STRUCT {
	uint32_t timestamp;
	uint32_t last_rotate;
	uint8_t pos;
	int8_t dir;
	uint8_t state;
} rotate_job = {0, 0, 0, 0, 0};


// initialization without a usable number
//uint16_t servo_pos[3] = {150, 210, 90}; // PWM
uint8_t actual_pos = 1;		///< the target position of the rotor

uint32_t rotorLockTime = 0;
uint8_t rotorLocked    = 0;
uint8_t rotorWasLocked = 0;

//wiggle out
uint32_t wiggle_time = 0;
int8_t wiggle_dir = 0;


//wiggle in
uint8_t wigInCounter = 0;
uint16_t rotItCounter = 0;

uint8_t curServoPos = 1;

//FIVE(5) PARAMS ARE ROBOT SPECIFIC!
//   servo_pos: kicker1 kicker2 kicker3
//uint16_t servo_pos[3] = {153, 200, 102}; // PWM zwerg
//uint16_t servo_pos[3] = {140, 186, 92}; // PWM fransen (deadband 2)
//uint16_t servo_pos[3] = {147, 205, 94}; // PWM fransen (deadband 1)
//uint16_t servo_pos[3] = {155, 216, 102}; // PWM bart
uint16_t servo_pos[3] = {143, 204, 91}; // PWM scotti
//uint16_t servo_pos[3] = {150, 197, 101}; // PWM muecke
//uint16_t servo_pos[3] = {157, 219, 94}; // PWM TODO

//#define SERVO_MAX_POS 220
//#define SERVO_MIN_POS 94
//#define SERVO_MAX_POS 218 //BART
//#define SERVO_MIN_POS 94
//#define SERVO_MAX_POS 201 //MUECKE
//#define SERVO_MIN_POS 98
//#define SERVO_MAX_POS 191 //FRANSEN
//#define SERVO_MIN_POS 89
//#define SERVO_MAX_POS 201 //ZWERG
//#define SERVO_MIN_POS 100
#define SERVO_MAX_POS 210 //SCOTTI
#define SERVO_MIN_POS 80

//END ROBOT SPECIFIC PARAMS

volatile uint16_t rotation_pending_ms = 1;
volatile uint16_t target_val = 0;
uint8_t target_pos = 1;


// called every 10us
ISR(TIMER2_COMP_vect) {
	static uint16_t count = 0;
	static uint8_t uscnt = 0;
	if (count > target_val)
		PORTD &= ~(1 << PD7);
	else
		PORTD |= (1 << PD7);

	if (count < 2000 + target_val)
		count++;
	else
		count = 0;

	if (rotation_pending_ms > 0 && ++uscnt > 100) {
		uscnt = 0;
		rotation_pending_ms--;
	}
}

void kicker_init(void) {

        uint32_t i;
	target_val = servo_pos[0];

	// configure the port to release the kicker
	SET_OUTPUT(RELEASE);
	RESET(RELEASE);

	// even if we don't use the interlock, we unlock it to have a safe state
	SET_OUTPUT(ROTOR_INTERLOCK);
	UNLOCK_ROTOR;

	SET_INPUT(ROTOR_CHECKLOCK);

#ifdef SERIAL_SERVO
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
#else
	// Init PWM
	TIMSK |= (1 << OCIE2);
	TCCR2 |= (1 << WGM21) | (1 << CS20);
	OCR2 = F_CPU/100000;

	// Datadirection output Port D Pin 7
	DDRD |= (1 << DDD7);
	//PORTD &= ~(1 << PD7);

	wiggle_dir = -1;
        for (i=0; i < 600; i++) {
		
		if (IS_ROTOR_LOCKED) {
			if (wiggle_dir>0) target_val=SERVO_MAX_POS;
			if (wiggle_dir<0) target_val=SERVO_MIN_POS;
			if (i%80 == 0) {
				wiggle_dir *=-1;
				//i=0;
			}
		}
		else {
			break;
		}
	       _delay_ms(1);
	}
	target_val=servo_pos[0];		
	for (i=0; i< 340; i++) {
	       _delay_ms(1);	
	}
	LOCK_ROTOR;
	for(i=0; i < 400; i++) {
		if (IS_ROTOR_LOCKED) break;
		if (wiggle_dir>0) target_val=MIN(SERVO_MAX_POS,servo_pos[0]+6);
		if (wiggle_dir<0) target_val=MAX(SERVO_MIN_POS,servo_pos[0]-6);
		if (i%80==0) wiggle_dir*=-1;	
		_delay_ms(1);
	}
	wiggle_dir = 0;
	target_val=servo_pos[0];		

	rotate_job.timestamp = 0;

#endif
}

#ifdef SERIAL_SERVO
int8_t query_servo(char *query, uint8_t *reply) {
	
	uint8_t i = 0;
	uint32_t ts = timer_get_ms();
	int c;

	// send data
	if (query == NULL) {
		error("NULL value");
		reply = NULL;
		return -1;
	}
	uart_puts(query);
	
	// receive data
	do {

		if ((timer_get_ms() - ts) > UART_TIMEOUT) {
			warning("UART Timeout");
			return -1;
		}

		c = uart_getc();

		if (c == UART_NO_DATA) {
			continue;
		}
		else if (c == UART_BUFFER_OVERFLOW) {
			warning("UART buffer overflow");
			return -1;
		}
		else if (c == UART_OVERRUN_ERROR) {
			warning("UART overrun error");
			return -1;
		}
		else if (c == UART_FRAME_ERROR) {
			warning("UART frame error");
			return -1;
		}

		reply[i++] = (uint8_t) c;

		if (i >= BUFFER_LENGTH) {
			error("UART reply is too long");
			return -1;
		}

	} while (c != '\n');

	return i;
}
#endif

void kicker_rotate_servo(uint8_t num) {

	//uint8_t sreg;

	// sanity checks
	if (num < 1 || num > 3)
		return;

	// if the servo is deactivated, we do not unlock the rotor
	// and do not send data
#ifndef DEACTIVATE_SERVO
#  ifdef USE_SOLENOID_INTERLOCK
	// more checks
	/*if (IS_ROTOR_LOCKED) {
		error("ROTOR is still locked");
		return;
	}*/
#  endif
	// set PWM
	target_val = servo_pos[num-1];
		
#endif

	target_pos = num;

	return;
}
void kicker_rotate_servo_to(uint16_t pos) {
	// set PWM
	target_val = pos;
	return;
}


void kicker_rotate_servo_pos(uint8_t num, uint16_t pos) {

	uint8_t sreg;

	// sanity checks
	if (num < 1 || num > 3)
		return;

	// if the servo is deactivated, we do not unlock the rotor
	// and do not send data
#ifndef DEACTIVATE_SERVO
#  ifdef USE_SOLENOID_INTERLOCK
	// more checks
#  endif
	// set PWM
	target_val = pos;
	sreg = SREG;
	cli();
	rotation_pending_ms = 580; // modify this depending of the distance
	// smaller value if rotation way is short
	if (num == 1 || actual_pos == 1)
		rotation_pending_ms = 220;
	SREG = sreg;
		
#endif

	return;
}


void kicker_toggle_interlock(void) {

#ifdef USE_SOLENOID_INTERLOCK
	static char lock = 1;

	if (lock) {
		UNLOCK_ROTOR;
		lock = 0;
	}
	else {
		LOCK_ROTOR;
		lock = 1;
	}
#endif
}

#ifdef SERIAL_SERVO
/**
 * Query the servo for the actual position.
 *
 * @return -1 if an error occured
 */
int16_t servo_get_pos(void) {

	int16_t ret = -1;
	uint8_t reply[BUFFER_LENGTH];

	int8_t len = query_servo("g\n", reply);

	if (len == 4 && reply[0] == 'v' && reply[3] == '\n') {
		ret = (reply[1] << 8) + reply[2];
	}

	return ret;
}
#endif

uint8_t kicker_get_pos(void) {

	return actual_pos;
}

void kicker_rotate_handler(void) {

	// wait some time after last kick
	if ((timer_get_ms() - kick_job.last_kick) < 1000)
		return;

//	if (target_pos == rotate_job.pos)
//		return;

	// check i(servo_pos[num-1] < servo_pos[curServoPos])f a job is to do
	if (rotate_job.timestamp > 0) {

		// first unlock the rotor.
		// if the servo is deactivated, we do not unlock the rotor
#ifndef DEACTIVATE_SERVO
#ifdef USE_SOLENOID_INTERLOCK
		if (rotate_job.state==ROTJOB_START) {
//warning("start");
			UNLOCK_ROTOR;
			rotate_job.state = ROTJOB_ROT;
			wiggle_time = timer_get_ms();
			wiggle_dir = 0;
			rotItCounter = 0;


//warning("unlock");
			return;
		}
		//_delay_ms(10);
#endif
#endif
		// even if the servo is deactivated, let us drop in. we need this!
		if (! IS_ROTOR_LOCKED && rotate_job.state==ROTJOB_ROT) {
			kicker_rotate_servo_pos(rotate_job.pos,servo_pos[rotate_job.pos-1]+2*rotate_job.dir);
			rotate_job.state = ROTJOB_END;
//warning("rot");
			return;
		}
		if (rotate_job.state == ROTJOB_ROT) { //Lock is still in
			if(wiggle_time + 100 < timer_get_ms()) {
				if(wiggle_dir == 0) { 
					wiggle_dir = 1;				
				}
				else {
					wiggle_dir *=-1;
				}
				wiggle_time = timer_get_ms();		
			}
			if (wiggle_dir != 0) {
				kicker_rotate_servo_to(MAX(SERVO_MIN_POS,MIN(SERVO_MAX_POS,(servo_pos[curServoPos-1] + 5*wiggle_dir))));
			}

			//UNLOCK_ROTOR;
		}
		if (IS_ROTOR_LOCKED && rotate_job.state==ROTJOB_END) {
			curServoPos = rotate_job.pos;
			kicker_rotate_servo(rotate_job.pos);			
			rotate_job.timestamp = 0;
//warning("fin");			
			return;
		}
		/*else if(rotate_job.state==ROTJOB_END && rotation_pending_ms==0) {
			kicker_rotate_servo(rotate_job.pos);
			curServoPos = rotate_job.pos;
			//rotate_job.timestamp = 0;
		}*/
		if (rotate_job.state==ROTJOB_END && (++rotItCounter >= 2000)) {
			rotItCounter = 0;
			rotate_job.state = ROTJOB_WIG_IN_LEFT;
//warning("left");
		}
		if(IS_ROTOR_LOCKED && (rotate_job.state==ROTJOB_WIG_IN_LEFT || rotate_job.state==ROTJOB_WIG_IN_RIGHT)) {
//warning("fin2");
			kicker_rotate_servo(rotate_job.pos);
			rotate_job.state = ROTJOB_END;			
			return;	
		}
		if(rotate_job.state==ROTJOB_WIG_IN_LEFT) {
			kicker_rotate_servo_to(MAX(SERVO_MIN_POS,MIN(SERVO_MAX_POS,(servo_pos[rotate_job.pos-1] + 6))));
			if (++wigInCounter >= 100) {
				rotate_job.state=ROTJOB_WIG_IN_RIGHT;
//warning("right");
				wigInCounter = 0;
			}
		}
		if(rotate_job.state==ROTJOB_WIG_IN_RIGHT) {
			kicker_rotate_servo_to(MAX(SERVO_MIN_POS,MIN(SERVO_MAX_POS,(servo_pos[rotate_job.pos-1] - 6))));
			if (++wigInCounter >= 100) {
				rotate_job.state=ROTJOB_WIG_IN_LEFT;
//warning("left");
				wigInCounter = 0;
			}
		}
		

	}
}

/**
 * Make a rotation-job from the request.
 *
 * The execution is done by the appropriate handler.
 *
 * @param num The kicker position. (1-3)
 */
void kicker_add_rotate_job(uint8_t num) {
#ifdef DEACTIVATE_SERVO
	return;
#endif
	if(rotate_job.timestamp != 0 || curServoPos == num) return;
	//if(curServoPos == num) return;
	rotate_job.timestamp = timer_get_ms();
	rotate_job.pos = num;
	if (servo_pos[num-1] < servo_pos[curServoPos-1]) {
		rotate_job.dir = -1;	
	}
	else if(servo_pos[num-1] > servo_pos[curServoPos-1]) {
		rotate_job.dir = 1;	
	}
	else rotate_job.dir = 0;
	rotate_job.state = ROTJOB_START;
}

/**
 * Update the angle position to the appropiate kicker position
 *
 * @param num The kicker number
 * @param val The va#define ROTJOB_START 0
lue of the position
 */
void kicker_set_servo_pos(uint8_t num, uint16_t val) {
	
	// sanity checks
	if (num < 1 || num > 3) {
		return;
	}
#ifdef SERIAL_SERVO
	if (val < 50 || val > 1000) {
#else
	if (val < 50 || val > 250) {
#endif
		error("Cannot set value");
		return;
	}

	servo_pos[num-1] = val;
	//char tmp[10];
	//sprintf(tmp, "val: %d", val);
	//debug(tmp);
	
	// update position in a safe way
	if (actual_pos == num || target_pos == num)
		kicker_add_rotate_job(num);

	return;
}





// save the message
// the kick is done by kicker_task_handler
void kicker_add_kick_job(uint8_t ms) {
	kick_job.timestamp = timer_get_ms();
	kick_job.release_time = ms;

	return;
}

// save the message
// the kick is done by kicker_task_handler
void kicker_add_kick_job_forced(uint8_t ms, uint8_t forceVoltage) {
	if (forceVoltage > max_voltage || forceVoltage < max_voltage - 10) {
		warning("Cannot reach this voltage");
		return;
	}

	kick_job.timestamp = timer_get_ms();
	kick_job.release_time = ms;
	kick_job.at_voltage = forceVoltage;

	return;
}

// handle the kick job
// _not_ thread safe
void kicker_kick_handler(void) {
	uint8_t sreg;
	//uint8_t i;
	uint32_t time_now = timer_get_ms();

	// no job to do if timestamp is 0
	if (kick_job.timestamp == 0)
		return;

	// time between shots
	if (time_now - kick_job.last_kick < TIME_BETWEEN_TWO_SHOTS) {
		// invalidate data
		kick_job.timestamp = 0;
		return;
	}

	// handle forced_voltage
	if (kick_job.at_voltage > 0) {
		int16_t delta = (((int16_t) kick_job.at_voltage) - ((int16_t)get_capacitors_voltage()));
		if (abs(delta) > EPSILON_FORCED_VOLTAGE)
			return;
		kick_job.at_voltage = 0;
	}

	// the job expires after some milliseconds
	if (time_now - kick_job.timestamp > KICK_TASK_EXPIRE) {
		warning("Kick job expired.");
		kick_job.timestamp = 0;
		return;
	}

	// check if the booster is enabled
	if (!booster_can_kick()) {
		debug("Cannot kick. Booster state is disabled.");
		return;
	}
	
	// check if we are inside the kicker position
	if (actual_pos != target_pos) {
		//debug("Kicker is not aligned");
		return;
	}
	if (rotate_job.timestamp != 0) {
		return;
	}
	
	// dont kick if not locked
#ifdef USE_SOLENOID_INTERLOCK
	if(!IS_ROTOR_LOCKED) {
		return;
	}
#endif
/*	//activate for safer shooting
	if(rotorLocked!=1) { //hendrik
		return;
	}
*/		
	booster_pwm_disable();
	SET(RELEASE);
sreg = SREG;
cli();
	rotation_pending_ms = kick_job.release_time;
SREG=sreg;
	for(;;) {
//sreg = SREG;
//cli();
		if(rotation_pending_ms == 0) break;
//SREG=sreg;

	}
/*
	for (i = 0; i < kick_job.release_time; i++) {
		_delay_ms(1);
	}
*/	
	RESET(RELEASE);
	if (auto_boost)
		booster_pwm_enable();

	// debug time between kicker message and release
	char out[30];
	uint32_t delta = timer_get_ms() - kick_job.timestamp;
	if (delta > 65000)
		delta = 0;
	sprintf(out, "Kicktime: %u ms %u", (uint16_t)delta);
	debug(out);

	// everything fine
	kick_job.timestamp = 0;
	kick_job.last_kick = timer_get_ms();

	return;
}

#ifdef SERIAL_SERVO
void kicker_set_servo(bool status)
{
	if( status )
	{
		// send data to the servo
		uart_putc('o');
		uart_putc('n');
		uart_putc('\n');
	}
	else
	{
		// send data to the servo
		uart_putc('o');
		uart_putc('f');
		uart_putc('f');
		uart_putc('\n');
	}
}
#endif



#ifdef USE_SOLENOID_INTERLOCK
/**
 * Interock the rotor
 */
//Sketch
void kicker_interlock_handler(void) {

	uint8_t sreg;
	// lock the rotor
	sreg = SREG;
	cli();
	if (rotation_pending_ms == 0 && (rotate_job.state==ROTJOB_END || rotate_job.state==ROTJOB_WIG_IN_LEFT || rotate_job.state==ROTJOB_WIG_IN_RIGHT)) {// && rotate_job.timestamp > 0)) {
		LOCK_ROTOR;
		actual_pos = target_pos;
	} else {
		//UNLOCK_ROTOR;	
	}
	SREG = sreg;
/* //activate for safer shooting
	if (IS_ROTOR_LOCKED) {
		if (rotorWasLocked) {
			if (timer_get_ms()-rotorLockTime >= 50) {
				rotorLocked = 1;
			}
		} else {
			rotorLockTime = timer_get_ms();
			rotorWasLocked = 1;
		}
	} else {
		rotorLocked = 0;
		rotorWasLocked = 0;
	}
*/
	
}
#endif

void kicker_task_handler(void) {
	kicker_rotate_handler();
#ifdef USE_SOLENOID_INTERLOCK
	kicker_interlock_handler();
#endif
	kicker_kick_handler();
}

