
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "defaults.h"
#include "global.h"
#include "ports.h"
#include "messages.h"
#include "timer.h"
#include "kicker.h"


#define PING_TIMEOUT			1000  // ms
#define MAX_BOOST_TIME			30000 // ms
#define MINIMUM_SUPPLY_VOLTAGE	17    // Volt

#define PWM_IS_ENABLED	(TCCR1A & (1 << COM1B1))

uint8_t booster_pwm_lock = 1;
uint8_t booster_auto_off = 1;
uint32_t pwm_start = 0;
uint32_t last_heartbeat = 0; // ms
uint8_t manual_mode = false;
uint8_t auto_boost = true;
uint16_t max_voltage = 330;
uint8_t init = 0;
uint8_t initMeassure = 0;

struct VOLTAGE_ERROR_STRUCT {
	uint8_t error :1;    //< this locks the booster
	uint8_t warn  :7;    //< this represents the warn levels
	uint32_t last_error; //< time in ms of the last check
} voltage_error_t = {0, 0};


// the info message
// 1. byte: CMD_STATE
// 2. byte: booster_state
// 3. byte: supply voltage HB
// 4. byte: supply voltage LB
// 5. byte: capacitors voltage
//
// booster_state:
// bit order: 76543210
// bit 7: Supply on
// bit 6: BoostPWM on
// bit 5: release on
// bit 4: error_state
// bit 3: reserved
// bit 2: reserved
// bit 1: rotate pos
// bit 0: rotate pos

/**
 * Structure builds the booster state
 *
 * 8 bit long
 */
struct BOOSTER_STATE {
	uint8_t               : 2; //< reserved two bit
	uint8_t error_state   : 1; //< Critical error occured?
	uint8_t release_state : 1; //< release switch state
	uint8_t pwm_state     : 1; //< load PWM state
	uint8_t power_state   : 1; //< booster supply state
};

/**
 * This structure builds the info message
 *
 * 4 bytes long
 */
struct BOOSTER_INFO {
	struct BOOSTER_STATE state;		//< the state of the booster
	uint16_t supply_voltage;		//< the adc value from the supply voltage. Not the real value!
	uint16_t  capacitors_voltage;	//< the voltage of the capacitors (Volt)
};

void booster_send_info(void) {

	struct BOOSTER_INFO info;
	
	info.state.power_state = ( (IS_SET(POWER_NORM) && !IS_SET(POWER_INV) ) > 0);
	info.state.pwm_state = ((PWM_IS_ENABLED) > 0);
	info.state.release_state = (IS_SET(RELEASE) > 0);
	info.state.error_state = voltage_error_t.error;
	
	info.supply_voltage = get_supply_raw_voltage();
	info.capacitors_voltage = get_capacitors_voltage();

	can_put_cmd(CMD_STATE, (uint8_t *)&info, 5);
}

void booster_pwm_enable(void) {

	if (booster_pwm_lock)
		return;

	// save timestamp
	pwm_start = timer_get_ms();

	TCCR1A |= (1 << COM1B1);
	RESET(LED_RED1); // PORT off - LED on
}

void booster_pwm_disable(void) {

	// visual that the pwm is off
	pwm_start = 0;

	TCCR1A &= ~(1 << COM1B1);
	// Make sure the output port ist disabled
	PORTD &= ~(1 << PD4);
	SET(LED_RED1); // PORT on - LED off
}

// disables power supply, locks the booster and dischages the capacitors
void booster_disable(void) {
	booster_pwm_lock = 1;
	booster_pwm_disable();
	_delay_ms(5);
	RESET(POWER_NORM);
	RESET(POWER_INV);
	RESET(NOT_DISCHARGE);
}

void booster_enable(void) {
	SET(NOT_DISCHARGE);
	SET(POWER_NORM);
	RESET(POWER_INV);
	//_delay_ms(500);
	if(init == 0) {
		init = 1;
		uint8_t sreg = SREG;
		cli();
		pending_us10 = 50000; //500ms * 100
		SREG=sreg;
	}
	else {
		if( pending_us10 <= 0)
		{
			init = 0;
			debug("Enable");
			booster_pwm_lock = 0;
			booster_pwm_enable();
			RESET(LED_GREEN1);
			booster_auto_off = 0;
		}
	}
}

void booster_set_max_voltage(uint16_t voltageIn) {
/*char stmp[16];
sprintf(stmp,"v2: %uV",voltageIn);
debug(stmp);*/
	if (voltageIn > 330u) {
		error("Cannot set max voltage");
	}

	max_voltage = voltageIn;
}

uint8_t booster_can_kick(void) {
	if ( IS_SET(POWER_NORM) && !IS_SET(POWER_INV) ) {
		return 1;
	}
	else {
		return 0;
	}
}

void booster_ctrl(void) {
	
	uint16_t capacitors = get_capacitors_voltage();
	uint32_t time_now = timer_get_ms();
	static uint32_t last_msg = 0;

	if (time_now < PING_TIMEOUT) {
		return;
	}
	
	// lock booster if an error occurret
	if (voltage_error_t.error) {
		booster_disable();
		uint32_t time_now = timer_get_ms();
		if (time_now - last_msg > 1000) {
			warning("Critical error. Booster disabled!");
			last_msg = time_now;
		}
		return;
	}

	// check supply voltage
	if (get_supply_voltage() < MINIMUM_SUPPLY_VOLTAGE) {
		// disable booster
		if (timer_get_ms() - voltage_error_t.last_error < 1000)
			return;
		booster_pwm_lock = 1;
		booster_pwm_disable();
		_delay_ms(5);
		RESET(POWER_NORM);
		RESET(POWER_INV);
		voltage_error_t.warn++;

		if (voltage_error_t.warn >= 5) {
			voltage_error_t.error = 1;
			booster_disable();
			warning("Supply voltage is too low. Booster disabled.");
		}
		else {
			warning("Supply voltage is too low.");
		}

		voltage_error_t.last_error = timer_get_ms();
		
		return;
	}
	else {
		// check for reenable
		if (voltage_error_t.warn) {
			if (timer_get_ms() - voltage_error_t.last_error < 5000)
				return;
			warning("Reenable booster.");
			SET(POWER_NORM);
			RESET(POWER_INV);
			booster_pwm_lock = 0;
			voltage_error_t.warn = 0;
		}
	}
		
	// check how long the charge takes time
	if (pwm_start > 0) { // pwm enabled?
		if ((time_now - pwm_start) > MAX_BOOST_TIME) {
			uint16_t cv = get_capacitors_voltage();
			if (cv < 17) {
				warning("Check cable between ReKick and booster!");
				voltage_error_t.error = 1;
			}
			else {
				warning("Charging took too long. Lock ReKick!");
				voltage_error_t.error = 1;
			}
			return;
		}
	}

	// disables the booster stage if we did not get a heartbeat from the laptop.
	if (time_now - last_heartbeat < PING_TIMEOUT || manual_mode) {
		if (booster_auto_off) {
			//wait 0.5sec till dc-dc is loaded (interrupt)
			//char deb[8];
			//sprintf(deb, "pen%d\n", pending_ms);
			//debug(deb);
			booster_enable();
		}

		if (!auto_boost)
			return;

		if (capacitors >= max_voltage && PWM_IS_ENABLED) {

		/*	if(initMeassure == 0) {
				initMeassure = 1;
				uint8_t sreg = SREG;
				cli();
				pending_ms = 420;
				SREG=sreg;
			}
			else {
				if( pending_ms <= 0)
				{
					initMeassure = 0;
					if (manual_mode)
						debug("A disab");
					booster_pwm_disable();
				}
			}
*/
			if (manual_mode)
				debug("A disab");
			booster_pwm_disable();
		}
		else if (capacitors <= max_voltage - 10 &&
				(booster_pwm_lock == 0) &&
				!PWM_IS_ENABLED) {
			if (manual_mode)
				debug("A enab");
			booster_pwm_enable();

		}
	}
	else {
		// disable everything
		if (!booster_auto_off) {
			booster_disable();
			SET(LED_GREEN1);
			booster_auto_off = 1;
		}
	}


}

// TODO: booster is disabled by default
void booster_init(void) {

	// set PWM channel as output and disable
	SET_OUTPUT(PWM_BOOST);
	RESET(PWM_BOOST);

	// Init PWM but dont connect the output port => disabled
	// Phase Correct, TOP is OCR1A, Update OCR1x at TOP, prescaler 8
	// Clear OC1B on Compare Match when upcounting. Set OC1B on Compare
	// Match when downcounting.
	TCCR1A = (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM13) | (1 << CS10);

	// timing for the pwm
	OCR1A = 104; // Obergrenze -> 13us Periode	
	//OCR1B = 52;
	OCR1B = 32;
	//OCR1A = 114;
	//OCR1B = 33;

	// Status LED red shows red
	SET_OUTPUT(LED_RED1);
	SET(LED_RED1); // PORT on - LED off
	SET_OUTPUT(LED_GREEN1);
	SET(LED_GREEN1); // PORT on - LED off

	booster_pwm_disable();
	_delay_ms(5);

	// Booster power is disabled per default
	SET_OUTPUT(POWER_NORM);
	SET_OUTPUT(POWER_INV);
	RESET(POWER_NORM);
	RESET(POWER_INV);

	// disable discharging
	SET_OUTPUT(NOT_DISCHARGE);
	SET(NOT_DISCHARGE);

	// per default the booster is off
	booster_auto_off = 1;

}
