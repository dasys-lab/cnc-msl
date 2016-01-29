#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "mcp2515.h"
#include "booster.h"
#include "kicker.h"
#include "messages.h"
#include "timer.h"
#include "ports.h"
#include "global.h"
#include "defaults.h"
#include "version.h"

#define CAN_MSG_BUFFER_LENGTH	30 // max 255
#define SEND_DELAY	10	//ms

#define STRING(a)	# a
#define XSTRING(s)	STRING(s)

// prototypes
void parse_default(tExtendedCAN *m);
void parse_manual(tExtendedCAN *m);

// globales
void (*parse_data)(tExtendedCAN *) = parse_default;

tExtendedCAN can_buffer[30];
uint8_t can_buffer_head = 0;
uint8_t can_buffer_tail = 0;

// Create filter and mask table
// 
// Both masks are zero => all messages are received
// filter 00000060
// mask   0000FFFF
// -----------------
// id     XXXX0060 <-- this is how the id has to look like
const char can_filter[] PROGMEM = {
	MCP2515_FILTER_EXTENDED(0x00000060),	// Filter 0
	MCP2515_FILTER_EXTENDED(0),	// Filter 1
	MCP2515_FILTER_EXTENDED(0),	// Filter 2
	MCP2515_FILTER_EXTENDED(0),	// Filter 3
	MCP2515_FILTER_EXTENDED(0),	// Filter 4
	MCP2515_FILTER_EXTENDED(0),	// Filter 5
	
	// mask0 -> buffer RXB0
	// mask1 -> buffer RXB1
	MCP2515_FILTER_EXTENDED(0x0000FFFF),	// Mask 0
	MCP2515_FILTER_EXTENDED(0x0000FFFF),	// Mask 1
};


/**
 * Initialize the can controller
 */
void can_init(void) {

	mcp2515_init();

	mcp2515_static_filter(can_filter);
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2) | (1<<REQOP1) | (1<<REQOP0), 0);
}

/**
 * Runs a test on the can_controller
 *
 * Generates 20 messages where the id is incremented with each message.
 */
void can_test(void) {

	tExtendedCAN message;
	uint8_t i;
	
	// 0x00, priority, sender, receiver
	uint8_t id[4] = {0x00, PRIORITY_NORM, REKICK_ID, ETH2CAN_ID};

	message.header.length = 1;

	for (i = 0; i < 20; i++) {
		
		id[3] = i;
		generate_extCAN_ID(id, message.id);
		message.data[0] = i;
		message.header.rtr = 0;

		while(mcp2515_send_extmessage(&message) == 0);
	}

	return;
}

/**
 * Recieve CAN Messages
 */
void can_receive_handler(void) {

	tExtendedCAN reply;

	if (mcp2515_check_message()) {
		mcp2515_get_extmessage(&reply);
		// soft-CAN_ID-filter
		if (reply.id[3] == REKICK_ID) {
			parse_data(&reply);
		}

	}
}

/**
 * Send CAN Messages
 *
 * This handler send messages which are stored in the buffer.
 *
 * \see \ref can_buffer
 */
void can_send_handler(void) {

	static uint32_t last_send = 0; // ms

	if (can_buffer_head == can_buffer_tail)
		return;

	
	if (timer_get_ms() - last_send < SEND_DELAY)
		return;
	

	if (mcp2515_send_extmessage(&can_buffer[can_buffer_tail]) != 0) {
		if (++can_buffer_tail == CAN_MSG_BUFFER_LENGTH)
			can_buffer_tail = 0;
		last_send = timer_get_ms();
	}

}

/**
 * CAN Message handler
 */
uint8_t itcounter = 0;

void message_handler(void) {
	can_receive_handler();
	can_send_handler();
	if (manual_mode && itcounter++ > 40) {
		itcounter = 0;
		print_voltage();
		can_send_handler();
	}
}

/**
 * Add a message to the send-buffer.
 *
 * This message takes a message and adds it to the send-buffer.
 *
 * \see \ref can_send_handler
 * \see \ref can_buffer
 *
 * \param msg The message structure to send
 */
void can_send_message(tExtendedCAN *msg) {

	// copy data
	memcpy(&can_buffer[can_buffer_head], msg, sizeof(tExtendedCAN));

	if (++can_buffer_head == CAN_MSG_BUFFER_LENGTH)
		can_buffer_head = 0;

	// check for buffer overflow.
	if (can_buffer_head == can_buffer_tail) {
		// overwrite existing data
		if (++can_buffer_tail == CAN_MSG_BUFFER_LENGTH)
			can_buffer_tail = 0;
	}

}

/**
 * Send data without buffer
 */
void can_send_message2(tExtendedCAN *message) {

	uint32_t send_ms = timer_get_ms();

	while (mcp2515_send_extmessage(message) == 0) {
		if ((timer_get_ticks() - send_ms) > 10)
			return;
	}
}


/**
 * Generate CAN-Message
 *
 * \param cmd The ReKick command
 * \param str A char array which holds the message.
 * \param len The length of the message
 */
void can_put_cmd(uint8_t cmd, uint8_t* str, uint8_t len) {

	tExtendedCAN message;

	// 0x00, priority, sender, receiver
	uint8_t id[4] = {0x00, PRIORITY_NORM, REKICK_ID, ETH2CAN_ID};
	generate_extCAN_ID(id, message.id);
	message.header.rtr = 0;

	// packet length is limited to 8 chars but one char
	// is already used for the cmd.
	if (len > 7)
		len = 7;

	message.data[0] = cmd;
	memcpy(message.data+1, str, len);
	message.header.length = len+1;

	// send to buffer
	can_send_message(&message);


	return;
}

/**
 * Split message into one or more messages.
 *
 * Splits the message therfor it fits into 8 bytes (limit of CAN data)
 *
 * \param cmd The ReKick command
 * \param A NULL terminated string
 */
void split_message(uint8_t cmd, char *str) {

	uint8_t len = strlen(str);
	uint8_t i;

	for (i = 0; i < len;) {
		if (len-i > 7) {
			can_put_cmd(cmd, ((uint8_t*)str) + i, 7);
		}
		else {
			can_put_cmd(cmd, ((uint8_t*)str) + i, len-i);
		}

		i += 7;
	}

	return;
}

/**
 * Sends a debug message to the ReKick Driver
 *
 * \see \ref warning
 * \see \ref error
 *
 * \param str A NULL terminated string
 */
void debug(char *str) {

	uint8_t len = strlen(str);

	if (str[len-1] != '\n') {
		char newstr[len+2];
		memcpy(newstr, str, len);
		newstr[len] = '\n';
		newstr[len+1] = 0x00;
		split_message(CMD_MSG, newstr);
	}
	else {
		split_message(CMD_MSG, str);
	}

	return;
}

/**
 * Sends a error message to the ReKick Driver
 *
 * \see \ref debug
 * \see \ref error
 *
 * \param str A NULL terminated string
 */
void warning(char *str) {

	uint8_t len = strlen(str);

	if (str[len-1] != '\n') {
		char newstr[len+2];
		memcpy(newstr, str, len);
		newstr[len] = '\n';
		newstr[len+1] = 0x00;
		split_message(CMD_WARNING, newstr);
	}
	else {
		split_message(CMD_WARNING, str);
	}

	return;
}

/**
 * Sends a error message to the ReKick Driver
 *
 * \see \ref debug
 * \see \ref warning
 *
 * \param str A NULL terminated string
 */
void error(char *str) {

	uint8_t len = strlen(str);

	if (str[len-1] != '\n') {
		char newstr[len+2];
		memcpy(newstr, str, len);
		newstr[len] = '\n';
		newstr[len+1] = 0x00;
		split_message(CMD_ERROR, newstr);
	}
	else {
		split_message(CMD_ERROR, str);
	}

	return;
}

/**
 * Removes all message from the receive buffer
 */
void clear_receive_buffer(void) {

	tExtendedCAN reply;

	while (mcp2515_check_message())
		mcp2515_get_extmessage(&reply);
}

/**
 * Callback function which prints the actual capacitors message
 */
void print_voltage(void) {
	
	char str[20];
	
	sprintf(str, "%dV", get_capacitors_voltage());
	debug(str);
}

/**
 * Parser fuction for the default state.
 *
 * This is the parser for the normal operation.
 *
 * \param m The received message
 */
void parse_default(tExtendedCAN *m) {

	volatile uint16_t tmpa = 0;

	switch (m->data[0]) {
		case CMD_PING:
			last_heartbeat = timer_get_ms();
			can_put_cmd(CMD_PONG, NULL, 0);
			break;
		case CMD_KICK:
			if (timer_get_ms() > 1000) {
				tmpa = m->data[1] + (m->data[2]<<8);
				if (m->header.length == 3)
					kicker_add_kick_job(tmpa);
				else if (m->header.length == 4)
					kicker_add_kick_job_forced(tmpa, m->data[3]);
			}
			break;
		case CMD_SET_MAX_VOLTAGE:
			
			if (m->header.length == 3) {

				tmpa = m->data[1] + (m->data[2]<<8);
				booster_set_max_voltage(tmpa);

			} else if (m->header.length == 2) {
				tmpa = ((uint16_t)(m->data[1])) & 0xFF;
				booster_set_max_voltage(tmpa);

			}
			break;
		case CMD_ROTATE:
			break;
		case CMD_GET_VERSION:
			{
				char *version = "RK" XSTRING(MAJOR) "." XSTRING(MINOR);
				uint8_t len = strlen(version);
				if (len > 7) len = 7;
				can_put_cmd(CMD_VERSION, (uint8_t*) version, len);
			}
			break;
		case CMD_GET_STATE:
			booster_send_info();
			break;
		case CMD_SET_PULSE_WIDTH: //silently ignore
			break;
		case 'm':
			debug("manual");
			//timer_register(print_voltage, 1000);
			manual_mode = true;
			parse_data = parse_manual;
			break;
		default:
			error("Command not implemented");
			break;
	}
}

/**
 * Parser function for the manual state.
 *
 * This is used in manual mode.
 *
 * \param m The received message.
 */
void parse_manual(tExtendedCAN *m) {

	static uint16_t release_time = 33;
	uint16_t tmp = 0;
	uint8_t i;
	char str[8];

	// set the power of a shot
	// full power is round about 3000
	// a slow pass is about 800
	if (m->data[0] == 's') {
		for (i = 1; i < m->header.length; i++) {
			if (m->data[i] >= 0x30 && m->data[i] <= 0x39) {
				tmp = tmp * 10 + (m->data[i] - 0x30);
				if (tmp > 9999) {//254) {
					error("ERR DATE");
					return;
				}
			}
			else {
				error("ERR NAN");
				return;
			}
		}
		release_time = tmp;
		sprintf(str, "nt%d\n", tmp);
		debug(str);
	}
	// SPACE release the kicker
	else if (m->data[0] == ' ') {
		if (release_time > 0)
			kicker_add_kick_job(release_time);
		debug("Release");
	}
	// enable auto boosting
	else if (m->data[0] == 'e') {
		auto_boost = true;
	}
	// force boosting. disable the software control
	// warning this may overload the capacitors if the
	// hardware disabling function fails
	else if (m->data[0] == 'w') {
		auto_boost = false;
		booster_pwm_enable();
	}
	// disable charging but holds the power
	else if (m->data[0] == 'q') {
		auto_boost = false;
		booster_pwm_disable();
	}
	// switch back to AUTOMATIC MODE
	// (without the driver the rekick driver
	// (the one in c#), the system goes into standby mode)
	else if (m->data[0] == 'a') {
		//timer_deregister(print_voltage);
		auto_boost = true;
		manual_mode = false;
		parse_data = parse_default;
	}
	else {
		error("ERR IMPL");
	}

	return;
}

