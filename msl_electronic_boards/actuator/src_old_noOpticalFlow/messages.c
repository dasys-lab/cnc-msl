#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "mcp2515.h"
#include "messages.h"
#include "mcp2515_defs.h"
#include "timer.h"
#include "global.h"
#include "defaults.h"
#include "version.h"
#include "motor.h"
#include <string.h>

#define CAN_MSG_BUFFER_LENGTH	30 // max 255
#define SEND_DELAY	10	//ms

#define STRING(a)	# a
#define XSTRING(s)	STRING(s)

uint8_t manual_mode = false;

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
prog_char can_filter[] = {
	MCP2515_FILTER_EXTENDED(0x000000A0),	// Filter 0
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
	uint8_t id[4] = {0x00, PRIORITY_NORM, SENSORBOARD_ID, ETH2CAN_ID};

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
		//char buf[10];
		//sprintf(buf,"id:%i",reply.id[3]);
		//uart1_puts(buf);
		// soft-CAN_ID-filter
		if (reply.id[3] == SENSORBOARD_ID) {
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
void message_handler(void) {
	can_receive_handler();
	can_send_handler();
	handle_error();
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
	uint8_t id[4] = {0x00, PRIORITY_NORM, SENSORBOARD_ID, ETH2CAN_ID};
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
 * Parser fuction for the default state.
 *
 * This is the parser for the normal operation.
 *
 * \param m The received message
 */
void parse_default(tExtendedCAN *m) {

	switch (m->data[0]) {
		case CMD_PING:
			last_heartbeat = timer_get_ms();
			can_put_cmd(CMD_PONG, NULL, 0);
			break;
		case CMD_GET_VERSION:
			{
				char *version = "RK" XSTRING(MAJOR) "." XSTRING(MINOR);
				uint8_t len = strlen(version);
				if (len > 7) len = 7;
				can_put_cmd(CMD_VERSION, (uint8_t*) version, len);
			}
			break;
		case CMD_DRIBBLE:
			{
				ballhandler_set((int8_t)m->data[1],(int8_t)m->data[2]);
			}
			break;
		case CMD_STOP:
			{
				stop_set((uint8_t)m->data[1]);
			}
			break;
		case CMD_SERVO:
			{
				servo_set((uint8_t)m->data[1]);
			}
			break;
		case 'm':
			debug("manual");
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

	uint16_t tmp = 0;
	uint16_t tmp2 = 0;
	uint8_t i;
	char str[30];
	bool ending = false;

	//dribbling
	if (m->data[0] == 'd') {
		tmp = atoi(m->data+1);
		char* pos = strchr(m->data,',');
		if (pos != NULL) {
		  tmp2 = atoi(pos+1);
		}
		
		sprintf(str, "Dribbling set : %d %d \n", tmp, tmp2);
		debug(str);
		ballhandler_set(tmp, tmp2);
	}
	// stop ball mechanism
	else if (m->data[0] == 's') {
		for (i = 1; i < m->header.length; i++) {
			if (m->data[i] >= 0x30 && m->data[i] <= 0x39) {
				tmp = tmp * 10 + (m->data[i] - 0x30);
				if (tmp > 254) {
					error("ERR DATE");
					return;
				}
			}
			else {
				error("ERR NAN");
				return;
			}
		}
		sprintf(str, "Stopping set : %d\n", tmp);
		debug(str);
		stop_set(tmp);
	}
	//servo set
	else if (m->data[0] == 'k') {
		for (i = 1; i < m->header.length; i++) {
			if (m->data[i] >= 0x30 && m->data[i] <= 0x39) {
				tmp = tmp * 10 + (m->data[i] - 0x30);
				if (tmp > 254) {
					error("ERR DATE");
					return;
				}
			}
			else {
				error("ERR NAN");
				return;
			}
		}
		sprintf(str, "Kicker set : %d \n", tmp);
		debug(str);
		servo_set(tmp);
	}
	// switch back to AUTOMATIC MODE
	// (without the driver the rekick driver
	// (the one in c#), the system goes into standby mode)
	else if (m->data[0] == 'a') {
		debug("auto");
		parse_data = parse_default;
		manual_mode = false;
	}
	else {
		error("ERR IMPL");
	}

	return;
}

void handle_error(void)
{
	char buf[32];

	if( TXB0 ) {
		uint8_t err = mcp2515_read_status(EFLG);
		if(err) {
			sprintf(buf,"reinit: %x\n",err);
			uart1_puts(buf);
			//error
			mcp2515_init();
			mcp2515_write_register( EFLG, 0x0 );
			//debug(buf);
			sprintf(buf,"reinit: %x\n",err);
		}
	}
}
