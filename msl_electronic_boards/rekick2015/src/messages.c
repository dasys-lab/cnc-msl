/*
 * can.c
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */

#include "messages.h"

#include "booster.h"

#include "defaults.h"
#include "global.h"
#include "version.h"

tExtendedCAN can_buffer[CAN_MSG_BUFFER_LENGTH];
uint8_t tx_buffer[DATA_BUFFER_SIZE];
uint8_t rx_buffer[DATA_BUFFER_SIZE];	// buffer to hold data from ipc

st_cmd_t tx_msg; // Tx MOb (MOb to Tranceive Data)
st_cmd_t rx_msg; // Rx Mob (MOb for Receiving Data)


uint8_t can_buffer_head = 0;
uint8_t can_buffer_tail = 0;

// prototypes
void parse_default(uint8_t *data, uint8_t length);
void parse_manual(uint8_t *data, uint8_t length);

void (*parse_data)(uint8_t *, uint8_t) = parse_default;


void communication_init()
{
	SET_OUTPUT(CAN_TX);
	RESET(CAN_TX);
	SET_INPUT(CAN_RX);
	RESET(CAN_RX);

	can_init(0);

	// Interrupts aktivieren?

	// Receive-Buffer
	rx_msg.pt_data = &rx_buffer[0];		// Point Rx MOb to first element of buffer
	rx_msg.status = 0;					// clear status
	configureRxMOb();
	
	// Transmit-Buffer
	tx_msg.pt_data = &tx_buffer[0];		// Point Tx MOb to first element of buffer
	tx_msg.status = 0;					// clear status
}

void message_handler()
{
	message_receive_handler();
	message_transmit_handler();
}

void message_receive_handler()
{
	switch (can_get_status(&rx_msg)) {
		case CAN_STATUS_COMPLETED:
			// Message received
			if ((rx_msg.id.ext & 0x000000FF) == REKICK_ID) {	// Ext-ID masked with 0x000000FF
				parse_data(rx_msg.pt_data, rx_msg.dlc);
			}
			rx_msg.status = 0;	// clear status
			break;
			
		case CAN_STATUS_NOT_COMPLETED:
			// waiting for Message
			return;
			break;
			
		case CAN_STATUS_ERROR:
			// CAN ERROR HANDLING
			
		default:
			rx_msg.cmd = CMD_ABORT;
			while (can_cmd(&rx_msg) != CAN_CMD_ACCEPTED);
			rx_msg.status = 0;	// clear status
			break;
	}

	configureRxMOb();	// activate Rx MOb again
}

void message_transmit_handler()
{
	if (can_buffer_head == can_buffer_tail)
		return;

	if (sendMsg(&can_buffer[can_buffer_tail]) > 0)
	{
		// sendMsg successful
		if (++can_buffer_tail == CAN_MSG_BUFFER_LENGTH)
			can_buffer_tail = 0;
	} else
	{
		// sendMsg not successful
		tx_msg.cmd = CMD_ABORT;
		while (can_cmd(&tx_msg) != CAN_CMD_ACCEPTED);
		tx_msg.status = 0;	// clear status

		// Some more Error handling?
		// CAN ERROR HANDLING
	}
}

void generate_extCAN_ID(uint8_t *bytes, uint32_t *result)
{
	*result = (bytes[0] << 3) | (bytes[1] >> 5);
	*result |= (uint32_t) ((bytes[1] & 0x03) | 0x0C | ((bytes[1] & 0xFC) << 3)) << 8;
	*result |= (uint32_t) bytes[2] << 16;
	*result |= (uint32_t) bytes[3] << 24;
}

void configureRxMOb()
{
	for(int i=0; i<DATA_BUFFER_SIZE; i++) {rx_buffer[i] = 0;}	// clear message object data buffer

	rx_msg.id.ext = 0x00000000;			// This message object only accepts frames from Target IDs (0x80) to (0x80 + NB_TARGET)
	rx_msg.ctrl.ide = 1;				// This message object accepts only extended (2.0B) CAN frames
	rx_msg.ctrl.rtr = 0;				// this message object is not requesting a remote node to transmit data back
	rx_msg.dlc = DATA_BUFFER_SIZE;		// Number of bytes (8 max) of data to expect
	rx_msg.cmd = CMD_RX_DATA;			// assign this as a "Receive Extended (2.0B) CAN frame" message object

	while(can_cmd(&rx_msg) != CAN_CMD_ACCEPTED); // Wait for MOb to configure (Must re-configure MOb for every transaction)
}

int8_t sendMsg(tExtendedCAN *message)
{
	memcpy(tx_buffer, message->data, message->length);

	tx_msg.id.ext = message->id;		// This message object only sends frames to Target IDs
	tx_msg.ctrl.ide = 1;				// This message object sends extended (2.0B) CAN frames
	tx_msg.ctrl.rtr = 0;				// This message object is sending Data
	tx_msg.dlc = message->length;		// Number of data bytes (8 max)
	tx_msg.cmd = CMD_TX_DATA;

	while(can_cmd(&tx_msg) != CAN_CMD_ACCEPTED); // Wait for MOb to configure (Must re-configure MOb for every transaction) and send request
	while(can_get_status(&tx_msg) == CAN_STATUS_NOT_COMPLETED);	// Wait for Tx to complete

	switch (tx_msg.status)
	{
		case MOB_TX_COMPLETED:
			break;

		case MOB_ACK_ERROR:
		case MOB_FORM_ERROR:
		case MOB_CRC_ERROR:
		case MOB_STUFF_ERROR:
		case MOB_BIT_ERROR:
			// Error Handling
			break;

		default:
			break;
	}
	return 1;
}

/**
 * Generate CAN-Message
 *
 * \param cmd The ReKick command
 * \param data A uint8_t array which holds the message.
 * \param length The length of the message
 * \param priority The priority (PRIORITY_LOW, _NORM, _HIGH) of the message
 */
int8_t prepareMsg(uint8_t cmd, uint8_t* data, uint8_t length, uint8_t priority)
{
	tExtendedCAN message;

	// 0x00, priority, sender, receiver
	uint8_t id[4] = {0x00, priority, REKICK_ID, ETH2CAN_ID};
	generate_extCAN_ID(id, &message.id);

	if (length > 7)
		length = 7;
	message.length = length + 1;

	message.data[0] = cmd;
	memcpy(message.data+1, data, length);

	can_buffer[can_buffer_head] = message;

	if (++can_buffer_head == CAN_MSG_BUFFER_LENGTH)
		can_buffer_head = 0;

	// check for buffer overflow.
	if (can_buffer_head == can_buffer_tail) {
		// overwrite existing data
		if (++can_buffer_tail == CAN_MSG_BUFFER_LENGTH)
			can_buffer_tail = 0;
	}

	return 1;
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
			prepareMsg(cmd, ((uint8_t*)str) + i, 7, PRIORITY_NORM);
		}
		else {
			prepareMsg(cmd, ((uint8_t*)str) + i, len-i, PRIORITY_NORM);
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
 * Parser function for the default state.
 *
 * This is the parser for the normal operation.
 *
 * \param data The data of the received message
 * \param length The length of the received message
 */
void parse_default(uint8_t *data, uint8_t length) {

	volatile uint16_t tmpa = 0;
	uint8_t placeholder[5] = {1, 2, 3, 4, 5};

	switch (data[0]) {
		case CMD_PING:
			timer_get_ms(&last_heartbeat);
			prepareMsg(CMD_PONG, placeholder, 0, PRIORITY_NORM);
			break;

		case CMD_KICK:
			if (1) {//timer_get_ms() > 1000) {
				tmpa = data[1] + (data[2] << 8);
				if (length == 3)
					kicker_addKickJob(tmpa);
				else if (length == 4)
					kicker_addKickJobForced(tmpa, data[3]);
			}
			break;

		case CMD_SET_MAX_VOLTAGE:
			if (length == 3) {
				tmpa = data[1] + (data[2]<<8);
				booster_setMaxVoltage((double) tmpa);

			} else if (length == 2) {
				tmpa = ((uint16_t)(data[1])) & 0xFF;
				booster_setMaxVoltage((double) tmpa);
			}
			break;

		case CMD_GET_VERSION:
			{
				char version[7];
				sprintf(version, "v%u.%u", VERSION_MAJOR, VERSION_MINOR);
				uint8_t len = strlen(&version);
				if (len > 7) len = 7;
				prepareMsg(CMD_VERSION, (uint8_t*) version, len, PRIORITY_NORM);
			}
			break;

		case CMD_GET_STATE:
			booster_sendInfo();
			break;

		case 'm':
			debug("manual");
			mode = Mode_Manual;
			parse_data = parse_manual;
			break;

		case CMD_ROTATE:	// Old Command
			break;

		case CMD_SET_PULSE_WIDTH:	// Old Command
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
 * \param data The data of the received message
 * \param length The length of the received message
 */
void parse_manual(uint8_t *data, uint8_t length) {

	static uint16_t release_time = 200;
	uint16_t tmp = 0;
	uint8_t i;
	char str[8];

	// set the power of a shot
	// full power is round about 3000
	// a slow pass is about 800
	if (data[0] == 's') {
		for (i = 1; i < length; i++) {
			if (data[i] >= 0x30 && data[i] <= 0x39) {
				tmp = tmp * 10 + (data[i] - 0x30);
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
	else if (data[0] == ' ') {
		if (release_time > 0)
			kicker_addKickJob(release_time);
		debug("Release");
	}
	// enable auto boosting
	else if (data[0] == 'e') {
		mode = Mode_SoftwareControlled;
	}
	// force boosting. disable the software control
	// warning this may overload the capacitors if the
	// hardware disabling function fails
	else if (data[0] == 'w') {
		mode = Mode_Automatic;
	}
	// disable charging but holds the power
	else if (data[0] == 'q') {
		mode = Mode_Stop;
	}
	// switch back to AUTOMATIC MODE
	// (without the driver the rekick driver
	// (the one in c#), the system goes into standby mode)
	else if (data[0] == 'a') {
		mode = Mode_Automatic;
		parse_data = parse_default;
	}
	else {
		error("ERR IMPL");
	}

	return;
}
