/*
 * can.c
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */

#include "can.h"


#define CAN_MSG_BUFFER_LENGTH	5 // max 255

st_cmd_t can_buffer[CAN_MSG_BUFFER_LENGTH];
uint8_t can_buffer_head = 0;
uint8_t can_buffer_tail = 0;


// prototypes
void parse_default(uint8_t *data, uint8_t length);
void parse_manual(uint8_t *data, uint8_t length);

// globales
void (*parse_data)(uint8_t *, uint8_t) = parse_default;


int8_t communication_init()
{
	SET_OUTPUT(CAN_TX);
	RESET(CAN_TX);
	SET_INPUT(CAN_RX);
	RESET(CAN_RX);

	can_init(0);

	// Interrupts aktivieren?

	rx_msg.pt_data = &rx_buffer[0];		// Point Response MOb to first element of buffer
	rx_msg.status = 0;					// clear status
	// set filter ?

	configureRxMOb();
	

	tx_msg.pt_data = &tx_buffer[0];		// Point Remote Tx MOb to first element of buffer
	tx_msg.status = 0;					// clear status
}

void message_handler()
{
	// 0x00, priority, sender, receiver
	//uint8_t id[4] = {0x00, PRIORITY_NORM, REKICK_ID, ETH2CAN_ID};
	//generate_extCAN_ID(id, message.id);
	//	message.data[0] = cmd;
	//	memcpy(message.data+1, str, len);
}

void can_receive_handler()
{
	switch (can_get_status(&rx_msg)) {
		case CAN_STATUS_COMPLETED:
			// Message received
			if ((rx_msg.id.ext & 0x0000FF00) == (REKICK_ID << 8)) {
				// ID Filter
				parse_data(rx_msg.pt_data, rx_msg.dlc);
			}
			rx_msg.status = 0;	// clear status
			configureRxMOb();	// activate Rx MOb again
			break;
			
		case CAN_STATUS_NOT_COMPLETED:
			// waiting for Message
			break;
			
		case CAN_STATUS_ERROR:
			// CAN ERROR HANDLING
			rx_msg.cmd = CMD_ABORT;
			while (can_cmd(&rx_msg) != CAN_CMD_ACCEPTED);
			break;
			
		default:
			rx_msg.cmd = CMD_ABORT;
			while (can_cmd(&rx_msg) != CAN_CMD_ACCEPTED);
			rx_msg.status = 0;	// clear status
			configureRxMOb();	// activate Rx MOb again
			break;
	}
}

void generate_extCAN_ID(uint8_t *bytes, char *resultchars)
{
	resultchars[0] = (bytes[0] << 3) | (bytes[1] >> 5);
	resultchars[1] = (bytes[1] & 0x03) | 0x0C | ((bytes[1] & 0xFC) << 3);
	resultchars[2] = bytes[2];
	resultchars[3] = bytes[3];
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

int8_t sendMsg(uint8_t* data, length)
{
	if (length > 7)
		length = 7;

	for(int i=0; i<=length; i++)
		tx_buffer[i] = data[i];			// fill MOb Data buffer
	tx_msg.id.ext = id;					// This message object only sends frames to Target IDs
	tx_msg.ctrl.ide = 1;				// This message object sends extended (2.0B) CAN frames
	tx_msg.ctrl.rtr = 0;				// This message object is sending Data
	tx_msg.dlc = length + 1;			// Number of data bytes (8 max)
	tx_msg.cmd = CMD_TX_DATA;

	while(can_cmd(&tx_msg) != CAN_CMD_ACCEPTED); // Wait for MOb to configure (Must re-configure MOb for every transaction) and send request

	while(can_get_status(&tx_msg) == CAN_STATUS_NOT_COMPLETED);	// Wait for Tx to complete

	// Error handling
}

/**
 * Parser fuction for the default state.
 *
 * This is the parser for the normal operation.
 *
 * \param m The received message
 */
void parse_default(uint8_t *data, uint8_t length) {

	volatile uint16_t tmpa = 0;

	switch (data[0]) {
		case CMD_PING:
			last_heartbeat = timer_get_ms();
			can_put_cmd(CMD_PONG, NULL, 0);
			break;
		case CMD_KICK:
			if (timer_get_ms() > 1000) {
				tmpa = data[1] + (data[2]<<8);
				if (length == 3)
					kicker_add_kick_job(tmpa);
				else if (length == 4)
					kicker_add_kick_job_forced(tmpa, data[3]);
			}
			break;
		case CMD_SET_MAX_VOLTAGE:
			
			if (length == 3) {

				tmpa = data[1] + (data[2]<<8);
				booster_set_max_voltage(tmpa);

			} else if (length == 2) {
				tmpa = ((uint16_t)(data[1])) & 0xFF;
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
void parse_manual(uint8_t *data, uint8_t length) {

	static uint16_t release_time = 33;
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
			kicker_add_kick_job(release_time);
		debug("Release");
	}
	// enable auto boosting
	else if (data[0] == 'e') {
		auto_boost = true;
	}
	// force boosting. disable the software control
	// warning this may overload the capacitors if the
	// hardware disabling function fails
	else if (data[0] == 'w') {
		auto_boost = false;
		booster_pwm_enable();
	}
	// disable charging but holds the power
	else if (data[0] == 'q') {
		auto_boost = false;
		booster_pwm_disable();
	}
	// switch back to AUTOMATIC MODE
	// (without the driver the rekick driver
	// (the one in c#), the system goes into standby mode)
	else if (data[0] == 'a') {
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
