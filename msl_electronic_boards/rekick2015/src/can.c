/*
 * can.c
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */

#include "can.h"

int8_t can2_init()
{
	SET_OUTPUT(CAN_TX);
	RESET(CAN_TX);
	SET_INPUT(CAN_RX);
	RESET(CAN_RX);

	can_init(0);

	// Interrupts aktivieren?


	tx_msg.pt_data = &tx_buffer[0];		// Point Remote Tx MOb to first element of buffer
	tx_msg.status = 0;					// clear status

	rx_msg.pt_data = &rx_buffer[0];		// Point Response MOb to first element of buffer
	rx_msg.status = 0;					// clear status

	configureRxMOb();
}

void can_handle()
{



					while(can_cmd(&response_msg) != CAN_CMD_ACCEPTED); // Wait for MOb to configure (Must re-configure MOb for every transaction)
}

void generate_extCAN_ID(uint8_t *bytes, char *resultchars) {

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

int8_t sendMsg(uint32_t id, uint8_t* data, length)
{
	tExtendedCAN message;

		// 0x00, priority, sender, receiver
		//uint8_t id[4] = {0x00, PRIORITY_NORM, REKICK_ID, ETH2CAN_ID};
		//generate_extCAN_ID(id, message.id);
		//	message.data[0] = cmd;
		//	memcpy(message.data+1, str, len);
		//	message.header.length = len+1;

		message.header.rtr = 0;

		// packet length is limited to 8 chars but one char
		// is already used for the cmd.
	if (length > 7)
		length = 7;


		// send to buffer
		can_send_message(&message);


	for(int i=0; i<DATA_BUFFER_SIZE; i++) {tx_buffer[i]=data[i];}		// fill MOb Data buffer
	tx_msg.id.ext = id;					// This message object only sends frames to Target IDs
	tx_msg.ctrl.ide = 1;				// This message object sends extended (2.0B) CAN frames
	tx_msg.ctrl.rtr = 0;				// This message object is sending Data
	tx_msg.dlc = DATA_BUFFER_SIZE;		// Number of data bytes (8 max)
	tx_msg.cmd = CMD_TX_DATA;

	while(can_cmd(&tx_msg) != CAN_CMD_ACCEPTED); // Wait for MOb to configure (Must re-configure MOb for every transaction) and send request

	while(can_get_status(&tx_msg) == CAN_STATUS_NOT_COMPLETED);	// Wait for Tx to complete

	// Error handling
}
