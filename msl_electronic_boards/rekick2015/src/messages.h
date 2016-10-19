/*
 * can.h
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_MESSAGES_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_MESSAGES_H_

#include "can_lib.h"

// CAN CMD - question
#define		CMD_PING			0x01
#define		CMD_ROTATE			0x02
#define 	CMD_KICK			0x03
#define		CMD_MANUAL			0x6D	// character 'm'

#define		CMD_SET_DEBUG_LEVEL	0x10
#define		CMD_SET_PULSE_WIDTH	0x30
#define		CMD_SET_MAX_VOLTAGE	0x31

#define		CMD_GET_STATE		0x40
#define		CMD_GET_VERSION		0x41

// CAN CMD - reply
#define		CMD_PONG			0xF1
#define		CMD_STATE			0xF2
#define		CMD_VERSION			0xF3
#define		CMD_RESET			0xF4	// send after a reset
#define 	CMD_ERROR			0x21	// char '!'
#define 	CMD_WARNING			0x22
#define		CMD_MSG				0x3E	// char '>'

// CAN Addresses
#define ETH2CAN_ID		0x00
#define MOTOR_ID		0x20
#define COMPASS_ID		0x40
#define REKICK_ID		0x60
#define SERVO_ID        0x80

// Priority levels
#define PRIORITY_HIGH	0x20
#define PRIORITY_NORM	0x40
#define PRIORITY_LOW	0x80


#define CAN_MSG_BUFFER_LENGTH	20 // max 255
#define DATA_BUFFER_SIZE 8

typedef struct
{
	uint32_t id;
	uint8_t data[8];
	uint8_t length;
} tExtendedCAN;

void communication_init();
void message_handler();
void message_receive_handler();
void message_transmit_handler();
void generate_extCAN_ID(uint8_t *bytes, uint32_t *result);
void configureRxMOb();
int8_t sendMsg(tExtendedCAN *message);
int8_t prepareMsg(uint8_t cmd, uint8_t* data, uint8_t length, uint8_t priority);
void split_message(uint8_t cmd, char *str);
void debug(char *str);
void warning(char *str);
void error(char *str);


#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_MESSAGES_H_ */
