/*
 * can.h
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_CAN_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_CAN_H_

#include "can_lib.h"


#define DATA_BUFFER_SIZE 8

uint8_t tx_buffer[DATA_BUFFER_SIZE];
uint8_t rx_buffer[DATA_BUFFER_SIZE];	// buffer to hold payload from Sensor Nodes

st_cmd_t tx_msg; // Tx MOb (MOb to Tranceive Data)
st_cmd_t rx_msg; // Rx Mob (MOb for Receiving Data)


#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_CAN_H_ */
