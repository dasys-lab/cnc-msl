#ifndef MESSAGES_H
#define MESSAGES_H

#define ETH2CAN_ID		0x00
#define MOTOR_ID		0x20
#define COMPASS_ID		0x40
#define REKICK_ID		0x60
#define SERVO_ID        0x80
#define SENSORBOARD_ID        0xA0

#define PRIORITY_HIGH	0x20
#define PRIORITY_NORM	0x40
#define PRIORITY_LOW	0x80

extern uint8_t manual_mode;

void debug(char *str);
void error(char *str);
void warning(char *str);
void can_put_cmd(uint8_t cmd, uint8_t* str, uint8_t len);
void can_init(void);
void can_test(void);
void can_send_handler(void);
void handle_error(void);

void message_handler(void);
void clear_receive_buffer(void);

// question
#define		CMD_PING			0x01
#define		CMD_SERVO			0x02
#define 	CMD_DRIBBLE			0x03
#define 	CMD_STOP			0x04
#define 	CMD_HAVE_BALL			0x05
#define 	CMD_VISION_STATE		0x06
#define 	CMD_BUNDLE_STATE		0x07
#define		CMD_MANUAL			0x6D	// character 'm'

#define		CMD_GET_STATE		0x40
#define		CMD_GET_VERSION		0x41

// reply
#define		CMD_PONG			0xF1
#define		CMD_VERSION			0xF2

#define 	CMD_ERROR			0x21	// char '!'
#define 	CMD_WARNING			0x22
#define		CMD_MSG				0x3E	// char '>'

#endif
