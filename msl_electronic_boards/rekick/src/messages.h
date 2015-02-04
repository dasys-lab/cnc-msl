#ifndef MESSAGES_H
#define MESSAGES_H

#define ETH2CAN_ID		0x00
#define MOTOR_ID		0x20
#define COMPASS_ID		0x40
#define REKICK_ID		0x60
#define SERVO_ID        0x80

#define PRIORITY_HIGH	0x20
#define PRIORITY_NORM	0x40
#define PRIORITY_LOW	0x80

void debug(char *str);
void error(char *str);
void warning(char *str);
void can_put_cmd(uint8_t cmd, uint8_t* str, uint8_t len);
void can_init(void);
void can_test(void);
void can_send_handler(void);

void message_handler(void);
void clear_receive_buffer(void);

// question
#define		CMD_PING			0x01
#define		CMD_ROTATE			0x02
#define 	CMD_KICK			0x03
#define		CMD_MANUAL			0x6D	// character 'm'

#define		CMD_SET_DEBUG_LEVEL	0x10
#define		CMD_SET_PULSE_WIDTH	0x30
#define		CMD_SET_MAX_VOLTAGE	0x31

#define		CMD_GET_STATE		0x40
#define		CMD_GET_VERSION		0x41

// reply
#define		CMD_PONG			0xF1
#define		CMD_STATE			0xF2
#define		CMD_VERSION			0xF3
#define		CMD_RESET			0xF4	// send after a reset
#define 	CMD_ERROR			0x21	// char '!'
#define 	CMD_WARNING			0x22
#define		CMD_MSG				0x3E	// char '>'

#endif
