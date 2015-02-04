#ifndef	DEFAULTS_H
#define	DEFAULTS_H

#define	P_MOSI			B,2
#define	P_MISO			B,3
#define	P_SCK			B,1

#define	MCP2515_CS		B,4
#define	MCP2515_INT		E,6
#define	MCP2515_RESET		A,0

#define MOTOR_LEFT_DIR		C,0
#define MOTOR_LEFT_ERROR_1	C,2
#define MOTOR_LEFT_ERROR_2	C,3
#define MOTOR_LEFT_RESET	C,1
#define MOTOR_LEFT_PWM		E,3

#define MOTOR_RIGHT_DIR		C,4
#define MOTOR_RIGHT_ERROR_1	C,6
#define MOTOR_RIGHT_ERROR_2	C,7
#define MOTOR_RIGHT_RESET	C,5
#define MOTOR_RIGHT_PWM		E,4

#define MOTOR_STOP_DIR		D,4
#define MOTOR_STOP_ERROR_1	D,6
#define MOTOR_STOP_ERROR_2	D,7
#define MOTOR_STOP_RESET	D,5
#define MOTOR_STOP_PWM		E,5

#define SERVO_PWM		B,5

#define LIGHT_BARRIER		D,1
#define MOTION_ON		F,0
#define VISION			F,2
#define VISION_LED		F,3
#define BUNDLE			F,4
#define BUNDLE_LED		F,5
#define BUTTON			F,6
#define BUTTON_LED		F,7

#define	OF_CS			E,2
#define	OF_RST			E,7
#define	OF_LED			B,6

#define SS			B,0

#endif	// DEFAULTS_H
