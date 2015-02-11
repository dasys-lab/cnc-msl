/** \file
 * Definitions for KURT2's firmware, including the assignment of the microcontroller ports.
 * $Id: $       
 */

/*
 * The contents of this file are subject to the Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *
 * The Original Code is KURT2 Open Source Firmware and Win32 Software,
 * released March 15, 2001.
 *
 * The Initial Developer of the Original Code is GMD National Research
 * Center for Information Technology.  Portions created by GMD are
 * Copyright (C) 2000 - 2001 GMD National Research Center for
 * Information Technology.  All Rights Reserved.
 * 
 * As of July 11, 2001, GMD has been integrated into the Fraunhofer-
 * Gesellschaft. As the new legal entity, Fraunhofer-Gesellschaft has thus
 * taken over all legal relationships involving GMD. Portions created by 
 * Fraunhofer-Gesellschaft are Copyright (C) 2002 Fraunhofer-Gesellschaft.
 * All Rights Reserved.
 * 
 * Contributor(s):
 */
 #include <reg167.h>

sbit p20   = P2^0;        /**< Pin 2.0: motor brake right.*/       
sbit dp20  = DP2^0;       /**< Direction register 2.0.*/

sbit p21   = P2^1;        /**< Pin 2.1: motor direction right.*/    
sbit dp21  = DP2^1;       /**< Direction register 2.1.*/

sbit p22   = P2^2;        /**< Pin 2.2: motor brake left.*/
sbit dp22  = DP2^2;       /**< Direction register 2.2.*/

sbit p23   = P2^3;        /**< Pin 2.3: motor direction left.*/ 
sbit dp23  = DP2^3;       /**< Direction register 2.3.*/

sbit p28   = P2^8;        /**< Pin 2.8: red LED.*/
sbit dp28  = DP2^8;       /**< Direction register 2.8.*/

sbit p29   = P2^9;        /**< Pin 2.9: actuator on/off.*/
sbit dp29  = DP2^9;       /**< Direction register 2.9.*/

sbit p210  = P2^10;       /**< Pin 2.10: bumper 0.*/
sbit dp210 = DP2^10;      /**< Direction register 2.10.*/

sbit p211  = P2^11;       /**< Pin 2.11: bumper 1.*/   
sbit dp211 = DP2^11;      /**< Direction register register 2.11.*/

sbit p212  = P2^12;       /**< Pin 2.12: bumper 2.*/     
sbit dp212 = DP2^12;      /**< Direction register 2.12.*/

sbit p213  = P2^13;       /**< Pin 2.13: bumper 3.*/  
sbit dp213 = DP2^13;      /**< Direction register 2.13.*/

sbit p214  = P2^14;       /**< Pin 2.14: bumper 4.*/
sbit dp214 = DP2^14;      /**< Direction register 2.14*/

sbit p215  = P2^15;       /**< Pin 2.15: bumper 5.*/   
sbit dp215 = DP2^15;      /**< Direction register 2.15.*/         

sbit p32   = P3^2;        /**< Pin 3.2: CAPIN for SPI-interrupt.*/
sbit dp32  = DP3^2;       /**< Direction register 3.2.*/ 

sbit p34   = P3^4;        /**< Pin 3.4: incremental encoding right.*/ 
sbit dp34  = DP3^4;       /**< Direction register 3.4.*/

sbit p36   = P3^6;        /**< Pin 3.6: motor encoder right.*/      
sbit dp36  = DP3^6;       /**< Direction register 3.6.*/

sbit p37   = P3^7;        /**< Pin 3.7: motor encoder left.*/    
sbit dp37  = DP3^7;       /**< Direction register 3.7.*/

sbit p310  = P3^10;       /**< Pin 3.10: serial interface TXD.*/    
sbit dp310 = DP3^10;      /**< Direction register 3.10.*/

sbit p311  = P3^11;       /**< Pin 3.11: serial interface RXD.*/  
sbit dp311 = DP3^11;      /**< Direction register 3.11.*/

/**
 * P5DIDIS is not yet included in reg167.\ h.
 * \todo Get an updated version of the Keil development environment.
 *
 * Analog input channels (port 5) of the first (or only) microcontroller:
 *
 * P5.0 : distance sensor 0 (back center).
 *
 * P5.1 : distance sensor 1 (back right edge).
 *
 * P5.2 : distance sensor 2 (right back).
 *
 * P5.3 : distance sensor 3 (right front).
 *
 * P5.4 : distance sensor 4 (front right edge).
 *
 * P5.5 : distance sensor 5 (front center).
 *
 * P5.6 : distance sensor 6 (front left edge).
 *
 * P5.7 : distance sensor 7 (left front).
 *
 * P5.8 : distance sensor 8 (left back).
 *
 * P5.9 : distance sensor 9 (back left edge).
 *
 * P5.10: optional sensor.
 *
 * P5.11: optional sensor.
 *
 * P5.12: motor current right.
 *
 * P5.13: motor current left.
 *
 * P5.14: optional sensor.
 *
 * P5.15: incremental encoding left (alternate function of this pin).
 *
 * For the optional second microcontroller, there is no fixed assignment of the analog channels.
 */
sfr P5DIDIS = 0xFFA4;

sbit p70   = P7^0;        /**< Pin 7.0: PWM output channel right.*/       
sbit dp70  = DP7^0;       /**< Direction register 7.0.*/
sbit p71   = P7^1;        /**< Pin 7.1: PWM output channel left.*/               
sbit dp71  = DP7^1;       /**< Direction register 7.1.*/
sbit p80   = P8^0;        /**< Pin 8.0: remote control channel 0.*/        
sbit dp80  = DP8^0;       /**< Direction register 8.0.*/
sbit p81   = P8^1;        /**< Pin 8.1: remote control channel 1.*/        
sbit dp81  = DP8^1;       /**< Direction register 8.1.*/
sbit p82   = P8^2;        /**< Pin 8.2: remote control channel 2.*/        
sbit dp82  = DP8^2;       /**< Direction register 8.2.*/
sbit p83   = P8^3;        /**< Pin 8.3: remote control channel 3.*/        
sbit dp83  = DP8^3;       /**< Direction register 8.3.*/
sbit p84   = P8^4;        /**< Pin 8.4: remote control channel 4.*/        
sbit dp84  = DP8^4;       /**< Direction register 8.4.*/
sbit p85   = P8^5;        /**< Pin 8.5: remote control channel 5.*/        
sbit dp85  = DP8^5;       /**< Direction register 8.5.*/         
sbit p86   = P8^6;        /**< Pin 8.6: remote control channel 6.*/        
sbit dp86  = DP8^6;       /**< Direction register 8.6.*/
sbit p87   = P8^7;        /**< Pin 8.7: remote control channel 7.*/         
sbit dp87  = DP8^7;       /**< Direction register 8.7.*/     

/* the GROUP addresse    
*/

#define _CMDGRP_MOTOR_CONFIN_ 		0x50	
#define _CMDGRP_MOTOR_CONFOUT_ 		0x51
#define _CMDGRP_MOTOR_CTRL_ 		0x52
//unused
#define _CMDGRP_MOTOR_STATUSIN_ 	0x54
#define _CMDGRP_MOTOR_STATUSOUT_ 	0x55
#define _CMDGRP_CONTROLLER_PARIN_ 	0x56
#define _CMDGRP_CONTROLLER_PAROUT_ 	0x57
//unused
#define _CMDGRP_MOTOR_ERR_ 			0x59		// err, emergency stop, bumper

/*@}*/

// the new CAN-IDs

/** \define VMC-GROUP 2 CANIds >> CAN identifiers used by VMC's firmware
 */
/*@{*/
#define USER_ADR_SPACE				0x07c0
#define VMC_RX_CAN_ID					0x07f0
#define VMC_TX_CAN_ID					0x07e0
#define _CAN_GROUP_MASK_			0x000f

#define _CAN_MOTOR_CONFIN_		VMC_RX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_MOTOR_CONFIN_ )
#define _CAN_MOTOR_CONFOUT_		VMC_TX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_MOTOR_CONFOUT_ )		//send
#define _CAN_MOTOR_CTRL_		VMC_RX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_MOTOR_CTRL_ )
#define _CAN_MOTOR_STATUSIN_	VMC_RX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_MOTOR_STATUSIN_ )
#define _CAN_MOTOR_STATUSOUT_	VMC_TX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_MOTOR_STATUSOUT_ )		// send
#define _CAN_CONTROLLER_PARIN_	VMC_RX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_CONTROLLER_PARIN_ )
#define _CAN_CONTROLLER_PAROUT_	VMC_TX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_CONTROLLER_PAROUT_ )	// send
#define _CAN_MOTOR_ERR_			VMC_TX_CAN_ID |( _CAN_GROUP_MASK_ & _CMDGRP_MOTOR_ERR_ )			//send

/*@}*/

/** \defgroup CANmobjs Mapping of CAN identifiers to C167 message objects
 */
/*@{*/
#define _MSG_MOTOR_CONFIN_		1
#define _MSG_MOTOR_CONFOUT_		2	// send
#define _MSG_MOTOR_CTRL_		3
#define _MSG_MOTOR_STATUSIN_	4
#define _MSG_MOTOR_STATUSOUT_	5	// send
#define _MSG_CONTROLLER_PARIN_	6
#define _MSG_CONTROLLER_PAROUT_	7	// send
#define _MSG_MOTOR_ERR_			8	// send








/** \defgroup CANIds CAN identifiers used by KURT2's firmware
 */
/*@{*/
#define CAN_CONTROL   0x1 /**< Control message.*/
#define CAN_INFO      0x4 /**< Firmware information.*/
#define CAN_ADC00_03  0x5 /**< Analog input channels 0 - 3.*/
#define CAN_ADC04_07  0x6 /**< Analog input channels 4 - 7.*/
#define CAN_ADC08_11  0x7 /**< Analog input channels 8 - 11.*/
#define CAN_ADC12_15  0x8 /**< Analog input channels 12 - 14, Temp.*/
#define CAN_ENCODER   0x9 /**< 2 motor encoders.*/ 
#define CAN_BUMPERC   0xA /**< 6 bumpers, 8 remote control buttons.*/
#define CAN_POSITION  0xB /**< X and Y coordinates, orientation.*/
#define CAN_ENC_ODO   0xC /**< Acumulated encoder values.*/ 
#define CAN_TILT_COMP 0xD /**< Tilt and compass data.*/ 
#define CAN_GYRO      0xE /**< Data from gyroscope-module.*/ 
/*@}*/

/** \defgroup CANmobjs Mapping of CAN identifiers to C167 message objects
 */
/*@{*/
#define MSG_CONTROL     1 /**< Control message.*/
#define MSG_INFO        4 /**< Firmware information.*/
#define MSG_ADC00_03    5 /**< Analog input channels 0 - 3.*/
#define MSG_ADC04_07    6 /**< Analog input channels 4 - 7.*/
#define MSG_ADC08_11    7 /**< Analog input channels 8 - 11.*/
#define MSG_ADC12_15    8 /**< Analog input channels 12 - 14, Temp.*/
#define MSG_ENCODER     9 /**< 2 motor encoders.*/ 
#define MSG_BUMPERC    10 /**< 6 bumpers, 8 remote control buttons.*/
#define MSG_POSITION   11 /**< X and Y coordinates, orientation.*/
#define MSG_ENC_ODO    12 /**< Acumulated encoder values.*/ 
#define MSG_TILT_COMP  13 /**< Tilt and compass data.*/ 
#define MSG_GYRO       14 /**< Data from gyroscope-module.*/ 
/*@}*/

/** \defgroup ControlMode Definition of control mode's values
 * Control mode is used to specify the meaning of a control message.
 * It is transmitted as the first two data bytes of the message with CAN identifier 1. 
 */
/*@{*/
#define RAW             0x0000 /**< Raw control mode.*/
#define SPEED           0x0001 /**< Wheel speed control mode in percent.*/
#define ACTUATOR_ON     0xFF00 /**< Switch actuator on.*/
#define ACTUATOR_OFF    0xFF01 /**< Switch actuator off.*/
#define CALIBRATE_GYRO  0xFF02 /**< Recalibrate gyroscope.*/
#define RESET_GYRO      0xFF03 /**< Reset gyroscope.*/
#define RESET_SSC       0xFF04 /**< Reset SSC-Interface (SPI).*/
#define UPDATE_ANGLE    0xFF05 /**< Update odometry orientation with new angle.*/
#define MC_RESET        0xFFFF /**< Submit software reset.*/
/*@}*/

/**
 * Baud rate of CAN bus (kBaud).
 */
#define CAN_BAUD 1000

/**
 * Timer resolution for global clock.
 * 39 * 25,6 usec ~= 1 msec
 */
#define MSEC 39L

/**
 * Resolution of pulse width modulation.
 */
#define PWM_PERIOD 1024

/**
 * Cycle time for main control loop including encoder readings.
 * 1953 * 51,2 usec ~= 100 msec.
 * 195  * 51,2 usec ~=  10 msec.
 */
#define ENC_TIME 195

/**
 * Maximum speed.
 * Number of encoder ticks per 100 msec at maximum speed with standard drive.
 */
#define SPEED_MAX 4000L

/**
 * Slow speed.
 * Speed that is slow enough to change direction.
 */
#define SLOW_SPEED 120

/**
 * Proportional gain for new speed controller.
 * Critical kp-value of left side from which kp is derived.
 */
#define GAIN_CRIT_L 700.0

/**
 * Proportional gain for new speed controller.
 * Critical kp-value of right side from which kp is derived.
 */
#define GAIN_CRIT_R 700.0

/**
 * Integral gain for new speed controller.
 * Period of left side's oscillation when using the critical kp-value.
 */
#define TIME_CRIT_L 190000.0

/**
 * Integral gain for new speed controller.
 * Period of right side's oscillation when using the critical kp-value.
 */
#define TIME_CRIT_R 190000.0

/**
 * Typedef for clarity in which form a value is.
 */
#define fixpoint_t  signed long

/**
 * Typedef for clarity in which form a value is.
 */
#define fixpoint2_t signed long

/**
 * Conversion to FIXPOINT.
 */
#define FIXPOINT    1000000L

/**
 * Conversion to FIXPOINT2.
 */
#define FIXPOINT2   100000000L

/**
 * Conversion from FIXPOINT to FIXPOINT2.
 */
#define FIXPOINTTOFIXPOINT2 100L

/**
 * Pi in FIXPOINT2 representation.
 */
#define ODO_PI      314159265L

/**
 * Conversion from encoder ticks to FIXPOINT mm.
 */
#define tic_length_left  17924L

/**
 * Conversion from encoder ticks to FIXPOINT mm.
 */
#define tic_length_right 17924L

/**
 * Width for odometric calculations.
 * This is a "virtual" width, depending on certain assumptions for modelling.
 */
#define width_odo   385L 

/**
 * Width of KURT2.
 * The actual width is 280mm, but an empirical found correction factor of 1,785 is needed.
 */
#define width_kurt  500L

extern int can_init(int baud_rate, int mc_no);
extern int can_send(char mobj);
extern int can_receive(char mobj);

extern void behave(void);

extern void pause(long maxcount);
extern unsigned long get_time (void);

/**
 * Measured values of all sensors.
 */
struct sensor_state {
  unsigned short sonar[10];               /**< Distance sensors.*/
  unsigned short tilt[2];                 /**< Tilt sensor.*/
  unsigned short cur_left;                /**< Current sensor left.*/
  unsigned short cur_right;               /**< Current sensor right.*/
  unsigned short compass[2];              /**< Compass sensor.*/
  unsigned char  bumpers;                 /**< Bumpers.*/
  unsigned char  rc;                      /**< Remote control.*/
  unsigned short adc_10;                  /**< Optional analog sensor at P5.10.*/
  unsigned short adc_11;                  /**< Optional analog sensor at P5.11.*/
  unsigned short adc_14;                  /**< Optional analog sensor at P5.14.*/
  unsigned short adcoptmc[16];            /**< Analog sensors of second microcontroller.*/
  signed short   temp;			          /**< Temperature sensor */
};

/**
 * Measured or computed values of odometry.
 */
struct odometry_state {
  signed long  enc_left;                  /**< Wheel encoder left.*/
  signed long  enc_right;                 /**< Wheel encoder right.*/
  signed long  odo_left;                  /**< Accumulated wheel encoder left.*/
  signed long  odo_right;                 /**< Accumulated wheel encoder right.*/
  signed int   speed_left;                /**< Speed left.*/
  signed int   speed_right;               /**< Speed right.*/
  signed int   speed_kurt;                /**< Translational speed.*/
  signed int   rotation_speed_kurt;       /**< Rotational speed.*/     
  signed long  position_x;                /**< X-position in cartesian coordinates.*/
  signed long  position_y;                /**< Y-position in cartesian coordinates.*/
  signed int   orientation;               /**< Orientation of the robot.*/
};

/**
 * Parameters for motor control.
 */
struct motor_ctrl {
  unsigned short control_mode;            /**< Meaning of the last command.*/
  unsigned char  dir_left;                /**< Direction left.*/
  unsigned char  dir_right;               /**< Direction right.*/
  unsigned char  brake_left;              /**< Brake left.*/
  unsigned char  brake_right;             /**< Brake right.*/
    signed short pwm_left;                /**< PWM value left.*/     
    signed short pwm_right;               /**< PWM value right.*/     
    signed int   speed_left;              /**< Target speed left.*/
    signed int   speed_right;             /**< Target speed right.*/
    signed int   speed_rot;               /**< Target speed rotational.*/
    signed int   speed_trans;             /**< Target speed translational.*/
};

/**
 * Definitions and declarations for gyro.
 * \todo Integrate gyro definitions properly.
 */
#define rxintno 0x2E                      /**< hardware interrupt # of SSC receive. */
#define txintno 0x2D                      /**< hardware interrupt # of SSC transmit. */

#define fCPU 20000000                     /**< system clock frequency. */
#define BAUDssc 1000000                   /**< SSC baud rate, 115200 is approx. 8.5us bit clock. */
#define RELOAD_VALUE (fCPU/(2*BAUDssc)) - 1  /**< Reload value for SSCBR. */
#define CYCLE 10
#define C_FACTOR 10000                    /**< factor required for using integer values. */

extern unsigned char sdata TX_BUFFER [6]; /**< max. 6 bytes allowed for transmit buffer. */
extern unsigned char sdata RX_BUFFER [6]; /**< max. 6 bytes allowed for receive  buffer. */

extern bit SILENT;
extern bit CONFIG;
extern bit IMU;

extern signed long updated_angle;
extern signed long R_odo;
extern struct sensor_state kurt2_state;



