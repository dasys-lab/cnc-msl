#include "main.h"
#include "asc0.h"
#include "io.h"
#include "cn_serial.h"
#include "cn_commands.h"
#include "cn_motor.h"
#include "cn_controller.h"
#include "cn_time.h"
#include <stdlib.h>
#include <stdio.h>

ubyte serial_buffer[SERIAL_BUFFER_SIZE];
uword serial_buffer_pos = 0; 

send_buffer snd_buffer[SERIAL_BUFFER_COUNT];
sbyte sending = -1;

void cn_serial_init(void) {
	int i;
	S0TBIC_S0TBIE = 0;
	for(i = 0; i < SERIAL_BUFFER_COUNT; ++i) {
	  snd_buffer[i].send_counter = 0;
	  snd_buffer[i].length = 0;
	  snd_buffer[i].empty = 1;
	}
	sending = 0;	
}


void cn_debug_serial(uword command) {

  sword pwm;

  unsigned static short int motor = MOTOR3;

  switch(command) {
    case 'a':
	  setMotorDirection(motor, DIRECTION_CCW);
	  break;
	case 's':
	  setMotorDirection(motor, DIRECTION_CW);
	  break;
    case '1':
      motor = MOTOR1;
	  break;
	case '2':
	  motor = MOTOR2;
	  break;
	case '3':
	  motor = MOTOR3;
	  break;
    case 'u': // PWM up
	  pwm  = getMotorPWM(motor);
	  pwm += 7;
	  setMotorPWM(motor, pwm);
	  break;
	case 'd': // PWM down
	  pwm  = getMotorPWM(motor);
	  pwm -= 7;
	  if(pwm < ZERO_PWM) pwm = ZERO_PWM;
	  setMotorPWM(motor, pwm);
	  break;
	case 't': // toggle direction
	  toggleMotorDirection(motor);
	  break;
	case ' ': // PWM off
	  stopAllMotors();
	  break;
    default:
	  break;
  }  

}
void cn_serial_tx_interrupt(void) {	   //called by interrupt
	sbyte cursend = sending;	
	S0TBUF = snd_buffer[sending].tosend[snd_buffer[sending].send_counter++];


	if (snd_buffer[sending].send_counter == snd_buffer[sending].length) {
		// current buffer is sent completely
		snd_buffer[sending].empty = 1;
		//snd_buffer[sending].send_counter = 0;
		//snd_buffer[sending].length = 0;
		
		do {		
			sending = (sbyte) ((sending+1)% SERIAL_BUFFER_COUNT);		
		} while(sending != cursend && snd_buffer[sending].empty!=0);
		if (sending == cursend) {
			S0TBIC_S0TBIE = 0;					
		}
	}
			
	S0TBIC_S0TBIR = 0;

}
void cn_serial_send_data(char* data, ubyte length) {
	sbyte buf;
	ubyte pos = 0;

	buf = (sbyte) ((sending+1) % SERIAL_BUFFER_COUNT);

	while(snd_buffer[buf].empty == 0) {
		buf = (sbyte)((buf+1) % SERIAL_BUFFER_COUNT);
	}
	
	while(pos < length) {
		snd_buffer[buf].tosend[pos]=*data;
		++data;
		++pos;
	}

	snd_buffer[buf].length = length;
	snd_buffer[buf].send_counter = 0;
	snd_buffer[buf].empty = 0;

	if (S0TBIC_S0TBIE == 0) { 
		sending = buf;
		snd_buffer[buf].send_counter = 1;
		// order is important!		
		S0TBUF = snd_buffer[buf].tosend[0];
		S0TBIC_S0TBIE = 1;				
	}
}

void cn_serial_rx_interrupt(void) { //called by interrupt

  //uword i;

  static uword cmd_state = CN_STATE_WAIT;

  uword c = ASC0_GetData();

  //cn_debug_serial(c);

  //printf("state: %d\n", cmd_state);
  //printf("char: %02x\n", c);

  switch ( cmd_state ) {

	case CN_STATE_WAIT :
				
			if (c == SERIAL_START) {

			  cn_serial_reset_buffer();

              cn_serial_store(c);

			  cmd_state = CN_STATE_RECEIVE;

			}

			break;

	case CN_STATE_RECEIVE :

	        // Check for special Characters			
    		switch ( c ) {
	 		  case SERIAL_START: 
					  cn_serial_reset_buffer(); // new Packet, new begin
					  cn_serial_store(c);		// Any unfinished Packet will be discarded
 					  break;                    

	 		  case SERIAL_END:	 

					  cn_serial_store(c);

					  // debug: print buffer
						/*
					  for(i = 0; i <serial_buffer_pos; ++i) {
					    printf("%02x", serial_buffer[i]);
					  } 
            printf(".\n");
						*/

					  cn_decode_command(serial_buffer, serial_buffer_pos);
					  					  
					  cmd_state = CN_STATE_WAIT;  // Wait for new Data
					  break;
	
	 		  case SERIAL_QUOTE:
					  cmd_state = CN_STATE_QUOTE; // next character will be read without checks
					  break;

			  // any non-special char will be stored
	 		  default:			 
			  		  cn_serial_store(c);
					  break;
					  	
		    }
						
			break;		


	// store char regardless of special meaning
    case CN_STATE_QUOTE : 

			cn_serial_store(c);

			cmd_state = CN_STATE_RECEIVE;

			break;

  }

  // check buffer fill
  if (serial_buffer_pos > SERIAL_BUFFER_SIZE) {
    
    // buffer is full but not a complete packet
	// ignore buffer and wait for next packet

	// debug: print buffer
	/*
	for(i = 0; i <serial_buffer_pos; ++i) {
	  printf("%02x", serial_buffer[i]);
	} 
  printf("\n");
	*/
    
	cmd_state = CN_STATE_WAIT;
  }
  
  //printf("state: %d\n", cmd_state);

}

__inline void cn_serial_store(uword c)	{

    serial_buffer[serial_buffer_pos] = (ubyte) c;
    serial_buffer_pos++;  

}

__inline void cn_serial_reset_buffer() {

    serial_buffer_pos = 0;

}

/*uchar cn_check_packet(uchar* inbuffer, uword length) {
  uword i;
  uchar	crc = 0;

  // only after start and before crc
  for(i = 1; i < (length - 2); ++i) {
    if(inbuffer[i] == SERIAL_QUOTE) ++i;
    crc += inbuffer[i];
  }

  // is crc correct?
  if(crc == inbuffer[i]) {
    return 1;
  } else {
    return 0;
  }
}*/


void cn_decode_command(ubyte* inbuffer, uword length) {

  uword  val_uword  = 0;
  uword  val_uword2 = 0;
  uword  val_uword3 = 0;
  udword val_udword	= 0;
  sword  val_sword  = 0;
  float  val_float  = 0.0;

  //printf("decode command\n");

  // min packet:
  // START, COUNTER, GROUP, CMD, CRC, END
  // NO COUNTER, NO CRC
  if(length < 4) return;

  //printf("length ok\n");

  // check crc
  // TODO: needs parameter for buffer
  //if(cn_check_packet() == 0) return;

  //printf("group: %02x, command: %02x\n", inbuffer[SERIAL_GROUP_POS], inbuffer[SERIAL_CMD_POS]);

  switch(inbuffer[SERIAL_GROUP_POS]) {

    case GROUP_CONFIGURE:
	  switch(inbuffer[SERIAL_CMD_POS]) {

    case CMD_SET_MODE:
      cn_command_configure_set_mode(&inbuffer[SERIAL_DATA_POS]);
      break;

    case CMD_SET_CYCLE_TIME:
      cn_command_configure_set_cycle_time(&inbuffer[SERIAL_DATA_POS]);
      break;

		case CMD_COMMAND_TIMEOUT:
		  cn_command_configure_command_timeout(inbuffer[SERIAL_DATA_POS]);
		  break;

		case CMD_MAX_CURRENT:
		  cn_command_configure_max_current(inbuffer[SERIAL_DATA_POS]);
		  break;

		case CMD_NOMINAL_CURRENT:
		  cn_command_configure_nom_current(inbuffer[SERIAL_DATA_POS]);
		  break;

		case CMD_MAX_RPM:
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
		  cn_command_configure_max_rpm(&val_uword);
		  break;

		case CMD_NOMINAL_RPM:
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
		  cn_command_configure_nom_rpm(val_uword);
		  break;

		case CMD_GEAR_RATIO:
		  cn_command_configure_gear_ratio(&inbuffer[SERIAL_DATA_POS], &inbuffer[SERIAL_DATA_POS+1]);
		  break;

		case CMD_WHEEL_RADIUS:
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
          cn_command_configure_wheel_radius(&val_uword);
          break;

		case CMD_TICKS_PER_ROTATION:
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
		  cn_command_configure_ticks_per_rotation(&val_uword);
		  break;

		case CMD_MOTOR_DIRECTION:
		  cn_command_configure_motor_direction(inbuffer[SERIAL_DATA_POS]);
		  break;

		case CMD_ROBOT_RADIUS:
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
		  cn_command_configure_robot_radius(&val_uword);
		  break;
		case CMD_TOGGLE_ODO_LOG:
		  cn_command_configure_toggle_odometry_log(inbuffer+SERIAL_DATA_POS);
		  break;		  

	    default:
		  cn_command_unknown();
		  break;
	  }
	  break;

  case GROUP_CONTROL:

	  switch(inbuffer[SERIAL_CMD_POS]) {

        case CMD_SET_ALL_PWM:
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
		  val_uword2 = *((uword*) (inbuffer + SERIAL_DATA_POS + sizeof(uword)));
		  val_uword3 = *((uword*) (inbuffer + SERIAL_DATA_POS + (sizeof(uword) * 2)));

      //printf("before Command: SetAllPWM(%d, %d, %d)\n", val_uword, val_uword2, val_uword3);

		  cn_command_control_set_all_pwm(val_uword, val_uword2, val_uword3);
		  cn_command_request_encoder_relative();
		  break;

		case CMD_SET_ALL_RPM:
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
		  val_uword2 = *((uword*) (inbuffer + SERIAL_DATA_POS + sizeof(uword)));
		  val_uword3 = *((uword*) (inbuffer + SERIAL_DATA_POS + (sizeof(uword) * 2)));
		  cn_command_control_set_all_rpm(val_uword, val_uword2, val_uword3);
		  cn_command_request_motor_rpm();
		  break;

		case CMD_SET_MOTION_VECTOR:
		  // speed
		  val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
		  // angle
		  val_uword2 = *((uword*) (inbuffer + SERIAL_DATA_POS + sizeof(uword)));
		  // rotation
		  val_uword3 = *((uword*) (inbuffer + SERIAL_DATA_POS + (sizeof(uword) * 2)));
		  cn_command_control_set_motion_vector(val_uword, val_uword2, val_uword3);
		  cn_command_request_path_vector();
		  break;

	    default:
		  cn_command_unknown();
		  break;
	  }
		
		// reset emergency stop timer since we received a new control message
		cn_reset_command_timeout();
		
/*
		if(inbuffer[(length - 1) + SERIAL_REQ_REL_POS] != 0x00) {
		
		  // this is quite a hack
			// but the packet should be decoded completely here anyway
		  inbuffer[SERIAL_CMD_POS] = inbuffer[(length - 1) + SERIAL_REQ_REL_POS];
		
		  goto forced_request_response;
		}
*/
		
	  break;

	case GROUP_REQUEST:
	
	//forced_request_response:
	
	  switch(inbuffer[SERIAL_CMD_POS]) {

    case CMD_MOTOR_RPM:
		  cn_command_request_motor_rpm();
		  break;

		case CMD_MOTOR_PWM:
		  //printf("before Command: RequestAllPWM()");
		  cn_command_request_motor_pwm();
		  break;
		  
		case CMD_MOTOR_CURRENT:
		  cn_command_request_motor_current();
		  break;

		case CMD_ENCODER_REL:
		  cn_command_request_encoder_relative();
		  break;

        case CMD_BATTERY_VOLTAGE:
		  //printf("before Command: RequestBatteryVoltage()");
		  cn_command_request_battery_voltage();
		  break;

        case CMD_MOTION_VECTOR:
		  cn_command_request_motion_vector();
		  break;

		case CMD_AVERAGE_SLEEP_TIME:
		  cn_command_request_average_sleep_time();
		  break;

        case CMD_PATH_VECTOR:
		  cn_command_request_path_vector();
		  break;

		case CMD_READ_ODO_LOG:
		 	val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_request_odometry_log(val_uword);
			break;
	    default:
		  cn_command_unknown();
		  break;
	  }
	  break;

	case GROUP_CONTROL_CONFIG:
	  switch(inbuffer[SERIAL_CMD_POS]) {

		case CMD_PID_KP:
			val_sword =  *((sword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_pid_kp(&val_sword);
			break;

		case CMD_PID_KI:
		    val_sword =  *((sword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_pid_ki(&val_sword);
			break;

		case CMD_PID_B:
			val_sword =  *((sword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_pid_b(&val_sword);
			break;

		case CMD_PID_KD:
			val_sword =  *((sword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_pid_kd(&val_sword);
			break;

		case CMD_PID_KDI:
			val_sword =  *((sword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_pid_kdi(&val_sword);
			break;

        case CMD_CONTROLLER_COMMIT:
		  cn_command_controlconf_controller_commit();
          break;

		case CMD_DEAD_BAND:
			val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_dead_band(&val_uword);
			break;

		case CMD_ROTATION_ERR_W:
			val_uword =  *((uword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_rot_err_w(&val_uword);
			break;

		case CMD_ROTATION_ERR_ACCEL_W:
			val_uword =  *((uword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_rot_err_accel_w(&val_uword);
			break;

		case CMD_ROTATION_ERR_VELO_W:
			val_uword =  *((uword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_rot_err_velo_w(&val_uword);
			break;

		case CMD_ACCEL_BOUND_CURVE_MIN:
			val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_accel_bound_curve_min(&val_uword);
			break;

		case CMD_ACCEL_BOUND_CURVE_MAX:
			val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_accel_bound_curve_max(&val_uword);
			break;
		
		case CMD_ACCEL_CURVE_DEGREE:
			val_float =  *((float*) (inbuffer + SERIAL_DATA_POS));
			cn_command_controlconf_accel_curve_degree(val_float);
			break;

		case CMD_FAIL_SAFE_VALUES:
			val_uword  = *((uword*) (inbuffer + SERIAL_DATA_POS));
			val_uword2 = *((uword*) (inbuffer + SERIAL_DATA_POS + sizeof(uword)));
			val_uword3 = *((uword*) (inbuffer + SERIAL_DATA_POS + (sizeof(uword) * 2)));
			cn_command_controlconf_fail_safe_values(&val_uword, &val_uword2, &val_uword3);
			break;

	    default:
		  cn_command_unknown();
		  break;
	  }
	  break;

    // unimplemented groups
    default:
	  cn_command_unknown();
	  break;
  }

}

void cn_write_response(ubyte group, ubyte cmd, ubyte* data, uword data_length) {
  
//  static uchar serial_packet_counter = 0;

  uchar  cbuffer[SERIAL_BUFFER_SIZE];
  uchar  obuffer[MAXPACKAGESIZE];
//  uchar  crc    = 0;
  uword  k;
  uword  i = 0, j;

  cbuffer[i++]  = SERIAL_START; // not in crc
//  cbuffer[i++]  = serial_packet_counter;
//  crc          += serial_packet_counter;
  cbuffer[i++]  = group;
//  crc          += group;
  cbuffer[i++]  = cmd;
//  crc          += cmd;

  for(j = 0; j < data_length; ++j) {
    cbuffer[i++]  = data[j];
//	crc          += data[j];
  }

//  cbuffer[i++] = crc;	   // not in crc
  cbuffer[i++] = SERIAL_END;  // not in crc

  //cn_write_quoted(cbuffer, i);
	k = cn_quote_data(cbuffer,obuffer,i);
	cn_serial_send_data(obuffer,k);
}
uword cn_quote_data(ubyte* inbuffer,ubyte* outbuffer, uword length) {

  uword i,j;
  j=1;
  outbuffer[0]=inbuffer[0];

  for(i = 1; i < length-1; ++i) {

    switch(inbuffer[i]) {

	  case SERIAL_START:
	  case SERIAL_END:
	  case SERIAL_QUOTE:
	    outbuffer[j++]=SERIAL_QUOTE;
	    break;

	  default:
	    // do nothing
	    break;

	}

	// write actual character
	outbuffer[j++]=inbuffer[i];

  }

  outbuffer[j++]=inbuffer[i];
  return j;
}

__inline void cn_write_quoted(ubyte* inbuffer, uword length) {

  uword i;

  cn_write_plain(inbuffer[0]);

  for(i = 1; i < length-1; ++i) {

    switch(inbuffer[i]) {

	  case SERIAL_START:
	  case SERIAL_END:
	  case SERIAL_QUOTE:
	    cn_write_plain(SERIAL_QUOTE);
	    break;

	  default:
	    // do nothing
	    break;

	}

	// write actual character
	cn_write_plain(inbuffer[i]);

  }

  cn_write_plain(inbuffer[length-1]);

}

__inline void cn_write_plain(ubyte c) {

  ASC0_TransmitData(c);

  cn_wait_tx_done();

}

__inline void cn_wait_tx_done() {
  while(ASC0_IsTransmitDone() == 0);
}
