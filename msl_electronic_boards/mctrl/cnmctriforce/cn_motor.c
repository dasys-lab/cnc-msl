#include "cn_motor.h"
#include "cn_pwm.h"
#include "cn_adc.h"
#include "main.h"
#include "cc6.h"

sdword encoder_accu_rel[3];
sdword encoder_rel[3];

void initMotorSystem() {

	stopAllMotors();
#ifdef VMC
	// init H-Bridge error pins
	dirP20 = 0;
	dirP21 = 0;
	dirP22 = 0;
	dirP23 = 0;
	dirP24 = 0;
	dirP25 = 0;
#endif
	getAndResetMotorTicks(MOTOR1);
	getAndResetMotorTicks(MOTOR2);
	getAndResetMotorTicks(MOTOR3);

	encoder_accu_rel[0] = 0;
	encoder_accu_rel[1] = 0;
	encoder_accu_rel[2] = 0;

	encoder_rel[0] = 0;
	encoder_rel[1] = 0;
	encoder_rel[2] = 0;

}

// reports the overtemperature flag of the H-Bridge
#ifdef VMC
__inline bool isOvertemp(ubyte motor) {

	bool flag;
	
	flag = 0;

	switch(motor) {
		case MOTOR1:
			flag =	(dir1_pwm0 == 1 && dir2_pwm0 == 1 && ERRA1 != ERRA2) ||
					(dir1_pwm0 == 1 && dir2_pwm0 == 0 && ERRA1 == 0 && ERRA2 == 1) ||
					(dir1_pwm0 == 0 && dir2_pwm0 == 1 && ERRA1 == 1 && ERRA2 == 0);
			break;
		case MOTOR2:
			flag =	(dir1_pwm1 == 1 && dir2_pwm1 == 1 && ERRB1 != ERRB2) ||
					(dir1_pwm1 == 1 && dir2_pwm1 == 0 && ERRB1 == 0 && ERRB2 == 1) ||
					(dir1_pwm1 == 0 && dir2_pwm1 == 1 && ERRB1 == 1 && ERRB2 == 0);
			break;
		case MOTOR3:
			flag =	(dir1_pwm2 == 1 && dir2_pwm2 == 1 && ERRC1 != ERRC2) ||
					(dir1_pwm2 == 1 && dir2_pwm2 == 0 && ERRC1 == 0 && ERRC2 == 1) ||
					(dir1_pwm2 == 0 && dir2_pwm2 == 1 && ERRC1 == 1 && ERRC2 == 0);
			break;
			
	}

	return flag;
}
#endif

void stopMotor(ubyte motor) {
  
  setMotorPWM(motor, ZERO_PWM);

}

void stopAllMotors() {
//#ifdef TMC
/*
	CC6_SetCCxReg(CC6_CC_0, ZERO_PWM);
	CC6_SetCCxReg(CC6_CC_1, ZERO_PWM);
	CC6_SetCCxReg(CC6_CC_2, ZERO_PWM);
*/
//#endif

	PWMREG1 = ZERO_PWM;
	PWMREG2 = ZERO_PWM;
	PWMREG3 = ZERO_PWM;
	
#ifdef TMC
	CC6_CommitValue_CC6_TIMER_12();
#endif

/*
#ifdef VMC
	PW0 = 0;
	PW1 = 0xFFFF;
	PW2 = 0x7FFF;
#endif
*/
}

void setMotorDirection(ubyte motor, sbyte direction) {
#ifdef VMC
	bool dir1;
	bool dir2;
#endif
	if(direction == DIRECTION_CCW) {
		direction = 1;
		#ifdef VMC
			dir1 = 1;
			dir2 = 0;
		#endif 
	} else if (direction == DIRECTION_CW) {
		direction = 0;
		#ifdef VMC
			dir1 = 0;
			dir2 = 1;
		#endif 		
	} else {
		#ifdef TMC
			return; // no valid direction
		#endif
		#ifdef VMC
			//direction = DIRECTION_FREE;
			dir1 = 0;
			dir2 = 0;
		#endif 		
	}

	switch(motor) {

		case MOTOR1:
#ifdef TMC
			IO_P1L_0 = direction;
#endif
#ifdef VMC
			dir1_pwm0 = dir1;
			dir2_pwm0 = dir2;
#endif
			break;
			
		case MOTOR2:
#ifdef TMC
			IO_P1L_2 = direction;
#endif
#ifdef VMC
			dir1_pwm1 = dir1;
			dir2_pwm1 = dir2;
#endif
			break;
			
		case MOTOR3:
#ifdef TMC
			IO_P1L_4 = direction;
#endif
#ifdef VMC
			dir1_pwm2 = dir1;
			dir2_pwm2 = dir2;
#endif
			break;
			
		default:
			break;
	}

}

/*void setMotorDirection(ubyte motor, bool direction) {

#ifdef VMC
	bool dir1;
	bool dir2;
#endif

	if(direction != 0) {
		direction = DIRECTION_CCW;
	} else {
		direction = DIRECTION_CW;
	}

#ifdef VMC
	if(direction == DIRECTION_CCW) {
		dir1 = 1;
		dir2 = 0;
	} else {
		dir1 = 0;
		dir2 = 1;
	}
#endif

	switch(motor) {

		case MOTOR1:
#ifdef TMC
			IO_P1L_0 = direction;
#endif
#ifdef VMC
			dir1_pwm0 = dir1;
			dir2_pwm0 = dir2;
#endif
			break;
			
		case MOTOR2:
#ifdef TMC
			IO_P1L_2 = direction;
#endif
#ifdef VMC
			dir1_pwm1 = dir1;
			dir2_pwm1 = dir2;
#endif
			break;
			
		case MOTOR3:
#ifdef TMC
			IO_P1L_4 = direction;
#endif
#ifdef VMC
			dir1_pwm2 = dir1;
			dir2_pwm2 = dir2;
#endif
			break;
			
		default:
			break;
	}

}
*/
sbyte getMotorDirection(ubyte motor) {

	sbyte direction;
	
#ifdef VMC
	bool dir1;
	bool dir2;
#endif

	direction = 0;
	
#ifdef VMC
	dir1 = dir2 = 0;
#endif

	switch(motor) {
    
		case MOTOR1:
#ifdef TMC
			direction = (sbyte) IO_P1L_0;
#endif
#ifdef VMC
			dir1 = dir1_pwm0;
			dir2 = dir2_pwm0;
#endif
		break;

		case MOTOR2:
#ifdef TMC
			direction = (sbyte) IO_P1L_2;
#endif
#ifdef VMC
			dir1 = dir1_pwm1;
			dir2 = dir2_pwm1;
#endif
		break;
		
		case MOTOR3:
#ifdef TMC
			direction = (sbyte) IO_P1L_4;
#endif
#ifdef VMC
			dir1 = dir1_pwm2;
			dir2 = dir2_pwm2;
#endif
		break;
		default:
			direction = 0x7F; // ERROR
		break;
  	}

#ifdef VMC
	if(dir1 == 1 && dir2 == 0) {
		direction = DIRECTION_CCW;
	} else if (dir1 == 0 && dir2 == 1) {
		direction = DIRECTION_CW;
	} else if (dir1 == 0 && dir2 == 0) {
		direction = DIRECTION_FREE;
	}
#endif

#ifdef TMC
	if (direction == 0) {
		direction = DIRECTION_CW;
	} else {
		direction = DIRECTION_CCW;
	}
#endif
	
	return direction;
}


void toggleMotorDirection(ubyte motor) {

  sbyte direction;

  direction = getMotorDirection(motor);

  switch(direction) {
    case DIRECTION_CW:
	  direction = DIRECTION_CCW;
	  break;
    case DIRECTION_CCW:
	  direction = DIRECTION_CW;
	  break;
    default:
	  break;
  }
  
  setMotorDirection(motor, direction);

}


void setMotorPWM(ubyte motor, ureg pwm) {

	ureg tmp;
	
	tmp = pwm = motor;

  /*
  if(pwm > MAX_PWM) pwm = MAX_PWM; // sanitize the value

  switch(motor) {
    case MOTOR1:
      CC6_SetCCxReg(CC6_CC_0, pwm);  
	  break;
	case MOTOR2:
	  CC6_SetCCxReg(CC6_CC_1, pwm);
	  break;
    case MOTOR3:
	  CC6_SetCCxReg(CC6_CC_2, pwm);
	  break;
  }

  CC6_CommitValue_CC6_TIMER_12();
  */

}


void setAllMotorPWM(ureg pwm1, ureg pwm2, ureg pwm3) {

  if(pwm1 > MAX_PWM) pwm1 = MAX_PWM;
  if(pwm2 > MAX_PWM) pwm2 = MAX_PWM;
  if(pwm3 > MAX_PWM) pwm3 = MAX_PWM;

/*
#ifdef TMC

	CC6_SetCCxReg(CC6_CC_0, pwm1);
	CC6_SetCCxReg(CC6_CC_0, pwm2);
	CC6_SetCCxReg(CC6_CC_0, pwm3);

#endif
*/

#ifdef VMC

	pwm1 = (ureg) ((((sdword) pwm1) - PWM_PERIOD) * (-1));
	pwm2 = (ureg) ((((sdword) pwm2) - PWM_PERIOD) * (-1));
	pwm3 = (ureg) ((((sdword) pwm3) - PWM_PERIOD) * (-1));

#endif

	PWMREG1 = pwm1;
	PWMREG2 = pwm2;
	PWMREG3 = pwm3;

#ifdef TMC
	CC6_CommitValue_CC6_TIMER_12();
#endif

}

ureg getMotorPWM(ubyte motor) {

	sdword value;

/*
#ifdef TMC
  switch(motor) {
    case MOTOR1:
	  return CC6_GetCCxReg_CC6_CC_0();
    case MOTOR2:
	  return CC6_GetCCxReg_CC6_CC_1();
	case MOTOR3:
	  return CC6_GetCCxReg_CC6_CC_2();
	default:
	  return 0;
  }
#endif
*/

	switch(motor) {
		case MOTOR1:
			value = PWMREG1;
			break;
		case MOTOR2:
			value = PWMREG2;
			break;
		case MOTOR3:
			value = PWMREG3;
			break;
		default:
			return 0;
	}

#ifdef VMC
	value = (value - PWM_PERIOD) * (-1);
#endif

	return (ureg) value;

}

sword getAndResetMotorTicks(ubyte motor) {

  sword tickdiff = 0;

  switch(motor) {
    case MOTOR1:
      tickdiff = GPT1_ReadTimer(MOTOR1_ENCODER);
	  GPT1_LoadTimer(MOTOR1_ENCODER, ENCODER_TICK_BIAS);    
	  break;
    case MOTOR2:
	  tickdiff = GPT1_ReadTimer(MOTOR2_ENCODER);
	  GPT1_LoadTimer(MOTOR2_ENCODER, ENCODER_TICK_BIAS);
      break;
	case MOTOR3:
	  tickdiff = GPT1_ReadTimer(MOTOR3_ENCODER);
	  GPT1_LoadTimer(MOTOR3_ENCODER, ENCODER_TICK_BIAS);
	  break;
    default:
	  break;
  }

  tickdiff = tickdiff - ENCODER_TICK_BIAS;
  
  //TODO change counter register config to do this implictly
  tickdiff *= -1;
  switch(motor) {
    case MOTOR1:
      encoder_accu_rel[0] += tickdiff;
      break;
    case MOTOR2:
      encoder_accu_rel[1] += tickdiff;
      break;
    case MOTOR3:
      encoder_accu_rel[2] += tickdiff;
      break;
    default:
      break;
}

  return tickdiff;
}
