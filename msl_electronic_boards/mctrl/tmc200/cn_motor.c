#include "cn_motor.h"
#include "cn_pwm.h"
#include "cn_adc.h"
#include "main.h"
#include "cc6.h"

sdword encoder_accu_rel[3];
sdword encoder_rel[3];

void initMotorSystem() {

  stopAllMotors();

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

void stopMotor(ubyte motor) {
  
  setMotorPWM(motor, ZERO_PWM);

}

void stopAllMotors() {
  CC6_SetCCxReg(CC6_CC_0, ZERO_PWM);
  CC6_SetCCxReg(CC6_CC_1, ZERO_PWM);
  CC6_SetCCxReg(CC6_CC_2, ZERO_PWM);

  CC6_CommitValue_CC6_TIMER_12();
}

void setMotorDirection(ubyte motor, bool direction) {
  
  if(direction != 0) {
  	direction = DIRECTION_CCW;
  } else {
  	direction = DIRECTION_CW;
  }
  
  switch(motor) {

    case MOTOR1:
	  IO_P1L_0 = direction;
	  break;
	case MOTOR2:
	  IO_P1L_2 = direction;
	  break;
	case MOTOR3:
	  IO_P1L_4 = direction;
	  break;
	default:
	  break;

  }

}

bool getMotorDirection(ubyte motor) {

  bool direction;
  
  switch(motor) {
    
    case MOTOR1:
	  direction = (bool) IO_P1L_0;
	  break;
	case MOTOR2:
	  direction = (bool) IO_P1L_2;
	  break;
    case MOTOR3:
	  direction = (bool) IO_P1L_4;
	  break;
	default:
	  direction = 0xFF; // ERROR
	  break;

  }
	if (direction == DIRECTION_CW) {
		direction = 0;
	} else {
		direction = 1;
	}
  return direction;
}


void toggleMotorDirection(ubyte motor) {

  bool direction;

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

}

void setAllMotorPWM(ureg pwm1, ureg pwm2, ureg pwm3) {

  if(pwm1 > MAX_PWM) pwm1 = MAX_PWM;
  if(pwm2 > MAX_PWM) pwm2 = MAX_PWM;
  if(pwm3 > MAX_PWM) pwm3 = MAX_PWM;

  CC6_SetCCxReg(CC6_CC_0, pwm1);
  CC6_SetCCxReg(CC6_CC_1, pwm2);
  CC6_SetCCxReg(CC6_CC_2, pwm3);

  CC6_CommitValue_CC6_TIMER_12();

}

ureg getMotorPWM(ubyte motor) {

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

}

sword getAndResetMotorTicks(ubyte motor) {

  sword tickdiff = 0;

  switch(motor) {
    case MOTOR1:
      tickdiff = GPT1_ReadTimer(GPT1_TIMER_2);
	  GPT1_LoadTimer(GPT1_TIMER_2, ENCODER_TICK_BIAS);    
	  break;
    case MOTOR2:
	  tickdiff = GPT1_ReadTimer(GPT1_TIMER_3);
	  GPT1_LoadTimer(GPT1_TIMER_3, ENCODER_TICK_BIAS);
      break;
	case MOTOR3:
	  tickdiff = GPT1_ReadTimer(GPT1_TIMER_4);
	  GPT1_LoadTimer(GPT1_TIMER_4, ENCODER_TICK_BIAS);
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
