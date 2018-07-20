#include "cn_serial.h"
#include "cn_commands.h"
#include "cn_controller.h"
#include "cn_time.h"
#include "cn_motor.h"
#include "cn_adc.h"
#include "cn_current.h"
#include "cn_force_lookup.h"
#include <limits.h>
#include <stdio.h>



float dummy = 0;

void cn_command_unknown(void) {
  cn_write_response(GROUP_ERROR_RESPONSE,CMD_UNKNOWN_CMD,0x00,0);  
  //printf("unknown group or command\n");
}

// ---------------------------------------------------------

void cn_command_configure_set_mode(ubyte* mode) {
	if (*mode >= CONTROLLER_MODE_LOWEST && *mode <= CONTROLLER_MODE_HIGHEST) {
		controller_mode = *mode;
		cn_write_response(GROUP_CONFIGURE_RESPONSE, CMD_SET_MODE, mode, 1);
	} else {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE,mode,1);
	}
}

void cn_command_configure_set_cycle_time(ubyte* milsec) {
	if (*milsec == 0) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE,milsec,1); 
	}
	else {			// calculate ticks
					//+-------------+ microsec (long!)			
		cycle_time = (uword) (*milsec * TIME_TICKS_PER_MILLISECOND);
		cn_write_response(GROUP_CONFIGURE_RESPONSE, CMD_SET_CYCLE_TIME, milsec, 1);
	}
}

void cn_command_configure_command_timeout(ubyte milsec) {

  unsigned long timeout = milsec * TIME_TICKS_PER_MILLISECOND;

  if(timeout > INT_MAX || timeout < cycle_time) {
    //cn_response_error_out_of_range();
  }	else {
    cn_set_command_timeout((uword) timeout);
  }

}
									  // dezi ampere
void cn_command_configure_max_current(ubyte current) {
  cn_controller_current_max = current * 10;
}
									  // dezi ampere
void cn_command_configure_nom_current(ubyte current) {
  dummy = current;
}

void cn_command_configure_max_rpm(uword* rpm) {
	if (*rpm < 1) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE, (ubyte*)rpm, sizeof(uword));	
	} else {
		controller_motor_params.maxRPM = *rpm;
		cn_write_response(GROUP_CONFIGURE_RESPONSE, CMD_MAX_RPM, (ubyte*)rpm, sizeof(uword));
	}

}

void cn_command_configure_nom_rpm(uword rpm) {
  dummy = rpm;
}
									 
void cn_command_configure_gear_ratio(ubyte* numerator, ubyte* denumerator) {
	uword answer = (*numerator) | (*denumerator<< 8); //erm ...there might be an endian issue here
	if (*numerator== 0) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE, numerator, 1);
	}
	else if (*denumerator == 0) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE, denumerator, 1);
	} else {
		controller_motor_params.gear_numerator=*numerator;
		controller_motor_params.gear_denumerator=*denumerator;
		cn_controller_recalc_derived_motor_params();
		cn_write_response(GROUP_CONFIGURE_RESPONSE, CMD_GEAR_RATIO, (ubyte*)&answer, 2);
	}
}
									  // dezi millimeter
void cn_command_configure_wheel_radius(uword* radius) {
	if (*radius == 0) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE,(ubyte*) radius, 2);
	} else {
		controller_motor_params.wheel_radius = *radius;
		cn_controller_recalc_derived_motor_params();
		cn_write_response(GROUP_CONFIGURE_RESPONSE, CMD_WHEEL_RADIUS, (ubyte*)radius, 2); 		  		
	}
}

void cn_command_configure_ticks_per_rotation(uword* ticks) {
  if (*ticks == 0) {
  	cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE, (ubyte*)ticks, 2);
  } else {
  	controller_motor_params.encoder_ticks = *ticks;
	cn_controller_recalc_derived_motor_params();
	cn_write_response(GROUP_CONFIGURE_RESPONSE, CMD_TICKS_PER_ROTATION, (ubyte*)ticks, 2);

  }
}

void cn_command_configure_motor_direction(ubyte bitfield) {

  bool m1 = (bool) (bitfield & 0x1);
  bool m2 = (bool) (bitfield & 0x2);
  bool m3 = (bool) (bitfield & 0x4);

}
									   // millimeter
void cn_command_configure_robot_radius(uword* radius) {
	if (*radius == 0) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE,(ubyte*) radius, 2);
	} else {
		controller_motor_params.robot_radius = *radius;
		cn_controller_recalc_derived_motor_params();
		cn_write_response(GROUP_CONFIGURE_RESPONSE, CMD_ROBOT_RADIUS, (ubyte*)radius, 2); 		  		
	}
}

void cn_command_configure_toggle_odometry_log(ubyte* toggle) {
	if (*toggle==0x00) controller_log_mode.log_enabled = 0;
	else controller_log_mode.log_enabled = 1;
	cn_write_response(GROUP_CONFIGURE_RESPONSE,CMD_TOGGLE_ODO_LOG,toggle,1);
}

void cn_command_configure_controller_log(sbyte* data) {
	sbyte* pos = data+1;
	if ((*data > LOG_ModeMax) || (*pos > 2) || (*pos < -1) || (*data < 0)) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE,0x00, 0);
	} else {
		controller_log_mode.log[*data]=(sbyte)((*pos)*3);
		cn_write_response(GROUP_CONFIGURE_RESPONSE,CMD_SET_LOG_MODE,data,sizeof(sbyte)*2);
	}
}
// ------------------------------------------------------------

void cn_command_control_set_all_pwm(sword val1, sword val2, sword val3) {
  
	bool dir1, dir2, dir3;
	ureg pwm1, pwm2, pwm3;

	//printf("inside Command: SetAllPWM(%d, %d, %d)\n", val1, val2, val3);

	if(val1 < 0) { dir1 = DIRECTION_CW; val1 *= -1; } else { 
		#ifdef VMC
			if (val1==0) {
				dir1=DIRECTION_FREE;
			} else
		#endif		
			dir1 = DIRECTION_CCW;
	}
	if(val2 < 0) { dir2 = DIRECTION_CW; val2 *= -1; } else { 
		#ifdef VMC
			if (val2==0) {
				dir2=DIRECTION_FREE;
			} else
		#endif
			dir2 = DIRECTION_CCW;
	}
	if(val3 < 0) { dir3 = DIRECTION_CW; val3 *= -1; } else {
		#ifdef VMC
			if (val3==0) {
				dir3=DIRECTION_FREE;
			} else
		#endif
		dir3 = DIRECTION_CCW;
	}

	//val1 &= 0xEFFF;
	//val2 &= 0xEFFF;
	//val3 &= 0xEFFF;

	if(val1 > MAX_PWM_RANGE) val1 = MAX_PWM_RANGE;
	if(val2 > MAX_PWM_RANGE) val2 = MAX_PWM_RANGE;
	if(val3 > MAX_PWM_RANGE) val3 = MAX_PWM_RANGE;

	pwm1 = (uword) ((((udword) MAX_PWM) * ((uword) val1)) / (udword) MAX_PWM_RANGE);
	pwm2 = (uword) ((((udword) MAX_PWM) * ((uword) val2)) / (udword) MAX_PWM_RANGE);
	pwm3 = (uword) ((((udword) MAX_PWM) * ((uword) val3)) / (udword) MAX_PWM_RANGE);

	controller_direct_pwm_target.dir[0] = dir1;
	controller_direct_pwm_target.dir[1] = dir2;
	controller_direct_pwm_target.dir[2] = dir3;
	controller_direct_pwm_target.pwm[0] = pwm1;
	controller_direct_pwm_target.pwm[1] = pwm2;
	controller_direct_pwm_target.pwm[2] = pwm3;

}

void cn_command_control_set_all_rpm(sword rpm1, sword rpm2, sword rpm3) {
  dummy = rpm1;
  dummy = rpm2;
  dummy = rpm3;
}

void cn_command_control_set_motion_vector(sword x, sword y, sword rotation) {
	sdword rot	= ((sdword)CLAMP(rotation,-3220,3220))*(ROTATION_PRECISION/64);
	x			= CLAMP(x,-10000,10000);
	y			= CLAMP(y,-10000,10000);

	controller_stat.motion_request.x=x;
	controller_stat.motion_request.y=y;
	controller_stat.motion_request.rotation=rot;
}

void cn_command_control_reset_hard() {
  
  // software reset the chip
  // configured to do a real hardware reset
  __asm {
    SRST;
  }
}

void cn_command_control_reset_controller() {
}

// ------------------------------------------------------------------

void cn_command_request_motor_current() {

	uword data[3];
  
	data[0] = motor_current[0];
	data[1] = motor_current[1];
	data[2] = motor_current[2];

	cn_write_response(GROUP_REQUEST_RESPONSE, CMD_MOTOR_CURRENT, (ubyte*) data, sizeof(uword) * 3);

}

void cn_command_request_motor_rpm() {}

void cn_command_request_motor_pwm() {

  sword data[3];

  //printf("inside Command: RequestMotorPWM():%d %d %d\n", getMotorPWM(MOTOR1), getMotorPWM(MOTOR2), getMotorPWM(MOTOR3));

  /*
  data[0] = (sword) ((((udword) getMotorPWM(MOTOR1)) * ((uword) MAX_PWM_RANGE)) / ((udword) MAX_PWM));
  data[1] = (sword) ((((udword) getMotorPWM(MOTOR2)) * ((uword) MAX_PWM_RANGE)) / ((udword) MAX_PWM));
  data[2] = (sword) ((((udword) getMotorPWM(MOTOR3)) * ((uword) MAX_PWM_RANGE)) / ((udword) MAX_PWM));
  */

  data[0] = (sword) getMotorPWM(MOTOR1);
  data[1] = (sword) getMotorPWM(MOTOR2);
  data[2] = (sword) getMotorPWM(MOTOR3);

  if(getMotorDirection(MOTOR1) == DIRECTION_CW) { data[0] *= -1; }
  if(getMotorDirection(MOTOR2) == DIRECTION_CW) { data[1] *= -1; }
  if(getMotorDirection(MOTOR2) == DIRECTION_CW) { data[2] *= -1; }

  //printf("%d %d %d\n", data[0], data[1], data[2]);

  cn_write_response(GROUP_REQUEST_RESPONSE, CMD_MOTOR_PWM, (ubyte*) data, (sizeof(*data) * 3));

}

void cn_command_request_encoder_relative() {

  cn_write_response(GROUP_REQUEST_RESPONSE, CMD_ENCODER_REL, (ubyte*) encoder_accu_rel, (sizeof(sdword) * 3));

  encoder_accu_rel[0] = 0;
  encoder_accu_rel[1] = 0;
  encoder_accu_rel[2] = 0;

}

void cn_command_request_battery_voltage() {

  uword voltage = getSupplyVoltage();

  //printf("inside Command: RequestBatteryVoltage()");

  cn_write_response(GROUP_REQUEST_RESPONSE, CMD_BATTERY_VOLTAGE, (ubyte*) &voltage, sizeof(uword));

}

void cn_command_request_motion_vector() {}

void cn_command_request_average_sleep_time() {}

void cn_command_request_path_vector() {
	sword answer[3];
	answer[0] = (sword)controller_stat.actual_motion_vector.x;
	answer[1] = (sword)controller_stat.actual_motion_vector.y;
	answer[2] = (sword)(controller_stat.actual_motion_vector.rotation/(ROTATION_PRECISION/64));
	cn_write_response(GROUP_REQUEST_RESPONSE,CMD_PATH_VECTOR,(ubyte*)&answer,sizeof(sword)*3);
}

void cn_command_request_odometry_log(uword pos) {
	sword answer[ODO_LOG_NUM_ELEMENTS];
//	sword answer[2];
	unsigned int i;
//	if (pos >= LINE_WIDTH*LINE_WIDTH) {
	if (pos >= ODO_LOG_SIZE) {
		cn_write_response(GROUP_ERROR_RESPONSE,CMD_OUT_OF_RANGE,0x00,0);
	}
	else {
		pos += odolog_write_pos;
		if (pos >=ODO_LOG_SIZE) pos -= ODO_LOG_SIZE;
		for(i = 0; i < ODO_LOG_NUM_ELEMENTS; ++i) {
			answer[i] = (sword) odometry_log[pos][i];
		}

		//cn_get_force_correction_by_idx(pos,answer);
		cn_write_response(GROUP_REQUEST_RESPONSE,CMD_READ_ODO_LOG, (ubyte*)answer, sizeof(sword) * ODO_LOG_NUM_ELEMENTS);
		//cn_write_response(GROUP_REQUEST_RESPONSE,CMD_READ_ODO_LOG, (ubyte*)answer, sizeof(sword) * 2);
	}
}

// ------------------------------------------------------------------

void cn_command_controlconf_pid_kp(sword* kp) {
 	controller_params_passive.kp = *kp;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_PID_KP,(ubyte*)kp,sizeof(sword));
}
void cn_command_controlconf_pid_ki(sword* ki) {
 	controller_params_passive.ki = *ki;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_PID_KI,(ubyte*)ki,sizeof(sword));
}
void cn_command_controlconf_pid_b(sword* b) {
 	controller_params_passive.b = *b;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_PID_B,(ubyte*)b,sizeof(sword));
}
void cn_command_controlconf_pid_kd(sword* kd) {
 	controller_params_passive.kd = *kd;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_PID_KD,(ubyte*)kd,sizeof(sword));
}
void cn_command_controlconf_pid_kdi(sword* kdi) {
 	controller_params_passive.kdi = *kdi;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_PID_KDI,(ubyte*)kdi,sizeof(sword));
}
void cn_command_controlconf_pid_lin(sword* lin) {
 	controller_params_passive.lin = (sdword)*lin;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_PID_LIN,(ubyte*)lin,sizeof(sword));
}



void cn_command_controlconf_current_error_bound(sword* bound) {
 	controller_params_passive.current_error_bound = *bound;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_CURRENT_ERROR_BOUND,(ubyte*)bound,sizeof(sword));
}

void cn_command_controlconf_current_kp(sword* kp) {
 	controller_params_passive.current_kp = *kp;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_CURRENT_KP,(ubyte*)kp,sizeof(sword));
}
void cn_command_controlconf_current_ki(sword* ki) {
 	controller_params_passive.current_ki = *ki;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_CURRENT_KI,(ubyte*)ki,sizeof(sword));
}
void cn_command_controlconf_current_kd(sword* kd) {
 	controller_params_passive.current_kd = *kd;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_CURRENT_KD,(ubyte*)kd,sizeof(sword));
}
void cn_command_controlconf_controller_commit() {
  controller_params_commit = 1;
  cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_CONTROLLER_COMMIT,0x00,0);
}

void cn_command_controlconf_dead_band(uword* dead_band) {
	controller_params_passive.dead_band = *dead_band;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_DEAD_BAND,(ubyte*)dead_band,sizeof(uword));  
}
void cn_command_controlconf_max_error_int(uword* max_error) {	
	controller_params_passive.max_error_int = ((sdword)(*max_error))*8192l;	
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_MAX_ERROR_INT,(ubyte*)max_error,sizeof(uword));  
}
void cn_command_controlconf_smooth(sword* smooth) {	
	controller_params_passive.smooth = (sdword)(*smooth);	
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_SMOOTH,(ubyte*)smooth,sizeof(sword));  
}
void cn_command_controlconf_max_rotation_accel(uword* maxRot) {	
	controller_params_passive.max_rotation_accel = ((udword)(*maxRot))*(ROTATION_PRECISION/64);	
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_MAX_ROTATION_ACCEL,(ubyte*)maxRot,sizeof(uword));  
}

void cn_command_controlconf_rot_err_w(uword* weight) {
	controller_params_passive.rot_err_w = *weight;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_ROTATION_ERR_W,(ubyte*)weight,sizeof(uword));  
}
void cn_command_controlconf_rot_err_accel_w(uword* weight) {
	controller_params_passive.rot_err_accel_w = *weight;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_ROTATION_ERR_ACCEL_W,(ubyte*)weight,sizeof(uword)); 
}
void cn_command_controlconf_rot_err_velo_w(uword* weight) {
	controller_params_passive.rot_err_velo_w = *weight;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_ROTATION_ERR_VELO_W,(ubyte*)weight,sizeof(uword)); 
}
void cn_command_controlconf_accel_bound_curve_min(uword* bound) {
	controller_params_passive.accelbound_min = *bound;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_ACCEL_BOUND_CURVE_MIN,(ubyte*)bound,sizeof(uword));
}
void cn_command_controlconf_accel_bound_curve_max(uword* bound) {
	controller_params_passive.accelbound_max = *bound;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_ACCEL_BOUND_CURVE_MAX,(ubyte*)bound,sizeof(uword));
}
void cn_command_controlconf_max_wheel_accel(sword* bound) {
	controller_params_passive.maxWheelAccel = (*bound);
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_MAX_WHEEL_ACCEL,(ubyte*)bound,sizeof(sword));
}
void cn_command_controlconf_max_wheel_deccel(sword* bound) {
	controller_params_passive.maxWheelDeccel = (*bound);
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_MAX_WHEEL_DECCEL,(ubyte*)bound,sizeof(sword));
}
void cn_command_controlconf_accel_curve_degree(float degree) {
  dummy = degree;
}
void cn_command_controlconf_fail_safe_values(uword* rpm_bound, uword* pwm_bound, uword* timeout) {
	uword answer[3];
	controller_params_passive.fail_safe_rpm_bound = *rpm_bound;
	controller_params_passive.fail_safe_pwm_bound = *pwm_bound;
	controller_params_passive.fail_safe_cycles    = *timeout;
	answer[0] = *rpm_bound;
	answer[1] = *pwm_bound;
	answer[2] = *timeout;
	cn_write_response(GROUP_CONTROL_CONFIG_RES,CMD_FAIL_SAFE_VALUES,(ubyte*)answer,sizeof(uword)*3);
}