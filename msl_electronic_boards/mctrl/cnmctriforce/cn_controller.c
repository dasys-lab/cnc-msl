#include "cn_controller.h"
#include "cn_motor.h"
#include "cn_adc.h"
#include "cn_current.h"
#include "cn_time.h"
#include "cn_led.h"
//#include "cn_force_lookup.h"

/*****************
Robot:
	 x
	 ^
	 |
y<---|
	+A1 Dir-A1
	+rot  <-|
			|
			|
			|   		
		   /\
	  	  /  \
	     /	  \	   +A3Dir
	    /	   \  /
   	   A2  		A3
   		\
		+A2Dir

- Robot Rotation is mathematically positive
- Wheel Directions are assumed to be such that they rotate positively, if the robot rotates positively
- Hence, Wheels rotate mathematically negatively (i.e., clockwise).
*****************/

uword controller_error_word;
ubyte controller_error_overcurrent;

uword cn_controller_current_max;
uword cn_controller_current_nom;

controller_params	controller_params_active;
controller_params	controller_params_passive;
bool              	controller_params_commit;
ubyte             	controller_mode;
//udword				last_cycle_time;
sdword				delta_time;

controller_pwm_vector controller_direct_pwm_target;
controller_pwm_vector controller_direct_pwm_final;

motor_params controller_motor_params;

controller_status controller_stat;
sdword delta_motion[3];
sdword accel_vector[2];
sdword deccel_vector[2];
sdword rot_vector[2];
sdword temp_vector[2];
sdword correction_vector[2];
float angle, accel, maxAccel;
float a,b,c, tempf;
uword dead_band_counter, fail_safe_counter, sleep_counter;

//sdword rpm_ringbuff[ODOBUFSIZE][3];
//sdword odo_ringbuff[ODOBUFSIZE][3];
sdword rpm_smoothed[3];
sdword odo_smoothed[3];
//uword odo_buff_pos;


log_mode controller_log_mode;


sword xhuge odometry_log[ODO_LOG_SIZE][ODO_LOG_NUM_ELEMENTS];
uword odolog_write_pos=0;

void cn_controller_init() {
  	 uword i;
  // load default values
  	controller_log_mode.log_enabled = 0;
	
	controller_log_mode.log[LOG_RPM] 		= -1;
	controller_log_mode.log[LOG_PWM] 		= -1;
	controller_log_mode.log[LOG_RPMGoal]	= -1;
	controller_log_mode.log[LOG_Current]	= -1;
	controller_log_mode.log[LOG_MGoal] 		= -1;
  	controller_log_mode.log[LOG_MRequest]	= -1;
	controller_log_mode.log[LOG_MaxPWM]		= -1;
	controller_log_mode.log[LOG_ErrorInt]	= -1;
	controller_log_mode.log[LOG_Motion]		= -1;
	controller_log_mode.log[LOG_MSmooth]	= -1;
	controller_log_mode.log[LOG_RPMSmooth]	= -1;
		
	
	controller_error_word = 0;
	controller_error_overcurrent = 0;

	controller_params_commit = 1;
	cn_controller_update_params();
	
	cn_controller_current_max = CN_CONTROLLER_CURRENT_DEFAULT_MAX;
	cn_controller_current_nom = CN_CONTROLLER_CURRENT_DEFAULT_NOM;
	
	//last_cycle_time = cn_getTimer();	

	controller_motor_params.maxRPM				= 10;
	controller_motor_params.nominal_current		= 10;
	controller_motor_params.gear_numerator		= 1;
	controller_motor_params.gear_denumerator	= 12;
	controller_motor_params.wheel_radius		= 650;
	controller_motor_params.robot_radius		= 245;
	controller_motor_params.encoder_ticks		= 500;
	controller_motor_params.ticks_2_rpms_factor	= 200; 
	controller_motor_params.wheel_circ_64		= 1;
	
	for (i=0; i<3; i++) {
		odo_smoothed[i] = 0;
		rpm_smoothed[i] = 0;
/*
		for (j=0; j< ODOBUFSIZE; j++) {
			rpm_ringbuff[j][i] = 0;
			odo_ringbuff[j][i] = 0;
		}
*/		
	}
//	odo_buff_pos = 0;
	
	for(i=0; i<3; ++i) {
		controller_stat.actual_rpm[i]=0;
	}
	controller_stat.actual_motion_vector.x = 0;
	controller_stat.actual_motion_vector.y = 0;
	controller_stat.actual_motion_vector.rotation = 0;

	cn_controller_reset();

	controller_params_passive.accelbound_min 	= 0;
	controller_params_passive.accelbound_max 	= 0;
	controller_params_passive.max_rotation_accel= 0;
	controller_params_passive.kp 				= 0;
	controller_params_passive.b  				= 0;
	controller_params_passive.ki 				= 0;	
	controller_params_passive.kd 				= 0;
	controller_params_passive.kdi 				= 0;
	controller_params_passive.lin 				= 0;
	controller_params_passive.max_error_int		= 0;
	
	controller_params_passive.dead_band 		= 0;
	controller_params_passive.fail_safe_rpm_bound = 10;
	controller_params_passive.fail_safe_pwm_bound = 100;
	controller_params_passive.fail_safe_cycles    = 0;
	
	controller_params_passive.rot_err_w 		= 0;
	controller_params_passive.rot_err_velo_w 	= 0;
	controller_params_passive.rot_err_accel_w 	= 0;
	
	controller_params_passive.max_acceleration = 10;
	controller_params_passive.max_decceleration = 10;	
	controller_params_passive.max_rot_force = 10;
	
	controller_params_passive.smooth = 8192;
	
	controller_params_passive.current_error_bound = CN_CONTROLLER_CURRENT_DEFAULT_NOM;
	controller_params_passive.current_kp = 0;
	controller_params_passive.current_ki = 0;
	controller_params_passive.current_kd = 0;
	
	controller_params_commit = 1;
	dead_band_counter = 0;
	sleep_counter     = 0;
	fail_safe_counter  = 0;
	
	controller_direct_pwm_target.pwm[0] = ZERO_PWM;
	controller_direct_pwm_target.pwm[1] = ZERO_PWM;
	controller_direct_pwm_target.pwm[2] = ZERO_PWM;
	
	cn_controller_update_params();
	//cn_init_force_lookup();
	cn_start_command_timeout();
}

void cn_controller_reset() {
	uword i;
	for(i=0; i<3;++i) {
		controller_stat.rpm_goal[i]		= controller_stat.actual_rpm[i]; //Set Ramp down to 0-vector
		controller_stat.pid_e[i]		= 0;
		controller_stat.pid_eI[i]		= 0;
		controller_stat.pid_eD[i]		= 0;
		controller_stat.pwm[i]			= 0;
		controller_stat.last_rpm[i] 	= 0;
		controller_stat.last_current[i]	= 0;
		controller_stat.current_eI[i]	= 0;
		controller_stat.maxPWM[i]		= MAX_PWM;
	}
	controller_stat.allMaxPWM			= MAX_PWM;
	controller_stat.motion_request.x=0;
	controller_stat.motion_request.y=0;
	controller_stat.motion_request.rotation=0;

	controller_stat.motion_goal.x=controller_stat.actual_motion_vector.x; //Set Ramp down to 0-vector
	controller_stat.motion_goal.y=controller_stat.actual_motion_vector.y;
	controller_stat.motion_goal.rotation=controller_stat.actual_motion_vector.rotation;
	
	controller_stat.rotation_error = 0;
}

void cn_controller_update_params() {

  	int i;
	ubyte* active;
	ubyte* passive;
	//bool update_lookup = 0;	
	if(controller_params_commit) {
    	/*
		update_lookup = (controller_params_active.accelbound_min != controller_params_passive.accelbound_min) ||
						(controller_params_active.accelbound_max != controller_params_passive.accelbound_max);
		*/
		controller_params_commit = 0;
		
		active  = (ubyte*) &controller_params_active;
		passive = (ubyte*) &controller_params_passive;
		
		for(i = 0; i < sizeof(controller_params); ++i) {
		  active[i] = passive[i];
		}
		cn_controller_recalc_derived_controller_params();    
		/*if (update_lookup) {
			normal_op=0;
			cn_controller_prep_lookup();	
		}*/
	}


}

void cn_controller_emergency_stop() {
	
	cn_controller_set_error(CN_CONTROLLER_ERROR_TIMEOUT);
	
	greenLedToggle();
#ifdef VMC
	//redLedOn();
#endif
	
}

void cn_controller_current_control() {
	sdword temp;
	uword i;
	sdword e;
	// check currents
	for(i = 0; i < 3; ++i) {
		
		if(motor_current[i] >= cn_controller_current_max) {
		
			// set overcurrent bit for motor
			controller_error_overcurrent |= (1 << i);
			
			//don't break: find all overcurrent motors
			
		} else if(motor_current[i] <= cn_controller_current_nom) {
		
			// reset overcurrent bit for motor
			controller_error_overcurrent &= ~(1 << i);
			
		}
		
	}
	
	if(controller_error_overcurrent != 0) {
	
		// overcurrent != 0 --> at least one motor has overcurrent
		cn_controller_set_error(CN_CONTROLLER_ERROR_CURRENT);
		
	} else {
	
		// overcurrent == 0 --> all motors are ok
		cn_controller_reset_error(CN_CONTROLLER_ERROR_CURRENT);
	 
	}
	controller_stat.allMaxPWM = MAX_PWM;
	for (i = 0; i < 3; ++i) { //Current Control PID
		//error		
		e = motor_current[i];
		e -= controller_params_active.current_error_bound;
		e *= MAX_PWM;
		e /= cn_controller_current_max;
		//error integral
		controller_stat.current_eI[i] += (controller_params_active.current_ki*e)/8192l;
		controller_stat.current_eI[i]  = CLAMP(controller_stat.current_eI[i],0,MAX_PWM);
		
		controller_stat.maxPWM[i] = MAX_PWM - controller_stat.current_eI[i];
		//proportional:
		if (motor_current[i] > controller_params_active.current_error_bound) {
			temp = e*controller_params_active.current_kp;
			temp /= 8192;
			controller_stat.maxPWM[i] -= temp;
		}
		
		
		
		//differential
		if (motor_current[i] >= controller_params_active.current_error_bound || controller_stat.last_current[i] >= controller_params_active.current_error_bound) {
			temp = (motor_current[i]-controller_stat.last_current[i])*controller_params_active.current_kd;
			temp /= 8192;
			temp *= MAX_PWM;
			temp /= cn_controller_current_max;
			controller_stat.maxPWM[i] -= temp;
		}

		controller_stat.maxPWM[i] = CLAMP(controller_stat.maxPWM[i],0,MAX_PWM);
		controller_stat.allMaxPWM = MIN(controller_stat.allMaxPWM,controller_stat.maxPWM[i]);
		controller_stat.last_current[i] = motor_current[i];
	}
	

}

void cn_controller_main_control() {

	cn_controller_calc_odometry();
#ifdef VMC
	cn_controller_current_control();
#endif	
	if(cn_controller_has_error() == 0) {
		// control normally
		switch(controller_mode) {
	
			case CONTROLLER_MODE_PID:
				cn_controller_control_pid();
			break;
	
			case CONTROLLER_MODE_DIRECT:
				cn_controller_control_direct();
			break;
			  
		  	default:
	      		cn_controller_emergency_stop();
			break;
		}
		
	} else if (cn_controller_is_error(CN_CONTROLLER_ERROR_TIMEOUT | CN_CONTROLLER_ERROR_CURRENT)) {
		
		// regular cleanup while no valid command
		cn_controller_reset();
		
		// overwrite all motor pwm
		// always do last
		controller_direct_pwm_final.pwm[0] = ZERO_PWM;
		controller_direct_pwm_final.pwm[1] = ZERO_PWM;
		controller_direct_pwm_final.pwm[2] = ZERO_PWM;
		controller_direct_pwm_final.dir[0] = DIRECTION_FREE;
		controller_direct_pwm_final.dir[1] = DIRECTION_FREE;
		controller_direct_pwm_final.dir[2] = DIRECTION_FREE;
		
		
	}

}

void cn_controller_control_direct() {

	int i=0;

	for(; i < 3; ++i) {
		controller_direct_pwm_final.dir[i] = controller_direct_pwm_target.dir[i];
		controller_direct_pwm_final.pwm[i] = controller_direct_pwm_target.pwm[i];
	}
			
}

void cn_controller_control_pid() {
	uword i;
	sdword temp;
	float temp_float;
	//first, get delta time
	

//	printf("P %u %lu %u\n",controller_motor_params.wheel_radius, controller_motor_params.wheel_circ_64,controller_motor_params.robot_radius);
//	printf("C %ld %ld %ld\n",controller_stat.motion_request.x,controller_stat.motion_request.y,controller_stat.motion_request.rotation);
//	printf("M %ld %ld %ld\n",controller_stat.actual_motion_vector.x,controller_stat.actual_motion_vector.y,controller_stat.actual_motion_vector.rotation);
	
	//DEAD BAND 	
		//if motion request says don't move and all rpm are below dead band, reset control and return
		if (controller_stat.motion_request.x == 0 && controller_stat.motion_request.y == 0 && controller_stat.motion_request.rotation == 0 &&
			ABS(controller_stat.actual_rpm[0]) < controller_params_active.dead_band &&
			ABS(controller_stat.actual_rpm[1]) < controller_params_active.dead_band &&
			ABS(controller_stat.actual_rpm[2]) < controller_params_active.dead_band
		) {
			dead_band_counter = MAX(dead_band_counter,dead_band_counter+1);
		} else {
			dead_band_counter = 0;
		}
		if (dead_band_counter >= (TIME_TICKS_PER_SECOND/2)/delta_time) {
			cn_controller_reset();	
			controller_direct_pwm_final.pwm[0] = 0;
			controller_direct_pwm_final.pwm[1] = 0;	
			controller_direct_pwm_final.pwm[2] = 0;
			return;
		}
	//FAIL SAFE comes here
		//if rpm goal is over upper bound and actual rpm is under lower bound, iterate count		
		if ((ABS(controller_stat.actual_rpm[0]) < controller_params_active.fail_safe_rpm_bound && ABS(controller_stat.pwm[0]) > controller_params_active.fail_safe_pwm_bound) ||
		(ABS(controller_stat.actual_rpm[1]) < controller_params_active.fail_safe_rpm_bound && ABS(controller_stat.pwm[1]) > controller_params_active.fail_safe_pwm_bound) ||
		(ABS(controller_stat.actual_rpm[2]) < controller_params_active.fail_safe_rpm_bound && ABS(controller_stat.pwm[2]) > controller_params_active.fail_safe_pwm_bound)
		) {
			fail_safe_counter = MAX(fail_safe_counter,fail_safe_counter+1);
			//if counter has reached limit, set sleep counter
			if (fail_safe_counter > controller_params_active.fail_safe_cycles) {
				sleep_counter = (uword)((TIME_TICKS_PER_SECOND/2)/delta_time);
			}

		} else {
			fail_safe_counter = 0;
		}		
		if (sleep_counter > 0) { //if sleep counter is active, reset controller, kill pwm, and decrement it.
			--sleep_counter;
			cn_controller_reset();
			controller_direct_pwm_final.pwm[0] = 0;
			controller_direct_pwm_final.pwm[1] = 0;	
			controller_direct_pwm_final.pwm[2] = 0;
			return;
		}
		//switch off everything if overheating is detected
#ifdef VMC
		if(isOvertemp(MOTOR1) || isOvertemp(MOTOR2) || isOvertemp(MOTOR3)) {
			sleep_counter = (uword)(TIME_TICKS_PER_SECOND/delta_time);			
			cn_controller_reset();
			controller_direct_pwm_final.pwm[0] = 0;
			controller_direct_pwm_final.pwm[1] = 0;	
			controller_direct_pwm_final.pwm[2] = 0;			
			return;			
		}
#endif
	//Now Calculate the new cartesian Motion Goal:	
	//Calculate Vector from Actual_Motion_vector to Motion_request
	delta_motion[0] = controller_stat.motion_request.x - controller_stat.motion_goal.x;
	delta_motion[1] = controller_stat.motion_request.y - controller_stat.motion_goal.y;
 	delta_motion[2] = controller_stat.motion_request.rotation - controller_stat.motion_goal.rotation;

	//RAMP rotation velocity:
	temp = (((sdword)controller_params_active.max_rotation_accel) * delta_time)/TIME_TICKS_PER_SECOND;
	delta_motion[2] = CLAMP(delta_motion[2],-temp,temp);

    
	//Calculate deccleration and accleration vector:
	
	
	if (controller_stat.motion_goal.x == 0 && controller_stat.motion_goal.y == 0) { //we are standing still, everything is acceleration
		accel_vector[0] = delta_motion[0];
		accel_vector[1] = delta_motion[1];
		deccel_vector[0] = 0;
		deccel_vector[1] = 0;
	} else {
	/*	temp = sqrt(controller_stat.actual_motion_vector.x*controller_stat.actual_motion_vector.x+controller_stat.actual_motion_vector.y*controller_stat.actual_motion_vector.y);
		normVelo[0] = controller_stat.actual_motion_vector.x * 8192l;
		normVelo[1] = controller_stat.actual_motion_vector.y * 8192l;
		normVelo[0] /= temp;
		normVelo[1] /= temp;
	*/
		if (controller_stat.motion_goal.y != 0) { //check against division by zero
			b = delta_motion[0] * controller_stat.motion_goal.x;
			b /= controller_stat.motion_goal.y;
			b += delta_motion[1];
			
			c = controller_stat.motion_goal.x*controller_stat.motion_goal.x;
			c /= -controller_stat.motion_goal.y;
			c += controller_stat.motion_goal.y;
			
			b /= c;
			a = delta_motion[0];
			a -= b*controller_stat.motion_goal.x;
			a /= controller_stat.motion_goal.y;	
		} else {	// simplified case
			b = delta_motion[0];
			b /= controller_stat.motion_goal.x;
			a = - delta_motion[1];
			a /= controller_stat.motion_goal.x;
		}
	}
	if (b > 0) { //acceleration along velocity vecotr is positive => everything is acceleration again
		accel_vector[0] = delta_motion[0];
		accel_vector[1] = delta_motion[1];
		deccel_vector[0] = 0;
		deccel_vector[1] = 0;		
	}
	else { //we got deceleration
		accel_vector[0] = (sdword)ROUND(a* (float)controller_stat.motion_goal.y); 
		accel_vector[1] = -(sdword)ROUND(a* (float)controller_stat.motion_goal.x); 
		deccel_vector[0] = (sdword)ROUND(b* (float)controller_stat.motion_goal.x); 
		deccel_vector[1] = (sdword)ROUND(b* (float)controller_stat.motion_goal.y); 
	}
	
	
	tempf = sqrt((float)(accel_vector[0]*accel_vector[0]+accel_vector[1]*accel_vector[1]));
	if (tempf > controller_params_active.max_acceleration) {
		accel_vector[0]*= controller_params_active.max_acceleration;
		accel_vector[0]/= tempf;
		accel_vector[1]*= controller_params_active.max_acceleration;
		accel_vector[1]/= tempf;
	}
	tempf = sqrt((float)(deccel_vector[0]*deccel_vector[0]+deccel_vector[1]*deccel_vector[1]));
	if (tempf > controller_params_active.max_decceleration) {
		deccel_vector[0]*= controller_params_active.max_decceleration;
		deccel_vector[0]/= tempf;
		deccel_vector[1]*= controller_params_active.max_decceleration;
		deccel_vector[1]/= tempf;
	}
	

//DEBUG
//delta_motion[0]=CLAMP(delta_motion[0],-10,10);
//delta_motion[1]=CLAMP(delta_motion[1],-10,10);
//END DEBUG


	//Calculate accel Vector:
	//Accel = Rotation * Velocity - Delta_Motion
//	accel_vector[0] = (controller_stat.actual_motion_vector.rotation*controller_stat.motion_goal.y)/(64);
//	accel_vector[1] = -(controller_stat.actual_motion_vector.rotation*controller_stat.motion_goal.x)/(64);

//	accel_vector[0] = (controller_stat.actual_motion_vector.rotation*controller_stat.actual_motion_vector.y)/(64);
//	accel_vector[1] = -(controller_stat.actual_motion_vector.rotation*controller_stat.actual_motion_vector.x)/(64);


	//this just barely fits into 32bit woth 10m/s, 8pi/s and 8192 PRECISION
	rot_vector[0] = (controller_stat.motion_goal.rotation*controller_stat.motion_goal.y)/ROTATION_PRECISION;
	rot_vector[1] = -(controller_stat.motion_goal.rotation*controller_stat.motion_goal.x)/ROTATION_PRECISION;


	rot_vector[0] *= delta_time;
	rot_vector[1] *= delta_time;
	rot_vector[0] /= TIME_TICKS_PER_SECOND;
	rot_vector[1] /= TIME_TICKS_PER_SECOND;
	
	
	tempf = sqrt((float)(rot_vector[0]*rot_vector[0]+rot_vector[1]*rot_vector[1]));
	if (tempf > controller_params_active.max_rot_force) { //rotation needs to be compensated
		a = tempf - controller_params_active.max_rot_force;
		rot_vector[0] *= a;
		rot_vector[1] *= a;
		rot_vector[0] /= tempf;
		rot_vector[1] /= tempf;
	} else {
		rot_vector[0] = 0;
		rot_vector[1] = 0;
	}
	
	delta_motion[0] = accel_vector[0]+deccel_vector[0] + rot_vector[0];
	delta_motion[1] = accel_vector[1]+deccel_vector[1] + rot_vector[1];
	
	accel =  sqrt((float)(delta_motion[0]*delta_motion[0]+delta_motion[1]*delta_motion[1]));
	

	//...and set it as Motion_Goal
	controller_stat.motion_goal.x += delta_motion[0];
	controller_stat.motion_goal.y += delta_motion[1];
	controller_stat.motion_goal.rotation += delta_motion[2];
	
//DEBUG:
//controller_stat.motion_goal.x = controller_stat.motion_request.x;
//controller_stat.motion_goal.y = controller_stat.motion_request.y;
//END DEBUG



	//Transform Motion_Goal to rpm
	//Basic equation:
	// (v1)		( 0    1 R) (Vx)
	// (v2) = 	(-t -0.5 R) (Vy)
	// (v3)		( t -0.5 R) (Vw)

	//where t = sin(120) and R the robot radius
//TODO: Optimise and reconsider ranges
	//rotation first:
	temp = controller_stat.motion_goal.rotation;
	temp *= controller_motor_params.robot_radius;
	temp /= controller_motor_params.wheel_circ_4;	//this takes care of the 64 factor as well
	temp *= 75; //10 (dezi millimeter) * 60 (seconds)	= 600 = 8*75
	temp /= (ROTATION_PRECISION/32);
	for (i=0; i<3; ++i) {
		controller_stat.rpm_goal[i] = temp;
	}
	//add trans in x-dir:
	temp = controller_stat.motion_goal.x;
	temp *= 600;
	temp *= SIN_120_64; //just _barely_ within sdword, works upto 8m/s...
//	temp *= SIN_120_8;
//	temp *= 8; //64/8  
	temp /= controller_motor_params.wheel_circ_64;	
	controller_stat.rpm_goal[1] -=  temp;
	controller_stat.rpm_goal[2] +=	temp;

	//add trans in y-dir:
	temp = controller_stat.motion_goal.y;	
	temp *= 600l*64l; //10 (dezi millimetter) * 60 (seconds) * 64
	temp /= controller_motor_params.wheel_circ_64;
	controller_stat.rpm_goal[0] += temp;
	temp /= 2;
	controller_stat.rpm_goal[1] -= temp;
	controller_stat.rpm_goal[2] -= temp;

	//Do the PID ... Oh, come on PID me baby...
	
	//Rotation Error:
	controller_stat.rotation_error  = controller_stat.motion_goal.rotation - controller_stat.actual_motion_vector.rotation; //actual error
	//controller_stat.rotation_error *= controller_params_active.rot_err_w; //weight
	controller_stat.rotation_error *= CN_CONTROLLER_MAX_E;//controller_stat.allMaxPWM;//MAX_PWM;
	//controller_stat.rotation_error /= PI_256; //scaled down (rotation max at 4pi /s, scaled up by 64)
	controller_stat.rotation_error /= PI_32768; //scaled down (rotation max at 4pi /s, scaled up by 8192)
	//controller_stat.rotation_error /= 8192l;   //and again for weight

	//TODO: Clean up and optimise calculations below:
	//current veloctiy:
	
	//weighted:

	temp_float = accel;
	temp_float *= controller_params_active.rot_err_accel_w;
	temp_float *= TIME_TICKS_PER_SECOND; //scale up to mm/s
	temp_float /= delta_time;	
	
	temp_float += controller_params_active.rot_err_velo_w * sqrt((float)(controller_stat.motion_goal.x*controller_stat.motion_goal.x +controller_stat.motion_goal.y*controller_stat.motion_goal.y));
	

		
	temp_float /= controller_motor_params.maxRPM; //divide by max velo
	temp_float *= 600.0/TWO_PI;  //60 (RPM to RPS), 10 (dezi millimeter), and wheel circ
	temp_float /= controller_motor_params.wheel_radius;
	
	
	//temp_float /= controller_motor_params.maxRPM;
	temp_float += controller_params_active.rot_err_w;
	
	controller_stat.rotation_error  *= temp_float; //error weighted by acceleration and velocity
	controller_stat.rotation_error  /= 8192l;
		
	for(i=0; i<3;++i) {
		//D Part:
		temp = (controller_stat.actual_rpm[i] - controller_stat.last_rpm[i]); //derivative
		temp *= controller_params_active.kd; 								  //weight
		temp *= CN_CONTROLLER_MAX_E;//controller_stat.maxPWM[i];//MAX_PWM;													  //scaled up
		temp /= (controller_motor_params.maxRPM); 				  //scaled down and derived with dt
		//temp *= TIME_TICKS_PER_MILLISECOND/2;
		controller_stat.pid_eD[i] = controller_params_active.kdi * controller_stat.pid_eD[i]; //old value, weighted
																							  //TODO: should time occur here?
		
		controller_stat.pid_eD[i] -= temp;									  //substract new value
		controller_stat.pid_eD[i] = CLAMP(controller_stat.pid_eD[i],-8192l*CN_CONTROLLER_MAX_E, 8192l*CN_CONTROLLER_MAX_E);
		//I PART:
		temp = controller_stat.rpm_goal[i] - controller_stat.actual_rpm[i];  //the relevant error		
		temp *=	controller_params_active.ki;								 //times weight
		temp /= controller_motor_params.maxRPM;								 //scaled down from rpm
		//temp *= delta_time;													 //Integrated over time
		temp *= CN_CONTROLLER_MAX_E;//controller_stat.maxPWM[i];//MAX_PWM;													 //scaled up to PWM
		temp += controller_stat.rotation_error*controller_params_active.ki;	  //add rotation error;
		//temp /= TIME_TICKS_PER_MILLISECOND/2;
		controller_stat.pid_eI[i] += temp;
		//controller_stat.pid_eI[i] =  CLAMP(controller_stat.pid_eI[i],-8192l*MAX_PWM*2l, 8192l*MAX_PWM*2l);
		//controller_stat.pid_eI[i] =  CLAMP(controller_stat.pid_eI[i],-8192l*controller_stat.maxPWM[i]*2l, 8192l*controller_stat.maxPWM[i]*2l);
		
		
		//Clamp integral by current:
		//temp = (controller_params_active.max_error_int / MAX_PWM) * controller_stat.allMaxPWM;
		//controller_stat.pid_eI[i] =  CLAMP(controller_stat.pid_eI[i],-temp,temp);
		
		//Clamp integral statically:
		controller_stat.pid_eI[i] =  CLAMP(controller_stat.pid_eI[i],-controller_params_active.max_error_int,controller_params_active.max_error_int);
		
		//P Part:
		temp = ((controller_params_active.b * controller_stat.rpm_goal[i])/8192l - controller_stat.actual_rpm[i]);		
		temp *= CN_CONTROLLER_MAX_E;//controller_stat.maxPWM[i];//MAX_PWM;
		temp /= controller_motor_params.maxRPM;
		temp += controller_stat.rotation_error;
		controller_stat.pid_e[i] = controller_params_active.kp * temp;
						

		//Prep for next cycle:
		controller_stat.pid_eD[i] /= 8192l;
		controller_stat.last_rpm[i] = controller_stat.actual_rpm[i];

		//Sum UP:		
//		controller_stat.pwm[i] =(sword) ((controller_stat.pid_e[i]+controller_stat.pid_eI[i])/8192l + controller_stat.pid_eD[i]);		
//		controller_stat.pwm[i] = CLAMP(controller_stat.pwm[i], -controller_stat.maxPWM[i], controller_stat.maxPWM[i]);
		
		//sum up pid
		temp =(sdword) ((controller_stat.pid_e[i]+controller_stat.pid_eI[i])/8192l + controller_stat.pid_eD[i]);
		
		//add linear factor:
		temp+= (((controller_params_active.lin*controller_stat.rpm_goal[i])/controller_motor_params.maxRPM)*CN_CONTROLLER_MAX_E)/8192l;
		
		//scale pwm		
		temp *= MAX_PWM;
		temp /= CN_CONTROLLER_MAX_E;
		
		//controller_stat.pwm[i] = CLAMP(temp, -controller_stat.maxPWM[i], controller_stat.maxPWM[i]);
		controller_stat.pwm[i] = CLAMP(temp, -controller_stat.allMaxPWM, controller_stat.allMaxPWM);
		
		
		
	}
	//printf("PA: %d %d %d %d %d\n",controller_params_active.kp,controller_params_active.b,controller_params_active.ki,controller_params_active.kd,controller_params_active.kdi);
//	printf("PID: %ld %ld %ld\n", controller_stat.pid_e[2],controller_stat.pid_eI[2],controller_stat.pid_eD[2]);
//	printf("PWM: %d %d %d\n",controller_stat.pwm[0],controller_stat.pwm[1],controller_stat.pwm[2]);
	//Set PWM
	controller_direct_pwm_final.pwm[0] = ABS(controller_stat.pwm[0]);
	controller_direct_pwm_final.pwm[1] = ABS(controller_stat.pwm[1]);	
	controller_direct_pwm_final.pwm[2] = ABS(controller_stat.pwm[2]);
	controller_direct_pwm_final.dir[0] = -(sbyte)SIGN2(controller_stat.pwm[0]);
	controller_direct_pwm_final.dir[1] = -(sbyte)SIGN2(controller_stat.pwm[1]);
	controller_direct_pwm_final.dir[2] = -(sbyte)SIGN2(controller_stat.pwm[2]);
	
	//DONE
}

void cn_controller_calc_odometry() {
	uword i;
	sdword temp;
//	udword now = cn_getTimer();
	delta_time = last_cycle_ticks;//(now - last_cycle_time);
//	last_cycle_time = now; //note that we do not want gaps in the time measure

	//Calculate RPM:
	for(i=0; i<3; ++i) { //this calculation is sensitve too overflow, should work up 10.000rpm motor and 100ms cycle time with 500 ticks per half rotation
		controller_stat.actual_rpm[i]= (encoder_rel[i]*(sdword)controller_motor_params.ticks_2_rpms_factor)/delta_time;
	}	
	
//	printf("E %ld %ld %ld\n",encoder_rel[0], encoder_rel[1],encoder_rel[2]);
//	printf("P %u %u %lu\n",controller_motor_params.ticks_2_rpms_denum,delta_time);
//	printf("RPM %ld %ld %ld\n",controller_stat.actual_rpm[0], controller_stat.actual_rpm[1], controller_stat.actual_rpm[2]);
	
	//Next, Calculate Cartesian PathVector:

	//Each wheel velocity is 2pi*wheelradius
	//The calculation is the inverse of motion -> motor vector

	//	(Vx)                  (0  -1.5R 1.5R)  (v1)
	//	(Vy)      = 1/(3*t*R) (2tR  -tR  -tR)* (v2)
	//	(Vw)                  (t      t    t)  (v3)
	// with R robot radius and t = sin(120)
	// simplified:

	//	(Vx)        (0    -1/(2t)  1/(2t))  (v1)
	//	(Vy)    =   (2/3  -1/3    -1/3   )* (v2)
	//	(Vw)        (1/3R  1/3R    1/3R  )  (v3)

	// -sin(240) = sin(120)
	//cos(120)=cos(240)=-0.5 soo sweet
	//rpm and dezi millimeter /600 => mm/s
//TODO: do some optimisations:
	temp = controller_stat.actual_rpm[2]-controller_stat.actual_rpm[1];
	temp *= controller_motor_params.wheel_circ_64;
	temp /= SIN_120_8*2;
	temp /= 600; //60 seconds, 10 dezi millimeter
	controller_stat.actual_motion_vector.x = temp/8;	 //64/8

	temp = -(controller_stat.actual_rpm[1]+controller_stat.actual_rpm[2]);
	temp += 2*controller_stat.actual_rpm[0];
	temp *= controller_motor_params.wheel_circ_64;
	temp /= 1800l; //600*3
	controller_stat.actual_motion_vector.y = temp/64;

	temp = (controller_stat.actual_rpm[0]+controller_stat.actual_rpm[1]+controller_stat.actual_rpm[2]);	
	temp *= controller_motor_params.wheel_circ_64; 
	//ideally it would be:
//	temp *= (ROTATION_PRECISION/64);
//	temp /=	1800l; //3 (mittlerwert) * 60 (seconds) * 10 (dezi millimeter)
	//less subject to overflow:
//	temp *= (ROTATION_PRECISION/512); //8192 = 512*16 , 64 =16*4
//scaling already incorporated
	temp *= (ROTATION_PRECISION/4096);
	temp /=	225l; //3 (mittlerwert) * 60 (seconds) * 10 (dezi millimeter) /512 (ROTATION PRESCISION)
	temp *= 8;
	temp /= controller_motor_params.robot_radius;
	controller_stat.actual_motion_vector.rotation = temp;
	//rotation is scaled up by 64, which roughly equates to degrees per second btw


//Ringbuffer and smoothed values:

/*	for (i=0; i<3; i++) {
		rpm_ringbuff[odo_buff_pos][i] = controller_stat.actual_rpm[i];
	}
	odo_ringbuff[odo_buff_pos][0] = controller_stat.actual_motion_vector.x;
	odo_ringbuff[odo_buff_pos][1] = controller_stat.actual_motion_vector.y;
	odo_ringbuff[odo_buff_pos][2] = controller_stat.actual_motion_vector.rotation;
*/	
	for (i=0; i<3; i++) {		
		rpm_smoothed[i] = (controller_params_active.smooth*rpm_smoothed[i]+(8192-controller_params_active.smooth)*controller_stat.actual_rpm[i])/8192;
	}
	odo_smoothed[0] = (controller_params_active.smooth*odo_smoothed[0]+(8192-controller_params_active.smooth)*controller_stat.actual_motion_vector.x)/8192;
	odo_smoothed[1] = (controller_params_active.smooth*odo_smoothed[1]+(8192-controller_params_active.smooth)*controller_stat.actual_motion_vector.y)/8192;
	odo_smoothed[2] = (controller_params_active.smooth*odo_smoothed[2]+(8192-controller_params_active.smooth)*controller_stat.actual_motion_vector.rotation)/8192;	
//	if (++odo_buff_pos >= ODOBUFSIZE) odo_buff_pos = 0;

	if(controller_log_mode.log_enabled==1) {
	
		if (controller_log_mode.log[LOG_RPM] >= 0) { // actual rpm		
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPM]] = (sword)controller_stat.actual_rpm[0];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPM]+1] = (sword)controller_stat.actual_rpm[1];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPM]+2] = (sword)controller_stat.actual_rpm[2];
		}
		
		if (controller_log_mode.log[LOG_RPMGoal] >= 0) {// target rpm
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPMGoal]] = (sword)controller_stat.rpm_goal[0];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPMGoal]+1] = (sword)controller_stat.rpm_goal[1];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPMGoal]+2] = (sword)controller_stat.rpm_goal[2];
		}
		if (controller_log_mode.log[LOG_PWM] >= 0) {// pwm
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_PWM]]   = (sword)controller_stat.pwm[0];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_PWM]+1] = (sword)controller_stat.pwm[1];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_PWM]+2] = (sword)controller_stat.pwm[2];
		}
		if (controller_log_mode.log[LOG_MGoal] >= 0) {
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MGoal]]   = (sword)controller_stat.motion_goal.x;
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MGoal]+1] = (sword)controller_stat.motion_goal.y;
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MGoal]+2] = (sword)(controller_stat.motion_goal.rotation/(ROTATION_PRECISION/64));
		}
		if (controller_log_mode.log[LOG_MRequest] >= 0) {
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MRequest]]   = (sword)controller_stat.motion_request.x;
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MRequest]+1] = (sword)controller_stat.motion_request.y;
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MRequest]+2] = (sword)(controller_stat.motion_request.rotation/(ROTATION_PRECISION/64));
		}
		if (controller_log_mode.log[LOG_MaxPWM] >= 0) {
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MaxPWM]]   = (sword)controller_stat.maxPWM[0];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MaxPWM]+1] = (sword)controller_stat.maxPWM[1];
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MaxPWM]+2] = (sword)controller_stat.maxPWM[2];
		}
		if (controller_log_mode.log[LOG_ErrorInt] >= 0) {
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_ErrorInt]]   = (sword)(controller_stat.pid_eI[0]/8192l);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_ErrorInt]+1] = (sword)(controller_stat.pid_eI[1]/8192l);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_ErrorInt]+2] = (sword)(controller_stat.pid_eI[2]/8192l);
		}
		if (controller_log_mode.log[LOG_Motion] >= 0) {
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Motion]]   = (sword)(controller_stat.actual_motion_vector.x);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Motion]+1] = (sword)(controller_stat.actual_motion_vector.y);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Motion]+2] = (sword)(controller_stat.actual_motion_vector.rotation/(ROTATION_PRECISION/64));
		}
		if (controller_log_mode.log[LOG_MSmooth] >= 0) {
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MSmooth]]   = (sword)(odo_smoothed[0]);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MSmooth]+1] = (sword)(odo_smoothed[1]);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_MSmooth]+2] = (sword)(odo_smoothed[2]/(ROTATION_PRECISION/64));
		}
		if (controller_log_mode.log[LOG_RPMSmooth] >= 0) {
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPMSmooth]]   = (sword)(rpm_smoothed[0]);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPMSmooth]+1] = (sword)(rpm_smoothed[1]);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_RPMSmooth]+2] = (sword)(rpm_smoothed[2]);
		}
		

		//correction vector:
//		odometry_log[odolog_write_pos][6] = (sword)correction_vector[0];
//		odometry_log[odolog_write_pos][7] = (sword)correction_vector[1];
		
		if (controller_log_mode.log[LOG_Current] >= 0) {// motor currents. inverted if overtemperature flag set on H-Bridge
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Current]]    = (sword) motor_current[0];//getMotorCurrent(MOTOR1);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Current]+1]  = (sword) motor_current[1];//getMotorCurrent(MOTOR2);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Current]+2]  = (sword) motor_current[2];//getMotorCurrent(MOTOR3);
			#ifdef VMC
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Current]]   *= (isOvertemp(MOTOR1) ? -1 : 1);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Current]+1] *= (isOvertemp(MOTOR2) ? -1 : 1);
			odometry_log[odolog_write_pos][controller_log_mode.log[LOG_Current]+2] *= (isOvertemp(MOTOR3) ? -1 : 1);
			#endif
			
		}
		
		if (++odolog_write_pos >= ODO_LOG_SIZE) odolog_write_pos = 0;
	}


}

void cn_controller_recalc_derived_motor_params() {
	udword temp;
	float tempf;
	//normaliser = 2* ticks per half-rotation * gear denum / gear num
	//controller_motor_params.ticks_2_rpms_denum = (controller_motor_params.encoder_ticks*2*controller_motor_params.gear_denumerator) / controller_motor_params.gear_numerator;	
	temp = (controller_motor_params.encoder_ticks*2*controller_motor_params.gear_denumerator) / controller_motor_params.gear_numerator;	
	controller_motor_params.ticks_2_rpms_factor = TIME_TICKS_PER_MINUTE/temp;
	temp = controller_motor_params.wheel_radius;
	controller_motor_params.wheel_circ_64	   = temp * PI_128;
	tempf = controller_motor_params.wheel_radius*PI*8.0;
	controller_motor_params.wheel_circ_4 = (udword)ROUND(tempf);
}
void cn_controller_recalc_derived_controller_params() {
		
		controller_params_active.accelcurve_min = ((float)controller_params_active.accelbound_min);
  		controller_params_active.accelcurve_range = (((float)(controller_params_active.accelbound_max - controller_params_active.accelbound_min))/THIRD_PI);
		
}

/*
void cn_controller_prep_lookup() {
		float maxA, minA;
		minA = (float)controller_params_active.accelbound_min;
		minA *= (cycle_time);
		minA /= TIME_TICKS_PER_SECOND;		
		maxA = (float)controller_params_active.accelbound_max;
		maxA *= (cycle_time);
		maxA /= TIME_TICKS_PER_SECOND;		
		
		cn_calc_force_lookup(minA,maxA);
		//cn_calc_force_lookup(5.0,15.0);
}
*/

void cn_controller_set_final_pwm() {
	setMotorDirection(MOTOR1, controller_direct_pwm_final.dir[0]);
	setMotorDirection(MOTOR2, controller_direct_pwm_final.dir[1]);
	setMotorDirection(MOTOR3, controller_direct_pwm_final.dir[2]);
	setAllMotorPWM(controller_direct_pwm_final.pwm[0], controller_direct_pwm_final.pwm[1], controller_direct_pwm_final.pwm[2]);  

}

//-----------------------------------------------
// Fast arctan2
//http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
// For optimisations, below can be implemented with integers:
// error < 0.1 rad
/*
float arctan2(float y, float x)
{

   float abs_y = fabs(y)+1e-10;      // kludge to prevent 0/0 condition
   float r,angle;
   if (x>=0)
   {
      r = (x - abs_y) / (x + abs_y);
      angle = PI_FOURTH*(1 - r);
   }
   else
   {
      r = (x + abs_y) / (abs_y - x);
      angle = THREE_PI_FOURTH - PI_FOURTH * r;
   }
   if (y < 0)
   return(-angle);     // negate if in quad III or IV
   else
   return(angle);
}*/

