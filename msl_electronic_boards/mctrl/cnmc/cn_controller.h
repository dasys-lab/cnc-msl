#ifndef CN_CONTROLLER
#define CN_CONTROLLER

#include "main.h"
#include <math.h>
#include <stdio.h>

typedef struct cn_cval {
  sword kp;
  sword ki;
  sword b;
  sword kd;
  sword kdi;
  uword dead_band;
  sword rot_err_w;
  sword rot_err_accel_w;
  sword rot_err_velo_w;
  uword accelbound_min;
  uword accelbound_max;
  float accelcurve_min;
  float accelcurve_range;
  uword fail_safe_rpm_bound;
  uword fail_safe_pwm_bound;
  uword fail_safe_cycles;
} controller_params;

typedef struct cn_cartMotionVector {
	sdword x;
	sdword y;
	sdword rotation;  
} cart_motion_vector;

typedef struct cn_motorConfig  {
	uword maxRPM;
	ubyte nominal_current;
	ubyte gear_numerator;
	ubyte gear_denumerator;
	uword wheel_radius;		//in 0.1 mm
	uword robot_radius;	   //in mm
	uword encoder_ticks;
	uword ticks_2_rpms_denum; //ticks / ticks_2_repms_denum = rotation per millisecond
	udword wheel_circ_64;

} motor_params;

typedef struct cn_controllerStatus {   //the state of the controller, more to come
	sdword actual_rpm[3];   					//actual rotations per minute
	sdword last_rpm[3];
	cart_motion_vector actual_motion_vector;	//what we are travelling at, aka odometry info
	cart_motion_vector motion_request;			//what we were told to travel at
	cart_motion_vector motion_goal;				//what we next aim for
	sdword rpm_goal[3];							//what we aim for in rpm, PID control signal
	sdword pid_e[3];
	sdword pid_eI[3];
	sdword pid_eD[3];
	sword pwm[3];
	sdword rotation_error;	
} controller_status;													   

typedef struct cn_controller_pwm_vector {
  
	ureg pwm[3];
	bool dir[3];
	
} controller_pwm_vector;

#define CONTROLLER_MODE_LOWEST 0
#define CONTROLLER_MODE_DIRECT 0
#define CONTROLLER_MODE_PID    1
#define CONTROLLER_MODE_HIGHEST 1

#define PI_128 402l
#define PI_256 804l

#define SIN_120_64 55l
#define SIN_120_8 7l

//TODO: verify this is all calculated at compile time
#define PI 3.1415926535897932
#define TWO_PI (2*PI)
#define SIXTH_PI_4 (2*PI/3)
#define SIXTH_PI_8 (4*PI/3)
#define THIRD_PI (PI/3)
#define PI_FOURTH (PI/4)
#define THREE_PI_FOURTH (3*PI/4)

#define CN_CONTROLLER_CURRENT_DEFAULT_MAX 750
#define CN_CONTROLLER_CURRENT_DEFAULT_NOM 100

#define ODO_LOG_SIZE 14000
#define ODO_LOG_NUM_ELEMENTS 9

extern uword cn_controller_current_max;
extern uword cn_controller_current_nom;

extern controller_params controller_params_active;
extern controller_params controller_params_passive;
extern bool              controller_params_commit;
extern ubyte             controller_mode;

extern controller_pwm_vector controller_direct_pwm_target;
extern controller_pwm_vector controller_direct_pwm_final;

extern motor_params controller_motor_params;
extern controller_status controller_stat;
extern bool controller_log_odometry;
extern sword xhuge odometry_log[][ODO_LOG_NUM_ELEMENTS];


extern uword controller_error_word;

#define cn_controller_has_error()    (controller_error_word != 0)
#define cn_controller_is_error(x)    (controller_error_word & (x))
#define cn_controller_set_error(x)   (controller_error_word |= (x))
#define cn_controller_reset_error(x) (controller_error_word ^= (controller_error_word & (x)))

#define CN_CONTROLLER_ERROR_TIMEOUT 1
#define CN_CONTROLLER_ERROR_CURRENT 2

void cn_controller_init();
void cn_controller_reset();
void cn_controller_update_params();
void cn_controller_emergency_stop();
void cn_controller_main_control();
void cn_controller_current_control();
void cn_controller_control_direct();
void cn_controller_control_pid();
void cn_controller_recalc_derived_motor_params();
void cn_controller_recalc_derived_controller_params();    
void cn_controller_set_final_pwm();
void cn_controller_calc_odometry();
void cn_controller_prep_lookup();
float arctan2(float y, float x);
#endif