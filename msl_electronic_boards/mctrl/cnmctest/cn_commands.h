#include "main.h"

// protocol definition			  
#define GROUP_CONFIGURE           0x50
#define GROUP_CONFIGURE_RESPONSE  0x51
								  
#define CMD_SET_MODE              0x10 //implemented
#define CMD_SET_CYCLE_TIME        0x11 //implemented
#define CMD_CURRENT_LIMIT_ON      0x12
#define CMD_COMMAND_TIMEOUT       0x13
#define CMD_MAX_CURRENT           0x20
#define CMD_MAX_RPM               0x21 //implemented
#define CMD_NOMINAL_CURRENT       0x22
#define CMD_NOMINAL_RPM           0x23
#define CMD_GEAR_RATIO            0x30 //implemented
#define CMD_WHEEL_RADIUS          0x31 //implemented
#define CMD_TICKS_PER_ROTATION    0x33 //implemented
#define CMD_MOTOR_TORQUE          0x34
#define CMD_MOTOR_DIRECTION       0x35
#define CMD_MOTOR_MAX_TEMP        0x40
#define CMD_MOTOR_NOMINAL_TEMP    0x41
#define CMD_ENVIRONMENT_TEMP      0x42
#define CMD_MOTOR_WINDING_TEMP    0x43
#define CMD_MOTOR_WINDING_GN      0x44
#define CMD_MOTOR_WINDING_GT      0x45
#define CMD_MOTOR_CHASSIS_TEMP    0x46
#define CMD_MOTOR_CHASSIS_GN      0x47
#define CMD_MOTOR_CHASSIS_GZ      0x48
#define CMD_SAVE_CONFIG           0x50
#define CMD_LOAD_CONFIG           0x51
#define CMD_IO_PORT_CONF          0x60
#define CMD_ROBOT_RADIUS          0x70 //implemented
#define CMD_WHEEL_ANGLE           0x71
#define CMD_TOGGLE_ODO_LOG		  0x72 //implemented
#define CMD_SET_LOG_MODE		  0x73 //implemented


#define GROUP_CONTROL             0x52
								  
#define CMD_SET_ALL_PWM           0x10
#define CMD_SET_PWM               0x11
#define CMD_SET_ALL_DIRECTION     0x12
#define CMD_SET_DIRECTION         0x13
#define CMD_SET_ALL_RPM           0x20
#define CMD_SET_RPM               0x21
#define CMD_SET_MOTION_VECTOR     0x30 //implemented
#define CMD_RESET_HARD            0x40
#define CMD_RESET_CONTROLLER      0x41
								  
#define GROUP_REQUEST             0x54
#define GROUP_REQUEST_RESPONSE    0x55
								  
#define CMD_MOTOR_RPM             0x10
#define CMD_MOTOR_PWM             0x11
#define CMD_MOTOR_VOLTAGE         0x12
#define CMD_MOTOR_CURRENT         0x13
#define CMD_MOTOR_TEMP            0x14
#define CMD_TORQUE_DE_MOTOR       0x15
#define CMD_MOTOR_POWER           0x16
#define CMD_ENCODER_REL           0x20
#define CMD_ENCODER_ABS           0x21
#define CMD_BATTERY_VOLTAGE       0x30
#define CMD_READ_IO               0x40
#define CMD_SET_IO                0x41
#define CMD_READ_ANA              0x42
#define CMD_MOTION_VECTOR         0x60 
#define CMD_AVERAGE_SLEEP_TIME    0x61
#define CMD_PATH_VECTOR           0x62 //implemented
#define CMD_READ_ODO_LOG		  0x63 //implemented

									 
#define GROUP_CONTROL_CONFIG      0x56
#define GROUP_CONTROL_CONFIG_RES  0x57

#define CMD_PID_KP                0x10	//implemented
#define CMD_PID_KI                0x11	//implemented
#define CMD_PID_B                 0x12  //implemented
#define CMD_PID_KD                0x13	//implemented
#define CMD_PID_KDI               0x14  //implemented
#define CMD_CONTROLLER_COMMIT     0x15  //implemented
#define CMD_DEAD_BAND             0x2A  //implemented
#define CMD_MAX_ERROR_INT         0x2B  //implemented
#define CMD_PID_LIN				  0x2C  //implemented
#define CMD_SMOOTH				  0x2D  //implemented
#define CMD_MAX_ROTATION_ACCEL	  0x2E  //implemented
#define CMD_ROTATION_ERR_W        0x31  //implemented
#define CMD_ROTATION_ERR_ACCEL_W  0x32  //implemented
#define CMD_ROTATION_ERR_VELO_W   0x33  //implemented
#define CMD_ACCEL_BOUND_CURVE_MIN 0x40  //implemented
#define CMD_ACCEL_BOUND_CURVE_MAX 0x41  //implemented
#define CMD_ACCEL_CURVE_DEGREE    0x42
#define CMD_FAIL_SAFE_VALUES      0x50  //implememted
#define CMD_CURRENT_ERROR_BOUND	  0x60  //implememted
#define CMD_CURRENT_KP			  0x61  //implememted
#define CMD_CURRENT_KI			  0x62  //implememted
#define CMD_CURRENT_KD			  0x63  //implememted
 

#define GROUP_ERROR_RESPONSE      0x59

#define CMD_UNKNOWN_ERROR         0x01
#define CMD_UNKNOWN_CMD			  0x02
#define CMD_PARAMETER_ERROR       0x10
#define CMD_OUT_OF_RANGE          0x11
#define CMD_CYCLE_OVERTIME        0x20

// range defines
#define MAX_PWM_RANGE 1000

// command functions
void cn_command_unknown(void);

// cn_command_<group>_<command>
void cn_command_configure_set_mode(ubyte* mode);
void cn_command_configure_set_cycle_time(ubyte* milsec);
void cn_command_configure_command_timeout(ubyte milsec);
void cn_command_configure_max_current(ubyte current);
void cn_command_configure_nom_current(ubyte current);
void cn_command_configure_max_rpm(uword* rpm);
void cn_command_configure_nom_rpm(uword rpm);
void cn_command_configure_gear_ratio(ubyte* numerator, ubyte* denominator);
void cn_command_configure_wheel_radius(uword* radius);
void cn_command_configure_ticks_per_rotation(uword* ticks);
void cn_command_configure_motor_direction(ubyte bitfield);
void cn_command_configure_robot_radius(uword* radius);
void cn_command_configure_toggle_odometry_log(ubyte* toggle);
void cn_command_configure_controller_log(sbyte* data);

void cn_command_control_set_all_pwm(sword val1, sword val2, sword val3);
void cn_command_control_set_all_rpm(sword rpm1, sword rpm2, sword rpm3);
void cn_command_control_set_motion_vector(sword x, sword y, sword rotation);
void cn_command_control_reset_hard();
void cn_command_control_reset_controller();

void cn_command_request_motor_rpm();
void cn_command_request_motor_pwm();
void cn_command_request_motor_current();
void cn_command_request_encoder_relative();
void cn_command_request_battery_voltage();
void cn_command_request_motion_vector();
void cn_command_request_average_sleep_time();
void cn_command_request_path_vector();
void cn_command_request_odometry_log(uword pos);

void cn_command_controlconf_pid_kp(sword* kp);
void cn_command_controlconf_pid_ki(sword* ki);
void cn_command_controlconf_pid_b(sword* b);
void cn_command_controlconf_pid_kd(sword* kd);
void cn_command_controlconf_pid_kdi(sword* kdi);
void cn_command_controlconf_pid_lin(sword* lin);
void cn_command_controlconf_smooth(sword* smooth);
void cn_command_controlconf_max_rotation_accel(uword* maxRot);
void cn_command_controlconf_controller_commit();
void cn_command_controlconf_dead_band(uword* dead_band);
void cn_command_controlconf_max_error_int(uword* max_error);	
void cn_command_controlconf_rot_err_w(uword* weight);
void cn_command_controlconf_rot_err_accel_w(uword* weight);
void cn_command_controlconf_rot_err_velo_w(uword* weight);
void cn_command_controlconf_accel_bound_curve_min(uword* bound);
void cn_command_controlconf_accel_bound_curve_max(uword* bound);
void cn_command_controlconf_accel_curve_degree(float degree);
void cn_command_controlconf_fail_safe_values(uword* rpm_bound, uword* pwm_bound, uword* timeout);
void cn_command_controlconf_current_error_bound(sword* bound);
void cn_command_controlconf_current_kp(sword* kp);
void cn_command_controlconf_current_ki(sword* ki);
void cn_command_controlconf_current_kd(sword* kd);
