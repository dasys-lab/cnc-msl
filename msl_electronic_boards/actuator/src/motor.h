#ifndef MOTOR_H__
#define MOTOR_H__

void motor_init(void);

#define WAIT_TIME_DIR_CHANGE 25000
#define WAIT_TIME_CONTROLLER 1000
#define PWM_STEP_SIZE 14

extern uint32_t last_heartbeat;
uint8_t request_position;
int8_t request_dir;



void     ballhandler_set(int16_t left, int16_t right);
void     stop_control(void);
void	 ballhandler_control(void);
void     stop_set(uint8_t pos);
void	 servo_set(uint8_t pos);
void	check_motors(uint32_t time);

#endif
