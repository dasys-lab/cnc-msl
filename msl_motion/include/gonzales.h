#ifndef gonzales_h
#define gonzales_h 1


#include "settings.h"
#include "util.h"
#include "driver/eposcan.h"


#define GONZ_RUNNING 1
#define GONZ_INITIALISING 2
#define GONZ_ERROR 3
#define GONZ_STOPPED 0

#define GONZ_REST_ON_ERROR_MS 500


#define GONZ_MODE_NORMAL 0
#define GONZ_MODE_TEST 1




extern Controlling::EposCan *ep;
extern controller_settings current_settings;

typedef struct  {
	double angle; //in rad/1024
	double velocity; //in mm or mm/s
	double rotation; //in rad/s/1024
} polarMotion;

typedef struct  {
	double x; //in mm or mm/s
	double y; //in mm or mm/s
	double rotation; //in rad/s/1024
} cartMotion;
typedef double wheelMotion[4];

typedef struct {
    double x;
    double y;
    double angle;
} position;

typedef struct {
	int state;
	int autorecover;
	int recover_timer;
	int newOdometryAvailable;
	cartMotion currentMotionGoal;
	wheelMotion currentMotorGoal;
	cartMotion actualMotion;
	position currentPosition;
	double currentSlip;
	double slipI;
	double currentRotationError;
	double lastRotationError;
	double rotationErrorInt;
	
} gonzales_state;



//void gonz_handleMCDCError(unsigned char nodeid,int errorCode);
void gonz_emergency_stop();
void gonz_init();
void gonz_main();
void gonz_idle();
void gonz_control();
void gonz_recover_from_error();
void gonz_set_error_state(int mayrecover,int timetorecover);
void gonz_reset();
void gonz_update_derived_settings();
void gonz_send_cmd();
void gonz_calc_odometry();
void gonz_send_odometry();
void gonz_notify_odometry();

void gonz_set_motion_request(double angle, double trans, double rot);
int gonz_check_status();


void gonz_set_mode(int mode);
int gonz_get_mode();

void gonz_test_loop();
#endif
