#ifndef settings_h
#define settings_h 1

#include <SystemConfig.h>
#include <Configuration.h>

#define PI 3.141592654
#define TWO_PI 6.283185308

using supplementary::SystemConfig;
using supplementary::Configuration;


typedef struct {
    int communication_timeout;

    //--------------motor specific:
    int gear_ratio_nominator;
    int gear_ratio_denominator;


    //int opMode=-1; //CONTMODE or VOLTMODE
    //int controlMode; //Position (M) or RPM (V)


    int max_deviation;

    int max_amp; //mA
    int max_continous_amp; //mA
    int thermalConstantWinding;
    int maxRPM;
    int max_accel; //r/s^2
    int max_deccel; //r/s^2
    int enabled; //not used atm

    //--------------OmniDrive specific:
    double definingAngle;
    //int gear_denum;
    //int gear_num;
    double wheelRadius;
    double robotRadius;

	int odometrySamplingTime;
	int controllerLoopTime;

	int nodeGuardTime;
	int errorRestTime;
	int commandTimeout;

	double maxTranslation;
	double maxRotation;

	int slipControlEnabled;
	double slipControlP;
	double slipControlI;
	double slipControlDecay;

	int rotationControlEnabled;
	double rotationControlP;
	double rotationControlI;
	double rotationControlD;
	double maxRotationErrorInt;
	double rotationControlByVeloP;
	
	
	int newGearHack;

} controller_settings;




void settings_init();

#endif
