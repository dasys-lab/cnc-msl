#include "settings.h"
//#include "MCDC.h"


#include <string>

controller_settings default_settings;
controller_settings current_settings;



SystemConfig* sc;


void settings_init() {
    default_settings.communication_timeout = 100;

    default_settings.gear_ratio_denominator=7;
    default_settings.gear_ratio_nominator=1;

    default_settings.maxRPM = 6700;//5520; //rpm
    default_settings.max_accel=30;
    default_settings.max_amp= 10000; //UNKNOWN FOR 3557CR
    default_settings.max_continous_amp=3800; //UNKNOWN FOR 3557CR
    default_settings.thermalConstantWinding = 8;
    default_settings.max_deccel=30;

    default_settings.definingAngle = 40.0*PI /180.0; //not read from config
    default_settings.wheelRadius = 65;
    default_settings.robotRadius = 280;
	default_settings.odometrySamplingTime = 10;
	default_settings.controllerLoopTime = 10;//*1000;

	default_settings.nodeGuardTime = 1000;
	
	default_settings.newGearHack = -1;

	sc = SystemConfig::getInstance();

	Configuration *motion = (*sc)["Motion"];

	
	default_settings.communication_timeout = motion->get<int>("Motion","Connection","Timeout",NULL);

    default_settings.gear_ratio_denominator = motion->get<int>("Motion","Motors","GearRatioDenum",NULL);
    default_settings.gear_ratio_nominator = motion->get<int>("Motion","Motors","GearRatioNom",NULL);
    default_settings.maxRPM = motion->get<int>("Motion","Motors","MaxRPM",NULL);
    default_settings.max_accel = motion->get<int>("Motion","MotorControl","MaxAccelRPM",NULL);
    default_settings.max_amp = motion->get<int>("Motion","AmpControl","MaxAmp",NULL);
    default_settings.max_continous_amp = motion->get<int>("Motion","AmpControl","MaxContinousAmp",NULL);
    
    default_settings.thermalConstantWinding = motion->get<int>("Motion","AmpControl","ThermalConstantWinding",NULL);
    
    default_settings.max_deccel = motion->get<int>("Motion","MotorControl","MaxDeccelRPM",NULL);


	default_settings.odometrySamplingTime = motion->get<int>("Motion","MotionControl","OdometrySamplingTime",NULL);
	default_settings.controllerLoopTime = motion->get<int>("Motion","MotionControl","ControllerLoopTime",NULL);

	default_settings.nodeGuardTime = motion->get<int>("Motion","MotorControl","NodeGuardTime",NULL);
	default_settings.errorRestTime = motion->get<int>("Motion","MotorControl","ErrorRestTime",NULL);
	default_settings.commandTimeout = motion->get<int>("Motion","MotorControl","ErrorRestTime",NULL);

	default_settings.maxTranslation = motion->get<double>("Motion","MotionControl","MaxTranslation",NULL);
	default_settings.maxRotation = motion->get<double>("Motion","MotionControl","MaxRotation",NULL);

	default_settings.slipControlEnabled = motion->get<bool>("Motion","SlipControl","Enabled",NULL);
	default_settings.slipControlP = motion->get<double>("Motion","SlipControl","Proportional",NULL);
	default_settings.slipControlI = motion->get<double>("Motion","SlipControl","Integral",NULL);
	default_settings.slipControlDecay = motion->get<double>("Motion","SlipControl","IntegralDecay",NULL);


	default_settings.rotationControlEnabled = motion->get<bool>("Motion","RotationControl","Enabled",NULL);
	default_settings.maxRotationErrorInt = motion->get<double>("Motion","RotationControl","maxIntegral",NULL)*1024.0;
	default_settings.rotationControlP = motion->get<double>("Motion","RotationControl","Proportional",NULL);
	default_settings.rotationControlI = motion->get<double>("Motion","RotationControl","Integral",NULL);
	default_settings.rotationControlD = motion->get<double>("Motion","RotationControl","Derivative",NULL);
	default_settings.rotationControlByVeloP = motion->get<double>("Motion","RotationControl","PropByVelocity",NULL);
	
	default_settings.newGearHack = motion->tryGet<int>(-1,"Motion","NewGearHack",NULL);

//   std::string traceData = motion->get<std::string>("Motion","Logging","AdditionalTraceData",NULL);
    /*
    if(traceData.find("Current")==0) {
        default_settings.additionalTraceData = MCDC_TRACE_CURRENT;//MCDC_TRACE_PWM; //pwm
    } else if(traceData.find("PWM")==0) {
        default_settings.additionalTraceData = MCDC_TRACE_PWM;
    }   else if(traceData.find("None")==0) {
        default_settings.additionalTraceData = MCDC_TRACE_NONE;
    } else if(traceData.find("targetRPM")==0) {
        default_settings.additionalTraceData = MCDC_TRACE_TARGET_RPM;
    } else if(traceData.find("HouseTemp")==0) {
        default_settings.additionalTraceData = MCDC_TRACE_HOUSE_TEMP;
    } else if(traceData.find("CoilTemp")==0) {
        default_settings.additionalTraceData = MCDC_TRACE_COIL_TEMP;
    } else {
        printf("Unknown Additional Trace Data\n");
        exit(-1);
    }*/


    default_settings.wheelRadius = motion->get<double>("Motion","MotionControl","WheelRadius",NULL);
    default_settings.robotRadius = motion->get<double>("Motion","MotionControl","RobotRadius",NULL);



    current_settings = default_settings;


}
