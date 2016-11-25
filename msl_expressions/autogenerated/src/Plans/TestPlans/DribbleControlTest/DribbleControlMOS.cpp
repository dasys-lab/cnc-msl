using namespace std;
#include "Plans/TestPlans/DribbleControlTest/DribbleControlMOS.h"

/*PROTECTED REGION ID(inccpp1479905178049) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "RawSensorData.h"
#include "MSLWorldModel.h"
#include "math.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1479905178049) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleControlMOS::DribbleControlMOS() :
            DomainBehaviour("DribbleControlMOS")
    {
        /*PROTECTED REGION ID(con1479905178049) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DribbleControlMOS::~DribbleControlMOS()
    {
        /*PROTECTED REGION ID(dcon1479905178049) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleControlMOS::run(void* msg)
    {
        /*PROTECTED REGION ID(run1479905178049) ENABLED START*/ //Add additional options here

    	//---
    	if (testCount >= 60) {
    		testCount = 0;
    		testAngle += M_PI/2 + M_PI/12;
    	}
    	msl_actuator_msgs::MotionControl motorMsg;
    	motorMsg.motion.angle = testAngle;
    	motorMsg.motion.rotation = testRot;
    	motorMsg.motion.translation = testSpeed;
    	send(motorMsg);
    	testCount++;
    	//---
//    	auto odom = wm->rawSensorData->getOwnVelocityMotion();
//
//    	auto robotAngle = odom->angle;
//    	auto robotVel = odom->translation;
//    	auto robotRot = odom->rotation;
//
//    	auto ballVel = getBallVelocity(robotAngle,robotVel,robotRot);
//    	auto ballAngle = getBallAngle(robotAngle,robotVel,robotRot);
//
//    	auto right = getRightArmVelocity(ballVel,ballAngle);
//    	auto left = getLeftArmVelocity(ballVel,ballAngle);
//
//    	msl_actuator_msgs::BallHandleCmd msgback;
//    	msgback.leftMotor = left;
//    	msgback.rightMotor = right;
//    	send(msgback);

    	//cout<<robotAngle<<"  "<<robotVel<<"  "<<robotRot<<"  "<<ballVel<<"  "<<ballAngle<<"  "<<left<<" "<<right<<endl;
        /*PROTECTED REGION END*/
    }
    void DribbleControlMOS::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1479905178049) ENABLED START*/ //Add additional options her
    	testSpeed = 2000;
    	testAngle = -M_PI;
    	testRot = 0;
    	testCount= 0;

    	velToInput = -1.77; //calibration desired velocity to input voltage of motors
    	rBallRobot = 300; //distance between robot rotation axis and ball center
    	epsilonT = 0.1; //change between robot velocity angle and desired ball velocity angle
    	epsilonRot = 0.2; //change between ball velocity angle through rotation and desired ball velocity angle
    	phi = M_PI/6; //horizontal angle between y and arm
    	forwConst = sqrt((sin(M_PI/2-0.82))*sin(M_PI/2-0.82)+(sin(0.75)*cos(M_PI/2-0.82))*(sin(0.75)*cos(M_PI/2-0.82)))/cos(0.349); //ballVel -> ArmVel for forward
    	sidewConst = sin(0.559)/sin(0.438); //ballVel -> ArmVel for sideways
    	diagConst = sin(1.18)/(cos(0.349)*sin(0.82)); //ballVel -> ArmVel for diagonal

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1479905178049) ENABLED START*/ //Add additional methods here


    double DribbleControlMOS::getBallVelocity(double angle, double translation, double rotation)
    {
    	double velX = -cos(angle)*translation;
    	double velY = -sin(angle)*translation + rotation*rBallRobot;
    	//correcting desired ball velocity towards robot to guarantee grib
    	velX -= epsilonT*abs(translation) + epsilonRot*abs(rotation)*rBallRobot;

    	return sqrt(velX*velX+velY*velY);
    }

    //returns values [pi,-pi[
    double DribbleControlMOS::getBallAngle(double angle, double translation, double rotation)
    {
        	double velX = -cos(angle)*translation;
        	double velY = -sin(angle)*translation + rotation*rBallRobot;
        	velX -= epsilonT*abs(translation) + epsilonRot*abs(rotation)*rBallRobot;

        	double ballAngle =0;
        	if (velX==0)
        	{
        		if (velY>=0)
        			ballAngle = M_PI/2;
        		else
        			ballAngle = -M_PI/2;
        	}
        	else if (velX > 0)
        	{
        		ballAngle = atan(velY/velX);
        	}
        	else
        	{
        		if (velY>=0)
        			ballAngle = atan(velY/velX)+M_PI;
        		else
        			ballAngle = atan(velY/velX)-M_PI;
        	}

        	return ballAngle;
    }

    double DribbleControlMOS::getLeftArmVelocity(double ballVelocity, double ballAngle)
    {
    	double sec0 = - M_PI;
    	double sec1 = - M_PI/2 - phi;
    	double sec2 = - M_PI/2;
    	double sec3 = - M_PI/2 + phi;
    	double sec4 = 0;
    	double sec5 = M_PI/2 - phi;
    	double sec6 = M_PI/2;
    	double sec7 = M_PI/2 + phi;
    	double sec8 = M_PI;

    	double angleConst = 0;

    	//linear interpolation of the constants in the 8 sectors

    	if (ballAngle <= sec1) {
    		angleConst = (ballAngle-sec1)*forwConst / (sec1-sec0);
    	}
    	else if (ballAngle <= sec2) {
    		angleConst = (ballAngle-sec1)*sidewConst / (sec2-sec1);
    	}
    	else if (ballAngle <= sec3) {
       		angleConst = sidewConst+(ballAngle-sec2)*(diagConst-sidewConst) / (sec3-sec2);
    	}
    	else if (ballAngle <= sec4) {
    	   		angleConst = diagConst+(ballAngle-sec3)*(forwConst-diagConst) / (sec4-sec3);
    	}
    	else if (ballAngle <= sec5) {
    	   		angleConst = forwConst+(ballAngle-sec4)*(-forwConst) / (sec5-sec4);
    	}
    	else if (ballAngle <= sec6) {
    	   		angleConst = (ballAngle-sec5)*(-sidewConst) / (sec6-sec5);
    	}
    	else if (ballAngle <= sec7) {
    	   		angleConst = -sidewConst+(ballAngle-sec6)*(sidewConst-diagConst) / (sec7-sec6);
    	}
    	else if (ballAngle >= sec7) {
    	   		angleConst = -diagConst+(ballAngle-sec7)*(-forwConst+diagConst) / (sec8-sec7);
    	}

    	return ballVelocity * angleConst * velToInput;
    }

    double DribbleControlMOS::getRightArmVelocity(double ballVelocity, double ballAngle)
    {
    	double sec0 = - M_PI;
    	double sec1 = - M_PI/2 - phi;
    	double sec2 = - M_PI/2;
    	double sec3 = - M_PI/2 + phi;
    	double sec4 = 0;
    	double sec5 = M_PI/2 - phi;
    	double sec6 = M_PI/2;
    	double sec7 = M_PI/2 + phi;
    	double sec8 = M_PI;

    	double angleConst = 0;

    	//linear interpolation of the constants in the 8 sectors

    	if (ballAngle <= sec1) {
    		angleConst = -forwConst+(ballAngle-sec0)*(-diagConst+forwConst) / (sec1-sec0);
    	}
    	else if (ballAngle <= sec2) {
    		angleConst = -diagConst+(ballAngle-sec1)*(-sidewConst+diagConst) / (sec2-sec1);
    	}
    	else if (ballAngle <= sec3) {
       		angleConst = -sidewConst+(ballAngle-sec2)*(sidewConst) / (sec3-sec2);
    	}
    	else if (ballAngle <= sec4) {
    	   		angleConst = (ballAngle-sec3)*forwConst / (sec4-sec3);
    	}
    	else if (ballAngle <= sec5) {
    	   		angleConst = forwConst+(ballAngle-sec4)*(diagConst-forwConst) / (sec5-sec4);
    	}
    	else if (ballAngle <= sec6) {
    	   		angleConst = diagConst+(ballAngle-sec5)*(sidewConst-diagConst) / (sec6-sec5);
    	}
    	else if (ballAngle <= sec7) {
    	   		angleConst = sidewConst+(ballAngle-sec6)*(-sidewConst) / (sec7-sec6);
    	}
    	else if (ballAngle >= sec7) {
    	   		angleConst = (ballAngle-sec7)*(-forwConst) / (sec8-sec7);
    	}

    	return ballVelocity * angleConst * velToInput;
    }



/*PROTECTED REGION END*/
} /* namespace alica */
