#include "ErrorMinLocalization.h"
#include "BallIntegrator.h"
#include "SpicaHelper.h"
#include "TimeHelper.h"
#include "EgoMotionEstimator.h"
#include <msl_msgs/MotionInfo.h>
#include <msl_msgs/PositionInfo.h>
#include <msl_sensor_msgs/LocalizationType.h>

using namespace msl_msgs;


namespace redwolf {

	ErrorMinLocalization::ErrorMinLocalization()
	{
		rawOdometryHelper = RawOdometryHelper::getInstance();

		fieldLines = getLines();

		fieldCircles = getCircles();
		coxLoc = new Cox::CoxLocalization(fieldLines, fieldCircles, 10.f, 600.0, 20.0, 20.0, 0.01);
		Position bufferInitPosition;
		bufferInitPosition.x = 0.0;
		bufferInitPosition.y = 0.0;
		bufferInitPosition.heading = 0.0;

		for(int i = 0; i < RAWODOBUFSIZE; i++){
			positionBuffer[i] = bufferInitPosition;
			timestampBuffer[i] = 0;
		}

		integrationIndex = 0;
		bufferInitialized = false;
		lastIteration = 0;
	}

	Cox::field_pose_t ErrorMinLocalization::pos2pose(Position pos)
	{
		Cox::field_pose_t pose(pos.x,pos.y,pos.heading);
		return pose;
	}
	Position ErrorMinLocalization::pose2pos(Cox::field_pose_t pose)
	{
		Position pos;
		pos.x = pose.x;
		pos.y = pose.y;
		pos.heading = pose.phi;
		return pos;
	}

	void ErrorMinLocalization::linePoint2linePointst(std::vector<LinePoint> lp, Cox::line_points_t & linepoints)
	{
		for(unsigned int i = 0; i < lp.size(); i++){
			linepoints.push_back(Cox::line_point_t(lp[i].x,lp[i].y));
		}
	}

	//double calculateWeightForEstimatedPosition(Position pos, std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, unsigned char * linePointsInvalidity, int invCounter);

//	CorrectedOdometryInfoPtr coi = CorrectedOdometryInfo::create();
	WeightedPosition ErrorMinLocalization::estimatedPosition(Position curPos, std::vector<LinePoint> lp, LineDistanceHelper & lineDistHelper)
	{
		//weighted position
		WeightedPosition weightPos;
		Cox::line_points_t detectedLP;
		linePoint2linePointst(lp, detectedLP);

		Cox::field_pose_t pose = pos2pose(curPos);

		Position pos1 = rawOdometryHelper->getPositionData(lastIteration);
		Position pos2 = rawOdometryHelper->getPositionData();

		Position updatePos;
		updatePos.x = pos2.x - pos1.x;
		updatePos.y = pos2.y - pos1.y;
		updatePos.heading = pos2.heading - pos1.heading;
		if(updatePos.heading > M_PI) updatePos.heading -= 2.0*M_PI;
		if(updatePos.heading < -M_PI) updatePos.heading += 2.0*M_PI;

		//updatePos.x = 0.0;
		//updatePos.y = 0.0;
		//updatePos.heading = 0.0;

		Cox::field_pose_t rawPose = pos2pose(updatePos);
		float dphi = pose.phi + rawPose.phi;
		float dr = rawPose.x;
		float ds = rawPose.y;

//		float phi2 = pose.phi - rawPose.phi;
		if(dphi < -M_PI)
			dphi += 2.0*M_PI;
		if(dphi > M_PI)
			dphi -= 2.0*M_PI;

		pose.x +=  dr * cos(dphi) + ds * -sin(dphi);
		pose.y +=  dr * sin(dphi) + ds * cos(dphi);
		pose.phi = dphi;
//		pose.phi += rawPose.phi;
		if(pose.phi > M_PI)
			pose.phi -= 2.0*M_PI;
		if(pose.phi < -M_PI)
			pose.phi += 2.0*M_PI;
		Cox::field_pose_t oldPos = pose;
		coxLoc->get_pose(pose, detectedLP );

		//if (pose.x > FootballField::FieldLength/2+maxOverLines || pose.x< -FootballField::FieldLength/2-maxOverLines || pose.y > FootballField::FieldWidth/2+maxOverLines || pose.y < -FootballField::FieldWidth/2-maxOverLines) coxLoc->global_localization(oldPos, detectedLP);
		Position newPosition = pose2pos(pose);

		// initialize buffer (ball velocity)
		if(bufferInitialized){
			integrationIndex++;
			if(integrationIndex >= RAWODOBUFSIZE)
				integrationIndex -= RAWODOBUFSIZE;

		}
		else {
			bufferInitialized = true;
		}

		positionBuffer[integrationIndex] = newPosition;
		timestampBuffer[integrationIndex] = TimeHelper::getInstance()->getVisionTimeOmniCam();

		EgoMotionEstimator * estimator = EgoMotionEstimator::getInstance();

		mrOld = mr;

		mr = estimator->trackObject(positionBuffer, timestampBuffer, RAWODOBUFSIZE, integrationIndex, 0.4E07);



		PositionInfo robotPosition;
		MotionInfo robotVelocity;

		robotPosition.x = (newPosition.x);
		robotPosition.y = (newPosition.y);
		robotPosition.angle = (newPosition.heading);


		unsigned long long timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();

		int logTime = (int) ((timestamp/1000) % 1000000);
		if(logTime > 500000)
			logTime -= 1000000;


		robotVelocity.translation = (sqrt(mr.velocity.vx*mr.velocity.vx + mr.velocity.vy*mr.velocity.vy));
		robotVelocity.rotation = (mr.velocity.w);
		robotVelocity.angle = (atan2(mr.velocity.vy, mr.velocity.vx));





		std::vector<LinePoint>::const_iterator first, last = lp.end();

		int invCounter = 0;
		unsigned char * linePointsInvalidity = (unsigned char *) malloc(lp.size());
		memset((void *) linePointsInvalidity, 0, lp.size());

		if((fabs(newPosition.x) > FootballField::FieldLength/2.0 - 2000.0 || fabs(newPosition.y) > FootballField::FieldWidth/2.0 - 2000.0))
		{


			double cos_ = cos(newPosition.heading);
			double sin_ = sin(newPosition.heading);

			int invIndex = 0;


			for(first = lp.begin(); first != last; ++first)
			{



				double realx = newPosition.x + cos_*(first->x) - sin_*(first->y);
				double realy = newPosition.y + sin_*(first->x) + cos_*(first->y);
				if(fabs(realx) > (FootballField::FieldLength/2.0 + 800.0) || fabs(realy) > (FootballField::FieldWidth/2.0 + 800.0) ){
					linePointsInvalidity[invIndex] = 1;
					invCounter++;


				}
				invIndex++;

			}

		}


		weightPos.x = newPosition.x;
		weightPos.y = newPosition.y;
		weightPos.heading = newPosition.heading;
		weightPos.weight = ParticleFilter::calculateWeightForEstimatedPosition(newPosition, lp, lineDistHelper, linePointsInvalidity, invCounter);
		coi.position = (robotPosition);
		coi.motion = (robotVelocity);
		coi.certainty = (weightPos.weight);
		coi.locType.type = (msl_sensor_msgs::LocalizationType::ErrorMin);


		//Corrected Odometry for Directed Camera
		CorrectedOdometry corrOdo;
		corrOdo.posX = weightPos.x;
		corrOdo.posY = weightPos.y;
		corrOdo.posAngle =	weightPos.heading;
		corrOdo.posCertainty = weightPos.weight;
		SharedMemoryHelper::getInstance()->writeCorrectedOdometry(&corrOdo);

		/*CovMatrix3DPtr covMatrix = CovMatrix3D::create();
		boost::shared_ptr<std::vector<double> > covariances = boost::shared_ptr<std::vector<double> >(new std::vector<double>);

		for(int i = 0; i < 9; i++){
			covariances->push_back(0.0);
		}

		(*covariances)[0] = 3.0*180.0/M_PI*3.0*180.0/M_PI;
		(*covariances)[4] = 150.0*150.0;
		(*covariances)[8] = 150.0*150.0;

		covMatrix->setValues(covariances);

		coi->setCovarianceMatrix(covMatrix);
*/

//double calculateWeightForEstimatedPosition(Position pos, std::vector<LinePoint> & linePoints, LineDistanceHelper & lineDistHelper, unsigned char * linePointsInvalidity, int invCounter);

		BallIntegrator::getInstance()->setRefPosition(newPosition);

		lastIteration = TimeHelper::getInstance()->getVisionTimeOmniCam();

		return weightPos;


	}

	void ErrorMinLocalization::writeCoi()
	{

		//printf("Gain NewLoc ErrorMin im WM\n");
		unsigned long long timestamp = TimeHelper::getInstance()->getVisionTimeOmniCam();
                coi.imageTime =(timestamp);
		SpicaHelper::wm->odometry = (coi);
		std::cout<<"ErrorMinLoc im WM"<<std::endl;
	}

	Cox::field_circles_t ErrorMinLocalization::getCircles()
	{
		//fieldcircles
		Cox::field_circles_t field_circles;
		//mid circle
		field_circles.push_back(Cox::circle_t(FootballField::MiddleCircleRadius, 0, 0));
		//cornercircles
		if (FootballField::CornerCircleExists)
		{
			field_circles.push_back(Cox::circle_t(FootballField::CornerCircleRadius, FootballField::FieldLength/2, FootballField::FieldWidth/2));
			field_circles.push_back(Cox::circle_t(FootballField::CornerCircleRadius, FootballField::FieldLength/2, -FootballField::FieldWidth/2));
			field_circles.push_back(Cox::circle_t(FootballField::CornerCircleRadius, -FootballField::FieldLength/2, -FootballField::FieldWidth/2));
			field_circles.push_back(Cox::circle_t(FootballField::CornerCircleRadius, -FootballField::FieldLength/2, FootballField::FieldWidth/2));
		}
		std::cout<<"NewLoc nr of circles "<<field_circles.size()<<std::endl;
		return field_circles;
	}

	Cox::field_lines_t ErrorMinLocalization::getLines()
	{

		Cox::field_lines_t lines;
		int start_x, start_y, end_x, end_y;



		//field lines
		start_x = FootballField::FieldLength/2;
		start_y = FootballField::FieldWidth/2;
		end_x = FootballField::FieldLength/2;
		end_y = -FootballField::FieldWidth/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = FootballField::FieldLength/2;
		start_y = -FootballField::FieldWidth/2;
		end_x = -FootballField::FieldLength/2;
		end_y = -FootballField::FieldWidth/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = -FootballField::FieldLength/2;
		start_y = -FootballField::FieldWidth/2;
		end_x = -FootballField::FieldLength/2;
		end_y = FootballField::FieldWidth/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = -FootballField::FieldLength/2;
		start_y = FootballField::FieldWidth/2;
		end_x = FootballField::FieldLength/2;
		end_y = FootballField::FieldWidth/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		//middle line
		start_x = 0;
		start_y = FootballField::FieldWidth/2;
		end_x = 0;
		end_y = -FootballField::FieldWidth/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		//goal left 6
		start_x =  -FootballField::FieldLength/2;
		start_y =  -FootballField::GoalInnerAreaLength/2;
		end_x 	=  -FootballField::FieldLength/2 + FootballField::GoalInnerAreaWidth;
		end_y 	=  -FootballField::GoalInnerAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));
		//7
		start_x = -FootballField::FieldLength/2 + FootballField::GoalInnerAreaWidth;
		start_y = -FootballField::GoalInnerAreaLength/2;
		end_x 	= -FootballField::FieldLength/2 + FootballField::GoalInnerAreaWidth;
		end_y 	=  FootballField::GoalInnerAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = -FootballField::FieldLength/2 + FootballField::GoalInnerAreaWidth;
		start_y =  FootballField::GoalInnerAreaLength/2;
		end_x = -FootballField::FieldLength/2;
		end_y = FootballField::GoalInnerAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		//goal right
		start_x = FootballField::FieldLength/2;
		start_y = -FootballField::GoalInnerAreaLength/2;
		end_x 	= FootballField::FieldLength/2 - FootballField::GoalInnerAreaWidth;
		end_y 	=  -FootballField::GoalInnerAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = FootballField::FieldLength/2 - FootballField::GoalInnerAreaWidth;
		start_y = -FootballField::GoalInnerAreaLength/2;
		end_x = FootballField::FieldLength/2 - FootballField::GoalInnerAreaWidth;
		end_y = FootballField::GoalInnerAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = FootballField::FieldLength/2 - FootballField::GoalInnerAreaWidth;
		start_y = FootballField::GoalInnerAreaLength/2;
		end_x = FootballField::FieldLength/2;
		end_y = FootballField::GoalInnerAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		//penaltyleft

		start_x =  -FootballField::FieldLength/2;
		start_y =  -FootballField::GoalAreaLength/2;
		end_x 	=  -FootballField::FieldLength/2 + FootballField::GoalAreaWidth;
		end_y 	=  -FootballField::GoalAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));
		//7
		start_x = -FootballField::FieldLength/2 + FootballField::GoalAreaWidth;
		start_y = -FootballField::GoalAreaLength/2;
		end_x 	= -FootballField::FieldLength/2 + FootballField::GoalAreaWidth;
		end_y 	=  FootballField::GoalAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = -FootballField::FieldLength/2 + FootballField::GoalAreaWidth;
		start_y =  FootballField::GoalAreaLength/2;
		end_x = -FootballField::FieldLength/2;
		end_y = FootballField::GoalAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));


		//penalty right
		start_x = FootballField::FieldLength/2;
		start_y = -FootballField::GoalAreaLength/2;
		end_x 	= FootballField::FieldLength/2 - FootballField::GoalAreaWidth;
		end_y 	=  -FootballField::GoalAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = FootballField::FieldLength/2 - FootballField::GoalAreaWidth;
		start_y = -FootballField::GoalAreaLength/2;
		end_x = FootballField::FieldLength/2 - FootballField::GoalAreaWidth;
		end_y = FootballField::GoalAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		start_x = FootballField::FieldLength/2 - FootballField::GoalAreaWidth;
		start_y = FootballField::GoalAreaLength/2;
		end_x = FootballField::FieldLength/2;
		end_y = FootballField::GoalAreaLength/2;
		std::cout<<"NewLoc ("<<start_x<<","<<start_y<<") "<<"("<<end_x<<","<<end_y<<") "<<std::endl;
		lines.push_back(Cox::Line2D(start_x,start_y,end_x,end_y));

		std::cout << "NewLoc map lines " << lines.size() << std::endl;
		return lines;

	}
}
