#ifndef ErrorMinLocalization_H
#define ErrorMinLocalization_H
#include "../global/CoxTypes.h"
#include "../global/Types.h"
#include "LinePoint.h"
#include "CoxLocalization.h"
#include "RawOdometryHelper.h"
#include "FootballField.h"
#include "Line2D.h"
#include "TimeHelper.h"
#include "LineDistanceHelper.h"
#include "ParticleFilter.h"
#include <iostream>
#include <CNSensorMsgs/CorrectedOdometryInfo.h>
//#include <CorrectedOdometryCovInfo.h> // merge to dev
#include "SharedMemoryHelper.h"


namespace redwolf {
	class ErrorMinLocalization
	{
		public:
			ErrorMinLocalization();
			~ErrorMinLocalization();
			WeightedPosition estimatedPosition(Position curPos, std::vector<LinePoint> lp, LineDistanceHelper & lineDistHelper);
			//WeightedPosition estimatedPosition(Position curPos, std::vector<LinePoint> lp, LineDistanceHelper & lineDistHelper);
			Cox::field_pose_t pos2pose(Position pos);
			Position pose2pos(Cox::field_pose_t pose);
			void writeCoi();
		
		protected:
			unsigned long long lastIteration;
			RawOdometryHelper *rawOdometryHelper;
			Cox::CoxLocalization *coxLoc ;
			//Cox::line_points_t detectedLinePoints;
			Cox::field_lines_t fieldLines;
			Cox::field_circles_t fieldCircles;
			Cox::field_lines_t getLines();
			Cox::field_circles_t getCircles();
			//field_circles_t getCircles();
			void linePoint2linePointst(std::vector<LinePoint>, Cox::line_points_t &);
			Position positionBuffer[RAWODOBUFSIZE];
			unsigned long long timestampBuffer[RAWODOBUFSIZE];
			int integrationIndex;
			bool bufferInitialized;
			MovingRobot mr;
			MovingRobot mrOld;
			//CorrectedOdometryCovInfoPtr coi;
			CNSensorMsgs::CorrectedOdometryInfo coi;

			

	};
}
#endif
