/*
 * Opponents.h
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_

#include <SystemConfig.h>
#include <vector>
#include <container/CNPoint2D.h>

using namespace std;

namespace msl
{

	class MSLWorldModel;
	class Opponents
	{
	public:
		Opponents(MSLWorldModel* wm, int ringBufferLength);
		virtual ~Opponents();
		double getOpponentProtectDistance();
		double getOpponentProtectAngle();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getOpponentsAlloClustered();
		void setOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsAlloClustered);
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getOpponentsEgoClustered();
		void setOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsEgoClustered);

	private:
		MSLWorldModel* wm;
		supplementary::SystemConfig* sc;
		int ringBufferLength;
		double opponentProtectDistance;
		double opponentProtectAngle;
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsEgoClustered;
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsAlloClustered;
	};


} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_ */
