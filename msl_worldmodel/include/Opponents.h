/*
 * Opponents.h
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_

#include <SystemConfig.h>

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

	private:
		MSLWorldModel* wm;
		supplementary::SystemConfig* sc;
		int ringBufferLength;
		double opponentProtectDistance;
		double opponentProtectAngle;
	};


} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_ */
