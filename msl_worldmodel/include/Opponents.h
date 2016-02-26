/*
 * Opponents.h
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_

namespace msl
{

	class MSLWorldModel;
	class Opponents
	{
	public:
		Opponents(MSLWorldModel* wm, int ringBufferLength);
		virtual ~Opponents();
	private:
		MSLWorldModel* wm;
		int ringBufferLength;
	};


} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_ */
