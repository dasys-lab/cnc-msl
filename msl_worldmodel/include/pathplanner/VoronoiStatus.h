/*
 * VoronoiStatus.h
 *
 *  Created on: Apr 26, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_VORONOISTATUS_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_VORONOISTATUS_H_

namespace msl
{

	/**
		 * Reflects the status of a VoronoiNet
		 */
			enum VoronoiStatus{New, Latest, Old, Calculating};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_VORONOISTATUS_H_ */
