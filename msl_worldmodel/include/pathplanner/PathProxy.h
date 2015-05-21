/*
 * PathProxy.h
 *
 *  Created on: May 17, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_PATHPROXY_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_PATHPROXY_H_

#include <memory>
#include "container/CNPoint2D.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "MSLWorldModel.h"

namespace msl
{

	class PathProxy
	{
	public:
		PathProxy(MSLWorldModel* wm);
		virtual ~PathProxy();
		shared_ptr<CNPoint2D> getEgoDirection(CNPoint2D egoTarget, bool stayInField, PathEvaluator* eval);

	private:
		CNPoint2D lastPathTarget;
		MSLWorldModel* wm;

	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_PATHPROXY_H_ */
