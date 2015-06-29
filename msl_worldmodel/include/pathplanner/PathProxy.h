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
		PathProxy();
		virtual ~PathProxy();
		shared_ptr<geometry::CNPoint2D> getEgoDirection(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<PathEvaluator> eval, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
		static PathProxy* getInstance();

	private:
		shared_ptr<geometry::CNPoint2D> lastPathTarget;
		MSLWorldModel* wm;

	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_PATHPROXY_H_ */
