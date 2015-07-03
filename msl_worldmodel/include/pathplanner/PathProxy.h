/*
 * PathProxy.h
 *
 *  Created on: May 17, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_PATHPROXY_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_PATHPROXY_H_

#define PATHPLANNER_DEBUG

//includes for CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DelaunayTriangulation> DelaunayAdaptionTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DelaunayTriangulation> DelaunayAdaptionPolicy;
typedef CGAL::Voronoi_diagram_2<DelaunayTriangulation, DelaunayAdaptionTraits, DelaunayAdaptionPolicy> VoronoiDiagram;
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;
typedef Kernel::Iso_rectangle_2 Iso_rectangle_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Ray_2 Ray_2;
typedef Kernel::Line_2 Line_2;

#include <memory>
#include "container/CNPoint2D.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "MSLWorldModel.h"
#include <ros/ros.h>
#include "VoronoiNet.h"


namespace msl
{

	class PathProxy
	{
	public:
		PathProxy();
		virtual ~PathProxy();
		shared_ptr<geometry::CNPoint2D> getEgoDirection(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<PathEvaluator> eval, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = nullptr);
		static PathProxy* getInstance();
		void sendPathPlannerMsg(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path);
		void sendVoronoiNetMsg(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> sites, shared_ptr<VoronoiNet> voronoi);
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > calculateCroppedVoronoi(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> sites, shared_ptr<VoronoiNet> voronoi);

	private:
		shared_ptr<geometry::CNPoint2D> lastPathTarget;
		MSLWorldModel* wm;
		ros::NodeHandle n;
		ros::Publisher pathPub;
		ros::Publisher voroniPub;

	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_PATHPLANNER_PATHPROXY_H_ */
