/*
 * PathPlanner.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_

//includes for CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>


/**
 * typedefs to short CGAL's class names e.g.CGAL::Voronoi_diagram_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>,
CGAL::Delaunay_triangulation_adaptation_traits_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>,
CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<CGAL::Delaunay_triangulation_2<CGAL::Exact_predicates_inexact_constructions_kernel>>>
to VoronoiDiagram
 */
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DelaunayTriangulation> DelaunayAdaptionTraits;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DelaunayTriangulation> DelaunayAdaptionPolicy;
typedef CGAL::Voronoi_diagram_2<DelaunayTriangulation, DelaunayAdaptionTraits, DelaunayAdaptionPolicy> VoronoiDiagram;
typedef DelaunayAdaptionTraits::Point_2 Point_2;
typedef DelaunayAdaptionTraits::Site_2 Site_2;

//other includes
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <math.h>
#include <mutex>

#include "SystemConfig.h"
#include "pathplanner/SearchNode.h"
#include "pathplanner/VoronoiNet.h"
#include "pathplanner/VoronoiStatus.h"
#include <msl_sensor_msgs/WorldModelData.h>


//namespaces
using namespace std;

namespace msl
{

	class MSLWorldModel;
	class PathPlanner
	{
	public:
		PathPlanner(MSLWorldModel* wm, int count);
		virtual ~PathPlanner();
		/**
		 * aStar search on a VoronoiDiagram
		 * @param voronoi shared_ptr<VoronoiNet>
		 * @param ownPos Point_2
		 * @param goal Point_2
		 * @return shared_ptr<vector<shared_ptr<Point_2>>>
		 */
		shared_ptr<vector<shared_ptr<Point_2>>> aStarSearch(shared_ptr<VoronoiNet> voronoi, Point_2 ownPos, Point_2 goal);
		/**
		 * processes the WorldModel msg
		 * @param msg msl_sensor_msgs::WorldModelDataPtr
		 */
		void processWolrdModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		/**
		 * gets all saved VoronoiNets
		 * @return vector<shared_ptr<VoronoiNet>>
		 */
		vector<shared_ptr<VoronoiNet>> getVoronoiNets();
		/**
		 * gets latest accesable VoronoiNet
		 * @return shared_ptr<VoronoiNet>
		 */
		shared_ptr<VoronoiNet> getCurrentVoronoiNet();

	protected:
		MSLWorldModel* wm;
		supplementary::SystemConfig* sc;
		mutex voronoiMutex;
		vector<shared_ptr<VoronoiNet>> voronoiDiagrams;
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_H_ */
