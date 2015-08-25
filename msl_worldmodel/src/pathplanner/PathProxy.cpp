/*
 * PathProxy.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/PathProxy.h"
#include "pathplanner/VoronoiNet.h"
#include "container/CNPosition.h"
#include "msl_msgs/PathPlanner.h"
#include "msl_msgs/VoronoiNetInfo.h"

namespace msl
{

	/**
	 * struct for generating a cropped voronoi from a delaunay
	 */
	struct Cropped_voronoi_from_delaunay
	{
		std::list<Segment_2> m_cropped_vd;
		Iso_rectangle_2 m_bbox;

		Cropped_voronoi_from_delaunay(const Iso_rectangle_2& bbox) :
				m_bbox(bbox)
		{
		}

		template<class RSL>
		void crop_and_extract_segment(const RSL& rsl)
		{
			CGAL::Object obj = CGAL::intersection(rsl, m_bbox);
			const Segment_2* s = CGAL::object_cast<Segment_2>(&obj);
			if (s)
				m_cropped_vd.push_back(*s);
		}

		void operator<<(const Ray_2& ray)
		{
			crop_and_extract_segment(ray);
		}
		void operator<<(const Line_2& line)
		{
			crop_and_extract_segment(line);
		}
		void operator<<(const Segment_2& seg)
		{
			crop_and_extract_segment(seg);
		}
	};

	PathProxy::PathProxy()
	{
		this->wm = MSLWorldModel::get();
		pathPub = n.advertise<msl_msgs::PathPlanner>("/PathPlanner/PathPlanner", 10);
		voroniPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);
		sc = supplementary::SystemConfig::getInstance();
		this->pathPlannerDebug = (*this->sc)["PathPlanner"]->get<bool>("PathPlanner", "pathPlannerDebug",
		NULL);

	}

	PathProxy::~PathProxy()
	{
	}

	/**
	 * get ego direction form path planner
	 * @param egoTarget shared_ptr<geometry::CNPoint2D>
	 * @param eval shared_ptr<PathEvaluator>
	 * @param additionalPoints shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
	 * @return shared_ptr<geometry::CNPoint2D>
	 */
	shared_ptr<geometry::CNPoint2D> PathProxy::getEgoDirection(shared_ptr<geometry::CNPoint2D> egoTarget,
																shared_ptr<PathEvaluator> eval,
																shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints)
	{
		//save target
		lastPathTarget = egoTarget;
		shared_ptr<VoronoiNet> net = this->wm->pathPlanner.getCurrentVoronoiNet();
		//TODO remove
//		auto tmp = net->blockCircle(make_shared<geometry::CNPoint2D>(0,0), 2000);
		//if there are additional points insert them into the voronoi diagram
		if(additionalPoints != nullptr)
		{
			net->insertAdditionalPoints(additionalPoints);
		}
		shared_ptr<geometry::CNPoint2D> retPoint = nullptr;
		//get own position
		shared_ptr<geometry::CNPosition> ownPos = this->wm->rawSensorData.getOwnPositionVision();
		if (ownPos != nullptr)
		{
			//plan
			shared_ptr<geometry::CNPoint2D> alloTarget = egoTarget->egoToAllo(*ownPos);
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path = this->wm->pathPlanner.plan(net,
			make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y), alloTarget, eval);
			if (path != nullptr)
			{
				//get first point of returned path
				retPoint = make_shared<geometry::CNPoint2D>(path->at(0)->x, path->at(0)->y);

				//send debug msgs
				if(pathPlannerDebug)
				{
					path->insert(path->begin(), make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y));
					sendPathPlannerMsg(path);
					sendVoronoiNetMsg(net->getSitePositions(),net);
				}
			}
		}
		//remove additional points form voronoi
		if(additionalPoints != nullptr)
		{
			net->removeSites(additionalPoints);
		}
//		net->removeSites(tmp);
		if(retPoint == nullptr)
		{
			return nullptr;
		}
//		cout << "pathproxy return " << retPoint->alloToEgo(*ownPos)->toString() << endl;
		return retPoint->alloToEgo(*ownPos);

	}

	/**
	 * get the path proxy instacne
	 */
	PathProxy* PathProxy::getInstance()
	{
		static PathProxy instance;
		return &instance;
	}

	/**
	 * send debug msg with pathplanner path
	 * @param path shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>>
	 */
	void PathProxy::sendPathPlannerMsg(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path)
	{
		msl_msgs::PathPlanner pathMsg;
		for(int i = 0; i < path->size(); i++)
		{
			msl_msgs::Point2dInfo info;
			info.x = path->at(i)->x;
			info.y = path->at(i)->y;
			pathMsg.pathPoints.push_back(info);
		}
		pathPub.publish(pathMsg);
	}

	/**
	 * send debug msg with voroni infos
	 * @param sites shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 * @param voronoi shared_ptr<VoronoiNet>
	 */
	void PathProxy::sendVoronoiNetMsg(shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > sites,
										shared_ptr<VoronoiNet> voronoi)
	{
		msl_msgs::VoronoiNetInfo netMsg;
		for (int i = 0; i < sites->size(); i++)
		{
			msl_msgs::Point2dInfo info;
			info.x = sites->at(i).first->x;
			info.y = sites->at(i).first->y;
			netMsg.sites.push_back(info);
		}
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > linePoints = calculateCroppedVoronoi(sites, voronoi);
		for (int i = 0; i < linePoints->size(); i++)
		{
			msl_msgs::Point2dInfo info;
			info.x = linePoints->at(i)->x;
			info.y = linePoints->at(i)->y;
			netMsg.linePoints.push_back(info);
		}
		voroniPub.publish(netMsg);
	}

	/**
	 * calculates cropped voronoi for debug msg
	 * @param sites shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>>
	 * @param voronoi shared_ptr<VoronoiNet>
	 */
	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > PathProxy::calculateCroppedVoronoi(
			shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int> > > sites, shared_ptr<VoronoiNet> voronoi)
	{
		//create bounding box around field with additional 3m distance
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		Iso_rectangle_2 bbox(- msl::MSLFootballField::FieldLength / 2 - 3000, -msl::MSLFootballField::FieldWidth / 2 - 3000,
							 msl::MSLFootballField::FieldLength / 2 + 3000, msl::MSLFootballField::FieldWidth / 2 + 3000);
		//create cropped voronoi
		Cropped_voronoi_from_delaunay vor(bbox);
		//get delaunay
		DelaunayTriangulation dt = voronoi->getVoronoi()->dual();
		dt.draw_dual(vor);
		//get sites
		for (auto it = vor.m_cropped_vd.begin(); it != vor.m_cropped_vd.end(); it++)
		{
			//check if source and target exist
			//if there is no source or target the edge is connected to point at infinity
			if (!std::isnan(it->source().x()) && !std::isnan(it->source().y()) && !std::isnan(it->target().x())
					&& !std::isnan(it->target().y()))
			{
				//push back source and target
				ret->push_back(make_shared<geometry::CNPoint2D>(it->source().x(), it->source().y()));
				ret->push_back(make_shared<geometry::CNPoint2D>(it->target().x(), it->target().y()));
			}
		}
		return ret;
	}

} /* namespace msl */
