/*
 * Obstacles.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: Stefan Jakob
 */

#include <obstaclehandler/Obstacles.h>
#include "MSLFootballField.h"
#include "obstaclehandler/SimpleCluster.h"
#include "MSLWorldModel.h"
#include "obstaclehandler/AnnotatedObstacleClusterPool.h"

namespace msl
{

	Obstacles::Obstacles(MSLWorldModel* wm, int ringbufferLength) : obstacles(ringbufferLength)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		DENSITY = (*sc)["PathPlanner"]->get<double>("PathPlanner.ObHandler.density", NULL);
		VARIANCE_THRESHOLD = DENSITY * DENSITY;
		TERRITORY_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.territoryRadius", NULL);
		SIGHT_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.sightradius", NULL); // how far an obstacle can be seen proper
		FIELD_TOL = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler.fieldTol", NULL);
		POS_CERTAINTY_TH_CLUSTERING = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyTHClustering", NULL);
		POS_CERTAINTY_HYS = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyHys", NULL);
		DFLT_OB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "obstacleRadius", NULL);
		DFLT_ROB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "teammateRadius", NULL);
		OBSTACLE_MAP_OUT_TOLERANCE = (*sc)["PathPlanner"]->get<double>("PathPlanner", "ObHandler",
																		"obstacleMapOutTolerance", NULL);
		LOCALIZATION_SUCCESS_CONFIDENCE = (*sc)["Localization"]->get<double>("Localization", "LocalizationSuccess",
		NULL);
		field = MSLFootballField::getInstance();
		shared_ptr<vector<shared_ptr<AnnotatedObstacleCluster>>> clusterArray = make_shared<vector<shared_ptr<AnnotatedObstacleCluster>>>();
		shared_ptr<vector<shared_ptr<AnnotatedObstacleCluster>>> newClusterArray = make_shared<vector<shared_ptr<AnnotatedObstacleCluster>>>();
		pool = new AnnotatedObstacleClusterPool();
	}

	Obstacles::~Obstacles()
	{
	}

	void Obstacles::handleObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > myObstacles)
	{
		//TODO save lists in WM
		// SETUP
		setupAnnotatedObstacles(myObstacles, wm->rawSensorData.getCorrectedOdometryInfo());
		// CLUSTERING
		clusterAnnotatedObstacles();

		// CREATE DATASTRUCTURES FOR WM, DELAUNAY-GENERATOR, ETC.
		shared_ptr<vector<AnnotatedObstacleCluster*>> newObsClustersAllo = make_shared<vector<AnnotatedObstacleCluster*>>();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newObsEgo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newOppEgo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newOppAllo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newTeammatesEgo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> newTeammatesAllo = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();

		shared_ptr<geometry::CNPoint2D> curAlloPoint = nullptr;
		shared_ptr<geometry::CNPoint2D> curEgoPoint = nullptr;
		for (int i = 0; i < newClusterArray->size(); ++i)
		{
			newObsClustersAllo->push_back(newClusterArray->at(i));

			curAlloPoint = make_shared<geometry::CNPoint2D>(newClusterArray->at(i)->x, newClusterArray->at(i)->y);
			curEgoPoint = curAlloPoint->alloToEgo(
					*(make_shared<geometry::CNPosition>(wm->rawSensorData.getCorrectedOdometryInfo()->position.x,
														wm->rawSensorData.getCorrectedOdometryInfo()->position.y,
														wm->rawSensorData.getCorrectedOdometryInfo()->position.angle)));

			if (newClusterArray->at(i)->ident == EntityType::Opponent)
			{
				// it is not a teammate
				if (field->isInsideField(curAlloPoint, FIELD_TOL))
				{
					newOppAllo->push_back(curAlloPoint);
					// egocentric obstacles, which are inside the field and do not belong to our team
					newOppEgo->push_back(curEgoPoint);
				}
			}
			else if (newClusterArray->at(i)->ident != wm->getOwnId())
			{
				newTeammatesEgo->push_back(curEgoPoint);
				newTeammatesAllo->push_back(curAlloPoint);
			}
		}

		// change the vNet references to the new lists
		this->obstaclesClustersAllo = newObsClustersAllo;
		this->obstaclesEgoClustered = newObsEgo;
		wm->robots.opponents.setOpponentsEgoClustered(newOppEgo);
		wm->robots.opponents.setOpponentsAlloClustered(newOppAllo);
		wm->robots.teammates.setTeammatesEgoClustered(newTeammatesEgo);
		wm->robots.teammates.setTeammatesAlloClustered(newTeammatesAllo);
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > Obstacles::clusterPoint2D(
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > obstacles, double varianceThreshold)
	{
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > retList = make_shared<
				vector<shared_ptr<geometry::CNPoint2D> > >();
		shared_ptr<vector<shared_ptr<SimpleCluster>>> clusterList = make_shared<vector<shared_ptr<SimpleCluster> > >();
		bool mergedCluster = true;

		// init cluster objects
		for (int i = 0; i < obstacles->size(); ++i)
		{
			clusterList->push_back(make_shared<SimpleCluster>(obstacles->at(i)));
		}

		while (mergedCluster)
		{
			// find the two nearest clusters
			int fstClusterId = EntityType::Opponent;
			int sndClusterId = EntityType::Opponent;
			double minDist = numeric_limits<double>::max();
			double curDist = 0;
			for (int i = 0; i < clusterList->size(); ++i)
			{
				for (int j = 0; j < i; j++)
				{
					// check dist
					curDist = clusterList->at(i)->distanceTo(clusterList->at(j));
					if (curDist < minDist)
					{
						fstClusterId = i;
						sndClusterId = j;
						minDist = curDist;
					}
				}
			}

			// check if variance after merging is below VARIANCE_THRESHOLD
			if (fstClusterId != EntityType::Opponent)
			{
				mergedCluster = clusterList->at(fstClusterId)->checkAndMerge(clusterList->at(sndClusterId),
																				varianceThreshold);
				if (mergedCluster)
				{
					clusterList->erase(clusterList->begin() + sndClusterId);
				}
			}
			else
			{
				mergedCluster = false;
			}
		}

		for (int i = 0; i < clusterList->size(); ++i)
		{
			retList->push_back(make_shared<geometry::CNPoint2D>(clusterList->at(i)->x, clusterList->at(i)->y));
			//				Console.WriteLine("2 Point2D: " + retList[i].X + " " + retList[i].Y);
		}

		return retList;
	}

	void Obstacles::clusterAnnotatedObstacles()
	{
		bool mergedCluster = true;

		while (mergedCluster)
		{
			// find the two nearest mergeable clusters
			int fstClusterId = EntityType::Opponent;
			int sndClusterId = EntityType::Opponent;
			double minDist = numeric_limits<double>::max();
			double curDist = 0;
			for (int i = 0; i < clusterArray->size(); ++i)
			{
				for (int j = 0; j < i; ++j)
				{
					if ((clusterArray->at(i)->ident == EntityType::Opponent || clusterArray->at(j)->ident == EntityType::Opponent)
							&& std::find(clusterArray->at(i)->supporter->begin(), clusterArray->at(i)->supporter->end(),
											clusterArray->at(j)->ident) == clusterArray->at(i)->supporter->end()
							&& std::find(clusterArray->at(j)->supporter->begin(), clusterArray->at(j)->supporter->end(),
											clusterArray->at(i)->ident) == clusterArray->at(j)->supporter->end())
					{
						// mergeable, check dist
						curDist = clusterArray->at(i)->distanceTo(clusterArray->at(j));
						if (curDist < minDist)
						{
							fstClusterId = i;
							sndClusterId = j;
							minDist = curDist;
						}
					}
				}
			}

			// check if variance after merging is below VARIANCE_THRESHOLD
			if (fstClusterId != EntityType::Opponent)
			{
				mergedCluster = clusterArray->at(fstClusterId)->checkAndMerge(clusterArray->at(sndClusterId),
																				VARIANCE_THRESHOLD);
				if (mergedCluster)
				{
					clusterArray->erase(clusterArray->begin() + sndClusterId);
				}
			}
			else
			{
				mergedCluster = false;
			}
		}

		newClusterArray = make_shared<vector<AnnotatedObstacleCluster*>>();
		for (int i = 0; i < clusterArray->size(); i++)
		{
			newClusterArray->push_back(clusterArray->at(i));
		}
		std::sort(newClusterArray->begin(), newClusterArray->end(), AnnotatedObstacleCluster::compareTo);
		clusterArray->clear();
	}

	void Obstacles::setupAnnotatedObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ownObs,
											shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> myOdo)
	{
		clusterArray->clear();
		AnnotatedObstacleCluster* obs = nullptr;
		int velX = 0;
		int velY = 0;
		for (pair<int, shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>> > curRobot : wm->robots.sharedWolrdModelData)
		{
			/* Ignore every robot, which:
			 * - is unlocalised
			 * - messages are old (25 ms)
			 * - I am (the obstacles from the WM are newer)*/
			shared_ptr<msl_sensor_msgs::SharedWorldInfo> currentRobot = curRobot.second->getLast()->getInformation();
			shared_ptr<geometry::CNPosition> curPlayerPosition = make_shared<geometry::CNPosition>(currentRobot->odom.position.x, currentRobot->odom.position.y, currentRobot->odom.position.angle);
			if(currentRobot == nullptr || currentRobot->odom.certainty < 0.8
			|| currentRobot->senderID == wm->getOwnId()
			|| curRobot.second->getLast()->timeStamp + 250000000 < wm->getTime()
			)
			{
				continue;
			}

			// add all obstacles seen by curRobot
			vector<msl_msgs::Point2dInfo> curOppList = currentRobot->obstacles;
			if (curOppList.size() > 0)// != nullptr
			{
				for(int i = 0; i < curOppList.size(); ++i)
				{
					// Nobody knows the positions of obstacles arround me better than I do!
					if(distance(curOppList.at(i), myOdo->position) < TERRITORY_RADIUS)
					{
						continue;
					}

					if (MSLFootballField::getInstance()->isInsideField(make_shared<geometry::CNPoint2D>(curOppList.at(i).x, curOppList.at(i).y), OBSTACLE_MAP_OUT_TOLERANCE))
					{
						obs = AnnotatedObstacleCluster::getNew(this->pool);
						obs->init((int) (curOppList.at(i).x + 0.5), (int) (curOppList.at(i).y + 0.5), // pos
						DFLT_OB_RADIUS,
						EntityType::Opponent, curRobot.first);
						clusterArray->push_back(obs);
					}
				}
			}

			// TODO: something to identify the other robot, when he is close to me

			/* add the curRobot itself as an obstacle, to identify teammates */

			// Convert ego motion angle to allo motion angle
			double alloMotionAngle = currentRobot->odom.position.angle + currentRobot->odom.motion.angle;
			if (alloMotionAngle > M_PI)
			{
				alloMotionAngle -= 2*M_PI;
			}
			else if (alloMotionAngle < -M_PI)
			{
				alloMotionAngle += 2 * M_PI;
			}

			// Calc x and y the velocity
			velX = (int) (cos(alloMotionAngle) * currentRobot->odom.motion.translation + 0.5);
			velY = (int) (sin(alloMotionAngle) * currentRobot->odom.motion.translation + 0.5);

			// predict the position along the translation
			double seconds = (double) (wm->getTime() - curRobot.second->getLast()->timeStamp) / 10000000.0;

			obs = AnnotatedObstacleCluster::getNew(this->pool);
			obs->init((int) (curPlayerPosition->x + seconds * velX + 0.5), (int) (curPlayerPosition->y + seconds * velY + 0.5),// pos
			DFLT_ROB_RADIUS,
			velX, velY,// velocity
			curRobot.first,
			curRobot.first);
			clusterArray->push_back(obs);
		}

		/* add my own obstacles from the worldmodel (they are egocentric :-( ) */
		shared_ptr<geometry::CNPoint2D> curPoint = make_shared<geometry::CNPoint2D>();
		for (int i = 0; i < ownObs->size(); ++i)
		{
			shared_ptr<geometry::CNPosition> me = make_shared<geometry::CNPosition>(myOdo->position.x,
																					myOdo->position.y,
																					myOdo->position.angle);
			curPoint = ownObs->at(i)->egoToAllo(*me);
			if (MSLFootballField::getInstance()->isInsideField(curPoint, OBSTACLE_MAP_OUT_TOLERANCE))
			{
				obs = AnnotatedObstacleCluster::getNew(this->pool);
				obs->init((int)(curPoint->x + 0.5), (int)(curPoint->y + 0.5), DFLT_OB_RADIUS, EntityType::Opponent, wm->getOwnId());
				clusterArray->push_back(obs);
			}
		}

		/* add my own position: */

		// Convert ego motion angle to allo motion angle
		double alloMotAngle = myOdo->position.angle + myOdo->motion.angle;
		if (alloMotAngle > M_PI)
		{
			alloMotAngle -= 2 * M_PI;
		}
		else if (alloMotAngle < -M_PI)
		{
			alloMotAngle += 2 * M_PI;
		}

		velX = (int)round(myOdo->motion.translation * cos(alloMotAngle));
		velY = (int)round(myOdo->motion.translation * sin(alloMotAngle));
		obs = AnnotatedObstacleCluster::getNew(this->pool);
		obs->init((int)(myOdo->position.x + 0.5), (int)(myOdo->position.y + 0.5), myOdo->position.angle,
					DFLT_ROB_RADIUS, velX, velY, myOdo->motion.rotation, myOdo->position.certainty, wm->getOwnId(),
					wm->getOwnId());
		clusterArray->push_back(obs);
	}

	void Obstacles::processNegSupporter(shared_ptr<geometry::CNPosition> myPosition)
	{
		double curAngle = 0.0;
		double curAngle2 = 0.0;
		double dangle = 0.0;
		double dangle2 = 0.0;
		double left = 0.0;
		double right = 0.0;
		double left2 = 0.0;
		double right2 = 0.0;
		double curDist = 0.0;
		double curDist2 = 0.0;
		shared_ptr<geometry::CNPoint2D> curPoint = make_shared<geometry::CNPoint2D>();
		shared_ptr<geometry::CNPoint2D> curPoint2 = make_shared<geometry::CNPoint2D>();
		bool sightIsBlocked;

		for (pair<int, shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>> > curRobot : wm->robots.sharedWolrdModelData)
		{
			//cout << "Robot: " << curRobot.first << endl;
			/* Ignore every robot, which is:
			 * - unlocalised
			 * - myself
			 */
			shared_ptr<msl_sensor_msgs::SharedWorldInfo> currentRobot = curRobot.second->getLast()->getInformation();
			if (currentRobot == nullptr || currentRobot->odom.certainty < 0.8
			|| currentRobot->senderID == wm->getOwnId())
			{
				//				 	cout << "Skip" << endl;
				continue;
			}

			for (int i = 0; i < newClusterArray->size(); ++i)
			{
				//cout << "Cluster: " << (newClusterArray->at(i)->x /1000.0) <<  " " << (newClusterArray->at(i)->y / 1000.0) << endl;
				// continue, if the curRobot is a supporter of the curCluster or
				// the curCluster is out of the sight of the curRobot or
				// the curCluster is near me (<TERRITORY_RADIUS) so nobody was allowed to merg
				if (newClusterArray->at(i)->distanceTo(myPosition) < TERRITORY_RADIUS)
				{
					continue;
				}
				curDist = newClusterArray->at(i)->distanceTo(make_shared<geometry::CNPosition>(currentRobot->odom.position.x, currentRobot->odom.position.y, currentRobot->odom.position.angle));
				if (curDist > SIGHT_RADIUS)
				{
					//cout << "Too far away!" << endl;
					continue;
				}

				if (find(newClusterArray->at(i)->supporter->begin(), newClusterArray->at(i)->supporter->end(),currentRobot->senderID) != newClusterArray->at(i)->supporter->end())
				{
					//cout << "I am supporter!" << endl;
					continue;
				}

				curPoint = make_shared<geometry::CNPoint2D>(newClusterArray->at(i)->x - currentRobot->odom.position.x,
				newClusterArray->at(i)->y - currentRobot->odom.position.y);
				curAngle = atan2(curPoint->y, curPoint->x);
				dangle = abs(asin(DENSITY / curDist));

				// normalize angles
				left = curAngle + dangle;
				if (left < -M_PI)
				{
					left += 2.0 * M_PI;
				}
				else if (left > M_PI)
				{
					left -= 2.0 * M_PI;
				}
				right = curAngle - dangle;
				if (right < -M_PI)
				{
					right += 2.0 * M_PI;
				}
				else if (left > M_PI)
				{
					right -= 2.0 * M_PI;
				}
//
//									cout << "Cluster Angels: \n\tleft: " << (left * 180) / M_PI
//									                  << "\n\tmiddle: " << (curAngle * 180) / M_PI
//									                  << "\n\tright: " << (right * 180) / M_PI
//									                  << "\n\tdangle: " << (dangle * 180) / M_PI << endl;

				sightIsBlocked = false;

				// Für jedes Obstacle überprüfen, ob es im Weg steht
				if (currentRobot->obstacles.size() > 0)// != nullptr)
				{
					for (int j = 0; j < currentRobot->obstacles.size(); ++j)
					{
						cout << "Own Obstacle: " << (currentRobot->obstacles.at(j).x / 1000.0) << " " << (currentRobot->obstacles.at(j).y / 1000.0) << endl;

						curPoint2 = make_shared<geometry::CNPoint2D>(currentRobot->obstacles.at(j).x - currentRobot->odom.position.x,
						currentRobot->obstacles.at(j).y - currentRobot->odom.position.y);

						curDist2 = curPoint2->length();
						if (curDist2 < curDist)
						{ // the curPoint2 is closer then curPoint
							curAngle2 = atan2(curPoint2->y, curPoint2->x);
							dangle2 = abs(asin(DENSITY / curDist2));

							// normalize angles
							left2 = curAngle2 + dangle2;
							if (left2 < -M_PI)
							{
								left2 += 2.0 * M_PI;
							}
							else if (left2 > M_PI)
							{
								left2 -= 2.0 * M_PI;
							}
							right2 = curAngle2 - dangle2;
							if (right2 < -M_PI)
							{
								right2 += 2.0 * M_PI;
							}
							else if (right2 > M_PI)
							{
								right2 -= 2.0 * M_PI;
							}
//
//														cout << "Own Obstacle Angels: \n\tleft: " << (left2 * 180) / M_PI
//														                  << "\n\tmiddle: " << (curAngle2 * 180) / M_PI
//														                  << "\n\tright: " << (right2 * 180) / M_PI
//														                  << "\n\tdangle: " << (dangle2 * 180) / M_PI << endl;

							if (leftOf(left, right2) && !leftOf(left, left2))
							{
								//								cout << "Left of Cluster is behind Obstacle" << endl;
								sightIsBlocked = true;
								break;
							}

							if (leftOf(right, right2) && !leftOf(right, left2))
							{
								//								cout << "Right of Cluster is behind Obstacle" << endl;
								sightIsBlocked = true;
								break;
							}

							//							cout << "Obstacle does not block the sight!" << endl;
						}
						//						else
						//						{
						//							cout << "Own obstacle is too far away!" << endl;
						//						}
					}
				}
				else
				{
					sightIsBlocked = true;
				}

				// Wenn die Sicht nicht blockiert ist bin ich gegen das Obstacle
				if (!sightIsBlocked)
				{
					newClusterArray->at(i)->opposer->push_back(currentRobot->senderID);
				}
			}
		}

		// Lösche einfach alle Obstacle, die mehr Opposer als Supporter haben
		for (int i = 0; i < newClusterArray->size(); ++i)
		{
			if (newClusterArray->at(i)->supporter->size() < newClusterArray->at(i)->opposer->size())
			{
				cout << "OH: removed this obstacle X: " << newClusterArray->at(i)->x << " Y:"
						<< newClusterArray->at(i)->y << endl;
				newClusterArray->erase(newClusterArray->begin() + i);
			}
		}
	}

	bool Obstacles::leftOf(double angle1, double angle2)
	{
		if ((angle1 > 0.0 && angle2 > 0.0) || (angle1 < 0.0 && angle2 < 0.0))
		{
			if (angle1 > angle2)
			{
				return true;
			}
		}
		else
		{
			if (angle1 > 0)
			{
				if (angle1 - angle2 < M_PI)
				{
					return true;
				}
			}
			else
			{
				if (angle2 - angle1 > M_PI)
				{
					return true;
				}
			}
		}
		return false;
	}

	shared_ptr<vector<AnnotatedObstacleCluster*>> Obstacles::getObstaclesClustersAllo()
	{
		return obstaclesClustersAllo;
	}

	double Obstacles::distance(msl_msgs::Point2dInfo point, msl_msgs::PositionInfo pos)
	{
		double dx = (point.x - pos.x);
		double dy = (point.y - pos.y);

		return sqrt(dx * dx + dy * dy);
	}


	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > Obstacles::getObstaclePoints(int index)
	{
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ret = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
		auto x = obstacles.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		msl_sensor_msgs::ObstacleInfo current;
		for(int i = 0; i < x->getInformation()->size(); i++)
		{
			current = x->getInformation()->at(i);
			ret->push_back(make_shared<geometry::CNPoint2D>(current.x, current.y));
		}
		return ret;
	}

	shared_ptr<vector<msl_sensor_msgs::ObstacleInfo> > Obstacles::getObstacles(int index)
	{
		auto x = obstacles.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	void Obstacles::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
	{
		unsigned long time = wm->getTime();
//		if ((time - data->odometry.timestamp) > 1000)
//		{
//			return;
//		}

		if (data->obstacles.size() > 0)
		{
			shared_ptr<vector<msl_sensor_msgs::ObstacleInfo>> obs = make_shared<vector<msl_sensor_msgs::ObstacleInfo>>(
					data->obstacles);
			shared_ptr<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>> o = make_shared<InformationElement<vector<msl_sensor_msgs::ObstacleInfo>>>(obs,
					time);
			obstacles.add(o);
		}
	}

} /* namespace msl */

