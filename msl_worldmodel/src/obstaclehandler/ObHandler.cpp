/*
 * ObHandler.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: Stefan Jakob
 */

#include "obstaclehandler/ObHandler.h"
#include "MSLFootballField.h"

namespace msl
{

	ObHandler::ObHandler(MSLWorldModel* wm, int ringbufferLength)
	{
		this->wm = wm;
		sc = supplementary::SystemConfig::getInstance();
		DENSITY = (*sc)["PathPlanner"]->get<double>("PathPlanner.ObHandler.density", NULL);
		VARIANCE_THRESHOLD = DENSITY * DENSITY;
		TERRITORY_RADIUS = (*sc)["PathPlanner"]->get<double>("ObHandler.territoryRadius", NULL);
		SIGHT_RADIUS = (*sc)["PathPlanner"]->get<double>("ObHandler.sightradius"); // how far an obstacle can be seen proper
		FIELD_TOL = (*sc)["PathPlanner"]->get<double>("ObHandler.fieldTol");
		POS_CERTAINTY_TH_CLUSTERING = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyTHClustering", NULL);
		POS_CERTAINTY_HYS = (*sc)["PathPlanner"]->get<double>("PathPlanner.posCertaintyHys", NULL);
		DFLT_OB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "obstacleRadius", NULL);
		DFLT_ROB_RADIUS = (*sc)["PathPlanner"]->get<double>("PathPlanner", "teammateRadius", NULL);
		OBSTACLE_MAP_OUT_TOLERANCE = (*sc)["PathPlanner"]->get<double>("ObHandler", "obstacleMapOutTolerance", NULL);
		localizedBefore = false;
		field = MSLFootballField::getInstance();
		shared_ptr<vector<shared_ptr<AnnotatedObstacleCluster>>> clusterArray = make_shared<vector<shared_ptr<AnnotatedObstacleCluster>>>();
		shared_ptr<vector<shared_ptr<AnnotatedObstacleCluster>>> newClusterArray = make_shared<vector<shared_ptr<AnnotatedObstacleCluster>>>();
	}

	ObHandler::~ObHandler()
	{
	}

	void ObHandler::handleObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > myObstacles)
	{
//		if (vNet.OwnOdometry == null ||
//					   vNet.OwnOdometry.Position == null ||
//					   vNet.OwnOdometry.Certainty < (!localizedBefore ? POS_CERTAINTY_TH_CLUSTERING : POS_CERTAINTY_TH_CLUSTERING - POS_CERTAINTY_HYS))
//					{
//						#if OHDEBUG
//						Console.WriteLine ("OH: " + myProperties.Name + ": myPos == null");
//						#endif
//						vNet.ObsClustersAllo = null;
//						vNet.ObsEgo = ClusterPoint2D (myObstacles, VARIANCE_THRESHOLD);
//						vNet.OppEgo = new List<Point2D> (vNet.ObsEgo);
//						vNet.OppAllo = null;
//						vNet.TeammatesEgo = null;
//						vNet.ObsWithoutOppKeeperEgo = new List<Point2D> (vNet.ObsEgo);
//						localizedBefore = false;
//					}
//					else
//					{
//						localizedBefore = true;
//						// SETUP
//						//				Console.WriteLine ("OH: Before SetupAnnotatedObstacles");
//						SetupAnnotatedObstacles (myObstacles, vNet.OwnOdometry, myProperties, teamSHWMData, vNet);
//						// CLUSTERING
//						//				Console.WriteLine ("OH: Before ClusterAnnotatedObstacles");
//						ClusterAnnotatedObstacles (vNet);
//
//						// PRUNING
//						//				ProcessNegSupporter(teamSHWMData, myProperties, vNet.OwnOdometry.Position);
//
//						// CREATE DATASTRUCTURES FOR WM, DELAUNAY-GENERATOR, ETC.
//						//				Console.WriteLine ("OH: Before creating Lists");
//						C5.SortedArray<AnnotatedObstacleCluster> newObsClustersAllo = new C5.SortedArray<AnnotatedObstacleCluster> ();
//						List<Point2D> newObsEgo = new List<Point2D> ();
//						List<Point2D> newOppEgo = new List<Point2D> ();
//						List<Point2D> newOppAllo = new List<Point2D> ();
//						List<Point2D> newObsWithoutOppKeeperEgo = new List<Point2D> ();
//						List<Point2D> newTeammatesEgo = new List<Point2D> ();
//						List<Point2D> newTeammatesAllo = new List<Point2D> ();
//
//						Point2D curAlloPoint, curEgoPoint;
//						for (int i = 0; i < newClusterArray.Count; ++i)
//						{
//							newObsClustersAllo.Add (newClusterArray[i]);
//
//							curAlloPoint = new Point2D (newClusterArray[i].x, newClusterArray[i].y);
//							curEgoPoint = WorldHelper.Allo2Ego (curAlloPoint, vNet.OwnOdometry.Position);
//
//							if (newClusterArray[i].ident != myProperties.Id)
//							{
//								newObsEgo.Add (curEgoPoint);
//								if (!field.InsideEnemyKeeperArea (curAlloPoint, 0))
//								{
//									// egocentric obstacles, which are not inside the enemy keeper area and do not belong to our team
//									newObsWithoutOppKeeperEgo.Add (curEgoPoint);
//								}
//							}
//
//							if (newClusterArray[i].ident == -1)
//							{
//								// it is not a teammate
//								if (field.InsideField (curAlloPoint, FIELD_TOL))
//								{
//									newOppAllo.Add (curAlloPoint);
//									// egocentric obstacles, which are inside the field and do not belong to our team
//									newOppEgo.Add (curEgoPoint);
//								}
//							}
//							else if (newClusterArray[i].ident != myProperties.Id)
//							{
//								newTeammatesEgo.Add (curEgoPoint);
//								newTeammatesAllo.Add (curAlloPoint);
//							}
//						}
//
//						// change the vNet references to the new lists
//						vNet.ObsClustersAllo = newObsClustersAllo;
//						vNet.ObsEgo = newObsEgo;
//						vNet.OppEgo = newOppEgo;
//						vNet.OppAllo = newOppAllo;
//						vNet.TeammatesEgo = newTeammatesEgo;
//						vNet.TeammatesAllo = newTeammatesAllo;
//						vNet.ObsWithoutOppKeeperEgo = newObsWithoutOppKeeperEgo;
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ObHandler::clusterPoint2D(
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > obstacles, double varianceThreshold)
	{
//		List<Point2D> retList = new List<Point2D>();
//					List<SimpleCluster> clusterList = new List<SimpleCluster>();
//					bool mergedCluster = true;
//
//					// init cluster objects
//					for(int i = 0; i < obstacles.Count; ++i)
//					{
//		//				Console.WriteLine("1 Point2D: " + obstacles[i].X + " " + obstacles[i].Y);
//						clusterList.Add(new SimpleCluster(obstacles[i]));
//					}
//
//
//					while(mergedCluster)
//					{
//						// find the two nearest clusters
//						int fstClusterId = -1;
//						int sndClusterId = -1;
//						double minDist = double.MaxValue;
//						double curDist = 0;
//						for(int i = 0; i < clusterList.Count; ++i)
//						{
//							for(int j = 0; j < i; j++)
//							{
//								// check dist
//								curDist = clusterList[i].DistanceTo(clusterList[j]);
//								if (curDist < minDist)
//								{
//									fstClusterId = i;
//									sndClusterId = j;
//									minDist = curDist;
//								}
//							}
//						}
//
//						// check if variance after merging is below VARIANCE_THRESHOLD
//						if (fstClusterId != -1)
//						{
//							mergedCluster = clusterList[fstClusterId].CheckAndMerge(clusterList[sndClusterId], varianceThreshold);
//							if (mergedCluster)
//							{
//								clusterList.RemoveAt(sndClusterId);
//							}
//						}
//						else
//						{
//							mergedCluster = false;
//						}
//					}
//
//					for(int i = 0; i < clusterList.Count; ++i)
//					{
//						retList.Add(new Point2D(clusterList[i].X, clusterList[i].Y));
//		//				Console.WriteLine("2 Point2D: " + retList[i].X + " " + retList[i].Y);
//					}
//
//					return retList;
	}

	void ObHandler::clusterAnnotatedObstacles()
	{
		bool mergedCluster = true;

		while (mergedCluster)
		{
			// find the two nearest mergeable clusters
			int fstClusterId = -1;
			int sndClusterId = -1;
			double minDist = numeric_limits<double>::max();
			double curDist = 0;
			for (int i = 0; i < clusterArray->size(); ++i)
			{
				for (int j = 0; j < i; ++j)
				{
					if ((clusterArray->at(i)->ident == -1 || clusterArray->at(j)->ident == -1)
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
			if (fstClusterId != -1)
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

		newClusterArray = make_shared<vector<shared_ptr<AnnotatedObstacleCluster>>>();
		for(int i = 0; i < clusterArray->size(); i++)
		{
			newClusterArray->push_back(clusterArray->at(i));
		}
		std::sort(newClusterArray->begin(), newClusterArray->end(), AnnotatedObstacleCluster::compareTo);
		clusterArray->clear();
	}

	void ObHandler::setupAnnotatedObstacles(shared_ptr<vector<shared_ptr<geometry::CNPoint2D> > > ownObs,
											msl_sensor_msgs::CorrectedOdometryInfo myOdo)
	{
//		clusterArray.Clear();
//					AnnotatedObstacleCluster obs = null;
//					int velX = 0;
//					int velY = 0;
//					foreach(RobotSHWMData curRobot in teamSHWMData.Values)
//					{
//						/* Ignore every robot, which:
//						 * - is unlocalised
//						 * - messages are old
//						 * - I am (the obstacles from the WM are newer)*/
//						Position curPlayerPosition = curRobot.playerPosition;
//						if(!curRobot.positionReceived
//						   || curPlayerPosition == null
//						   || curRobot.properties.Id == myProperties.Id
//						   || curRobot.timeLastOpponentsReceived + 250*10000 < DateTime.UtcNow.Ticks
//						   )
//						{
//							continue;
//						}
//
//						// add all obstacles seen by curRobot
//						List<Point2D> curOppList = curRobot.oppList;
//						if (curOppList != null)
//						{
//							for(int i = 0; i < curOppList.Count;  ++i)
//							{
//								// Nobody knows the positions of obstacles arround me better than I do!
//								if(Geometry.Distance(curOppList[i],myOdo.Position) < TERRITORY_RADIUS)
//								{
//									continue;
//								}
//
//								if (FootballField.GetInstance().InsideField(curOppList[i], OBSTACLE_MAP_OUT_TOLERANCE))
//								{
//									obs = vNet.GetCluster();
//									obs.Init((int) (curOppList[i].X+0.5), (int) (curOppList[i].Y+0.5), // pos
//									         DFLT_OB_RADIUS,
//									         -1, curRobot.properties.Id);
//									clusterArray.Add(obs);
//								}
//							}
//						}
//
//						// TODO: something to identify the other robot, when he is close to me
//
//						/* add the curRobot itself as an obstacle, to identify teammates */
//
//						// Convert ego motion angle to allo motion angle
//						double alloMotionAngle = curRobot.playerPosition.Angle + curRobot.motion.Angle;
//						if (alloMotionAngle > Math.PI) {
//							alloMotionAngle -= 2*Math.PI;
//						} else if (alloMotionAngle < -Math.PI) {
//							alloMotionAngle += 2*Math.PI;
//						}
//
//						// Calc x and y the velocity
//						velX = (int) (Math.Cos(alloMotionAngle)*curRobot.motion.Translation+0.5);
//						velY = (int) (Math.Sin(alloMotionAngle)*curRobot.motion.Translation+0.5);
//
//						// predict the position along the translation
//						double seconds = (double) (DateTime.UtcNow.Ticks - curRobot.timeLastPositionEvent) / 10000000.0;
//
//						obs = vNet.GetCluster();
//						obs.Init((int) (curPlayerPosition.X + seconds * velX + 0.5), (int) (curPlayerPosition.Y + seconds * velY + 0.5), // pos
//						         DFLT_ROB_RADIUS,
//						         velX, velY,													   // velocity
//						         curRobot.properties.Id,
//						         curRobot.properties.Id);
//						clusterArray.Add(obs);
//					}
//
//					/* add my own obstacles from the worldmodel (they are egocentric :-( ) */
//					Point2D curPoint;
//					for(int i=0; i < ownObs.Count; ++i)
//					{
//						curPoint = WorldHelper.Ego2Allo(ownObs[i], myOdo.Position);
//						if (FootballField.GetInstance().InsideField(curPoint, OBSTACLE_MAP_OUT_TOLERANCE))
//						{
//							obs = vNet.GetCluster();
//							obs.Init((int) (curPoint.X+0.5),
//							         (int) (curPoint.Y+0.5),
//							         DFLT_OB_RADIUS, -1, myProperties.Id);
//							clusterArray.Add(obs);
//						}
//					}
//
//					/* add my own position: */
//
//					// Convert ego motion angle to allo motion angle
//					double alloMotAngle = myOdo.Position.Angle + myOdo.Motion.Angle;
//					if (alloMotAngle > Math.PI) {
//						alloMotAngle -= 2*Math.PI;
//					} else if (alloMotAngle < -Math.PI) {
//						alloMotAngle += 2*Math.PI;
//					}
//
//					velX = (int)Math.Round (myOdo.Motion.Translation * Math.Cos (alloMotAngle));
//					velY = (int)Math.Round (myOdo.Motion.Translation * Math.Sin (alloMotAngle));
//					obs = vNet.GetCluster();
//					obs.Init((int) (myOdo.Position.X+0.5),
//					    (int) (myOdo.Position.Y+0.5),
//						myOdo.Position.Angle,
//					    DFLT_ROB_RADIUS, velX, velY,
//						myOdo.Motion.Rotation,
//						myOdo.Position.Certainty, myProperties.Id,
//					    myProperties.Id);
//					clusterArray.Add(obs);
	}

	void ObHandler::processNegSupporter(shared_ptr<geometry::CNPosition> myPosition)
	{
//		double curAngle, curAngle2,  dangle, dangle2, left, right, left2, right2, curDist, curDist2;
//					Point2D curPoint, curPoint2;
//					bool sightIsBlocked;
//
//					foreach(RobotSHWMData curRobot in teamSHWMData.Values)
//					{
//		//				Console.WriteLine("Robot: " + curRobot.properties.Name);
//						/* Ignore every robot, which is:
//						 * - unlocalised
//						 * - myself
//						 */
//						if(!curRobot.positionReceived
//						   || curRobot.playerPosition == null
//						   || curRobot.properties.Id == myProperties.Id)
//						{
//		//				 	Console.WriteLine("Skip");
//							continue;
//						}
//
//						for(int i=0; i < newClusterArray.Count; ++i)
//						{
//		//					Console.WriteLine("Cluster: " + (newClusterArray[i].X/1000.0).ToString("F") + " " + (newClusterArray[i].Y/1000.0).ToString("F"));
//							// continue, if the curRobot is a supporter of the curCluster or
//							// the curCluster is out of the sight of the curRobot or
//							// the curCluster is near me (<TERRITORY_RADIUS) so nobody was allowed to merg
//							if (newClusterArray[i].DistanceTo(myPosition) < TERRITORY_RADIUS)
//							{
//								continue;
//							}
//							curDist = newClusterArray[i].DistanceTo(curRobot.playerPosition);
//							if (curDist > SIGHT_RADIUS)
//							{
//		//						Console.WriteLine("Too far away!");
//								continue;
//							}
//
//							if (newClusterArray[i].supporter.Contains(curRobot.properties.Id))
//							{
//		//						Console.WriteLine("I am supporter!");
//								continue;
//							}
//
//							curPoint = new Point2D(newClusterArray[i].x - curRobot.playerPosition.X,
//							                       newClusterArray[i].y - curRobot.playerPosition.Y);
//							curAngle = Math.Atan2(curPoint.Y, curPoint.X);
//							dangle = Math.Abs(Math.Asin(DENSITY/curDist));
//
//							// normalize angles
//							left = curAngle+dangle;
//							if(left < -Math.PI)
//								left += 2.0*Math.PI;
//							else if(left > Math.PI)
//								left -= 2.0*Math.PI;
//
//							right = curAngle-dangle;
//							if(right < -Math.PI)
//								right += 2.0*Math.PI;
//							else if(left > Math.PI)
//								right -= 2.0*Math.PI;
//
//		//					Console.WriteLine("Cluster Angels: \n\tleft: " + (left * 180)/ Math.PI
//		//					                  + "\n\tmiddle: " + (curAngle * 180)/Math.PI
//		//					                  + "\n\tright: " + (right * 180)/Math.PI
//		//					                  + "\n\tdangle: " + (dangle * 180)/Math.PI);
//
//							sightIsBlocked = false;
//
//							// Für jedes Obstacle überprüfen, ob es im Weg steht
//							if (curRobot.oppList != null)
//							{
//								for(int j=0; j < curRobot.oppList.Count; ++j)
//								{
//			//						Console.WriteLine("Own Obstacle: "+ (curRobot.oppList[j].X/1000.0).ToString("F") + " " + (curRobot.oppList[j].Y/1000.0).ToString("F"));
//
//									curPoint2 = new Point2D(curRobot.oppList[j].X - curRobot.playerPosition.X,
//									                        curRobot.oppList[j].Y - curRobot.playerPosition.Y);
//
//									curDist2 = curPoint2.Distance();
//									if(curDist2 < curDist)
//									{// the curPoint2 is closer then curPoint
//										curAngle2 = Math.Atan2(curPoint2.Y, curPoint2.X);
//										dangle2 = Math.Abs(Math.Asin(DENSITY/curDist2));
//
//										// normalize angles
//										left2 = curAngle2+dangle2;
//										if(left2 < -Math.PI)
//											left2 += 2.0*Math.PI;
//										else if(left2 > Math.PI)
//											left2 -= 2.0*Math.PI;
//
//										right2 = curAngle2-dangle2;
//										if(right2 < -Math.PI)
//											right2 += 2.0*Math.PI;
//										else if(right2 > Math.PI)
//											right2 -= 2.0*Math.PI;
//
//			//							Console.WriteLine("Own Obstacle Angels: \n\tleft: " + (left2 * 180)/ Math.PI
//			//							                  + "\n\tmiddle: " + (curAngle2 * 180)/Math.PI
//			//							                  + "\n\tright: " + (right2 * 180)/Math.PI
//			//							                  + "\n\tdangle: " + (dangle2 * 180)/Math.PI);
//
//										if (leftOf(left, right2) && !leftOf(left,left2))
//										{
//			//								Console.WriteLine("Left of Cluster is behind Obstacle");
//											sightIsBlocked = true;
//											break;
//										}
//
//										if (leftOf(right, right2) && !leftOf(right, left2))
//										{
//			//								Console.WriteLine("Right of Cluster is behind Obstacle");
//											sightIsBlocked = true;
//											break;
//										}
//
//			//							Console.WriteLine("Obstacle does not block the sight!");
//									}
//			//						else
//			//						{
//			//							Console.WriteLine("Own obstacle is too far away!");
//			//						}
//								}
//							}
//							else
//							{
//								sightIsBlocked = true;
//							}
//
//							// Wenn die Sicht nicht blockiert ist bin ich gegen das Obstacle
//							if (!sightIsBlocked)
//							{
//								newClusterArray[i].opposer.Add(curRobot.properties.Id);
//							}
//						}
//					}
//
//					// Lösche einfach alle Obstacle, die mehr Opposer als Supporter haben
//					for(int i = 0; i < newClusterArray.Count; ++i)
//					{
//						if(newClusterArray[i].supporter.Count < newClusterArray[i].opposer.Count)
//						{
//							Console.WriteLine("OH: removed this obstacle X: " + newClusterArray[i].x + " Y:" + newClusterArray[i].y);
//							newClusterArray.RemoveAt(i);
//						}
//					}
	}

	bool ObHandler::leftOf(double angle1, double angle2)
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

} /* namespace msl */

