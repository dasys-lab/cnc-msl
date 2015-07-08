using namespace std;
#include "Plans/Attack/SearchForPassPoint.h"

/*PROTECTED REGION ID(inccpp1436269017402) ENABLED START*/ //Add additional includes here
#include <GeometryCalculator.h>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1436269017402) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	SearchForPassPoint::SearchForPassPoint() :
			DomainBehaviour("SearchForPassPoint")
	{
		/*PROTECTED REGION ID(con1436269017402) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	SearchForPassPoint::~SearchForPassPoint()
	{
		/*PROTECTED REGION ID(dcon1436269017402) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void SearchForPassPoint::run(void* msg)
	{
		/*PROTECTED REGION ID(run1436269017402) ENABLED START*/ //Add additional options here
//		if (eps.size() <= 0)
//		{
//			cout << "S4PP: All EPs is null" << endl;
//			return;
//		}
//
//		shared_ptr<geometry::CNPoint2D> alloBall = this->wm->ball.getAlloBallPosition();
//		if (alloBall == nullptr)
//		{
//			cout << "S4PP: Ball is null" << endl;
//			return;
//		}
//
//		shared_ptr<geometry::CNPosition> alloPos = this->wm->rawSensorData.getOwnPositionVision();
//		if (alloPos == nullptr)
//		{
//			cout << "S4PP: OwnPos is null" << endl;
//			return;
//		}
//
//		// ensures, that we have the ball and are not in melee with some opp.
//		if (!true && this.WM.SharedWorldModel.GetGameState() != GameState.Attack)
//		{
//			Console.WriteLine("S4PP: Gamestate is not Attack");
//			return;
//		}
//
//		// the only teammate in the corresponding task/ entrypoint
//		teamMateIds.clear();
//		for(EntryPoint* ep : eps)
//		{
//			auto teammates = robotsInEntryPointOfHigherPlan(ep);
//			for(int mateId : teammates)
//			{
//				this->teamMateIds.push_back(mateId);
//				break;
//			}
//		}
//		if (teamMateIds.size() <= 0)
//		{
//			cout << "S4PP: Somethine Strange is going on with RobotIDs and Entrypoints" << endl;
//			return;
//		}
//
//		VoronoiNet vNet = this.WM.GetCurrentVoronoiNet();
//		if (vNet == null)
//			return;
//		try
//		{
//#ifdef BEH_DEBUG
//			DebugMsg dm = new DebugMsg();
//			foreach(VEdge edge in vNet.VNetEdges)
//			{
//				Point2dInfo p = new Point2dInfo();
//				p.X = edge.p1.p.X;
//				p.Y = edge.p1.p.Y;
//				Edge ed = new Edge();
//				ed.P1 = p;
//
//				p = new Point2dInfo();
//				p.X = edge.p2.p.X;
//				p.Y = edge.p2.p.Y;
//				ed.P2 = p;
//
//				//dm.Edges.Add(ed);
//			}
//#endif
//
//		foreach(int teamMateId in this.teamMateIds)
//		{
//
//			List<VNode> vNodes = vNet.GetTeamMateVNodes(teamMateId);
//			AnnotatedObstacleCluster aoc = vNet.TeamCells[teamMateId];
//			for (int i = 0; i < vNodes.Count; i++)
//			{
//				//Console.WriteLine("Neighbour Point: " + vNodes[i].p);
//				// make the passpoints closer to the receiver
//				Point2D passPoint = vNodes[i].p;
//				Point2D receiver = new Point2D(aoc.x, aoc.y);
//				Point2D rcv2PassPoint = passPoint - receiver;
//				double rcv2PassPointDist = rcv2PassPoint.Distance();
//				double factor = closerFactor;
//				if (factor * rcv2PassPointDist > minCloserOffset)
//				{
//					factor = factor * rcv2PassPointDist;
//				}
//				else
//				{
//					factor = rcv2PassPointDist - minCloserOffset;
//				}
//				factor = Math.Max (factor, 0.0);
//				passPoint = receiver + rcv2PassPoint.Normalize() * factor;
//#ifdef BEH_DEBUG
//				Point2dInfo tmp = new Point2dInfo();
//				tmp.X = passPoint.X;
//				tmp.Y = passPoint.Y;
//				dm.Points.Add(tmp);
//				dm.Values.Add(0.0);
//#endif
//
//				if (ff.InsideField (passPoint, distToFieldBorder) // pass point must be inside the field with distance to side line of 1.5 metre
//						&& !ff.InsidePenalty(passPoint, 0.0)
//						&& alloBall.DistanceTo(passPoint) < maxPassDist// max dist to pass point
//						&& alloBall.DistanceTo(passPoint) > minPassDist// min dist to pass point
//				)
//				{
//
//					// min dist to opponent
//					if ((vNodes[i].tri.p[0].ident == -1 && vNodes[i].tri.p[0].DistanceTo(passPoint) < minOppDist)
//							||(vNodes[i].tri.p[1].ident == -1 && vNodes[i].tri.p[1].DistanceTo(passPoint) < minOppDist)
//							||(vNodes[i].tri.p[2].ident == -1 && vNodes[i].tri.p[2].DistanceTo(passPoint) < minOppDist))
//					{
//#ifdef BEH_DEBUG
//						dm.Values[dm.Values.Count-1] = 0.2;
//#endif
//						continue;
//					}
//
//					// small angle to turn to pass point
//					if (GeometryHelper.AbsDeltaAngle(alloPos.Angle+Math.PI, (passPoint - alloPos).Angle()) > maxTurnAngle)
//					{
//#ifdef BEH_DEBUG
//						dm.Values[dm.Values.Count-1] = 0.4;
//#endif
//						continue;
//					}
//
//					// some calculation to check whether any opponent is inside the pass vector triangle
//					Point2D ball2PassPoint = passPoint - alloBall;
//					double passLength = ball2PassPoint.Distance();
//					Point2D ball2PassPointOrth = new Point2D (-ball2PassPoint.Y, ball2PassPoint.X).Normalize() * ratio * passLength;
//					Point2D left = passPoint + ball2PassPointOrth;
//					Point2D right = passPoint - ball2PassPointOrth;
//					if (!OutsideTriangle(alloBall, right, left, ballRadius,vNet.OppAllo)
//							&& !OutsideCorridore(alloBall, passPoint, this.passCorridorWidth, vNet.OppAllo))
//					{
//#ifdef BEH_DEBUG
//						dm.Values[dm.Values.Count-1] = 0.6;
//#endif
//						continue;
//					}
//
//					// no opponent was in dangerous distance to our pass vector, now check our teammates with other parameters
//					if (!OutsideCorridoreTeammates(alloBall, passPoint, this.ballRadius*4, vNet.TeammatesAllo))
//					{
//#ifdef BEH_DEBUG
//						dm.Values[dm.Values.Count-1] = 0.8;
//#endif
//						continue;
//					}
//					else
//					{
//						this.SuccessStatus = true;
//#ifdef BEH_DEBUG
//						dm.Values[dm.Values.Count-1] = 1.0;
//#else
//						return;
//#endif
//					}
//				}
//
//			}
//		}
//	}
//	catch (exception& e)
//	{
//		throw e;
//	}

	/*PROTECTED REGION END*/
}
void SearchForPassPoint::initialiseParameters()
{
	/*PROTECTED REGION ID(initialiseParameters1436269017402) ENABLED START*/ //Add additional options here
	teamMatePlanName.clear();
	teamMateTaskName.clear();
	sc = supplementary::SystemConfig::getInstance();
	this->minCloserOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "MinCloserOffset", NULL);
	this->closerFactor = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor", NULL);
	string tmp;
	string tmp2;
	bool success = true;
	success &= getParameter("DistToFieldBorder", tmp);
	try
	{
		if (success)
		{
			this->distToFieldBorder = stod(tmp);
		}

		success &= getParameter("FreeOppAngle", tmp);
		if (success)
		{
			this->freeOppAngle = stod(tmp) / 2;
			this->ratio = tan(freeOppAngle);
		}
		success &= getParameter("MaxPassDist", tmp);
		if (success)
		{
			this->maxPassDist = stod(tmp);
		}
		success &= getParameter("MaxTurnAngle", tmp);
		if (success)
		{
			this->maxTurnAngle = stod(tmp);
		}
		success &= getParameter("MinOppDist", tmp);
		if (success)
		{
			this->minOppDist = stod(tmp);
		}
		success &= getParameter("MinPassDist", tmp);
		if (success)
		{
			this->minPassDist = stod(tmp);
		}
		success &= getParameter("PassCorridorWidth", tmp);
		if (success)
		{
			this->passCorridorWidth = stod(tmp);
		}
		int iter = 0;
		stringstream ss;
		stringstream ss2;
		while (true)
		{
			ss << "TeamMateTaskName" << iter;
			ss2 << "TeamMatePlanName" << iter;
			if (getParameter(ss.str(), tmp) && getParameter(ss2.str(), tmp2))
			{
				teamMateTaskName.push_back(tmp);
				teamMatePlanName.push_back(tmp2);
			}
			else
			{
				break;
			}
			ss.str("");
			ss2.str("");
			iter++;
		}
		eps.clear();
		for (int i = 0; i < teamMatePlanName.size(); i++)
		{
			EntryPoint* ep = getHigherEntryPoint(teamMatePlanName[i], teamMateTaskName[i]);
			if (ep != nullptr)
			{
				eps.push_back(ep);
			}
		}

	}
	catch (exception& e)
	{
		cerr << "Could not cast the parameter properly" << endl;
	}
	if (!success)
	{
		cerr << "Parameter does not exist" << endl;
	}
	/*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1436269017402) ENABLED START*/ //Add additional methods here
bool outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint,
						double passCorridorWidth, vector<shared_ptr<geometry::CNPoint2D>> points)
{
	for (int i = 0; i < points.size(); i++)
	{
		if (geometry::GeometryCalculator::distancePointToLineSegment(points[i]->x, points[i]->y, ball, passPoint)
				< passCorridorWidth)
		{
			return false;
		}
	}
	return true;
}

bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint,
								double passCorridorWidth, vector<shared_ptr<geometry::CNPoint2D>> points)
{
	for (int i = 0; i < points.size(); i++)
	{
		if (geometry::GeometryCalculator::distancePointToLineSegment(points[i]->x, points[i]->y, ball, passPoint)
				< passCorridorWidth && ball->distanceTo(points[i]) < ball->distanceTo(passPoint) - 100)
		{
			return false;
		}
	}
	return true;
}

bool outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
						shared_ptr<geometry::CNPoint2D> c, double tolerance,
						vector<shared_ptr<geometry::CNPoint2D>> points)
{
	shared_ptr<geometry::CNPoint2D> a2b = b - a;
	shared_ptr<geometry::CNPoint2D> b2c = c - b;
	shared_ptr<geometry::CNPoint2D> c2a = a - c;

	shared_ptr<geometry::CNPoint2D> a2p;
	shared_ptr<geometry::CNPoint2D> b2p;
	shared_ptr<geometry::CNPoint2D> c2p;
	shared_ptr<geometry::CNPoint2D> p;
	for (int i = 0; i < points.size(); i++)
	{
		p = points[i];
		a2p = p - a;
		b2p = p - b;
		c2p = p - c;

		if ((a2p->x * a2b->y - a2p->y * a2b->x) / a2p->normalize()->length() < tolerance
				&& (b2p->x * b2c->y - b2p->y * b2c->x) / b2p->normalize()->length() < tolerance
				&& (c2p->x * c2a->y - c2p->y * c2a->x) / c2p->normalize()->length() < tolerance)
		{
			return false;
		}

	}
	return true;
}
/*PROTECTED REGION END*/
} /* namespace alica */
