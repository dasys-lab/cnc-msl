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
		/*PROTECTED REGION END*/
	}
	void SearchForPassPoint::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1436269017402) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1436269017402) ENABLED START*/ //Add additional methods here
	bool outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth, vector<shared_ptr<geometry::CNPoint2D>> points)
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

	bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth, vector<shared_ptr<geometry::CNPoint2D>> points)
	{
		for (int i = 0; i < points.size(); i++)
		{
			if (geometry::GeometryCalculator::distancePointToLineSegment(points[i]->x, points[i]->y, ball, passPoint) < passCorridorWidth
					&& ball->distanceTo(points[i]) < ball->distanceTo(passPoint) - 100)
			{
				return false;
			}
		}
		return true;
	}

	bool outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b, shared_ptr<geometry::CNPoint2D> c, double tolerance, vector<shared_ptr<geometry::CNPoint2D>> points)
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
