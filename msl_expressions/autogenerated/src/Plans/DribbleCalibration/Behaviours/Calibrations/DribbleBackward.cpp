/*
 * DribbleBackward.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: cn
 */

#include "Plans/DribbleCalibration/Behaviours/Calibrations/DribbleBackward.h"

DribbleBackward::DribbleBackward()
{
	// TODO Auto-generated constructor stub

}

DribbleBackward::~DribbleBackward()
{
	// TODO Auto-generated destructor stub
}

MotionControl DribbleBackward::move(int trans)
{
	MotionControl mc;
	return mCon.move(mCon.Backward, trans);
}

void DribbleBackward::writeConfigParameters()
{

}

void DribbleBackward::adaptParams()
{

}

