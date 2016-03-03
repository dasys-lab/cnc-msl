/*
 * RobotVisualization.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: Stefan Jakob
 */

#include <tools/basestation/RobotVisualization/RobotVisualization.h>

RobotVisualization::RobotVisualization()
{
	id = 0;
	senderId = 0;
}

RobotVisualization::~RobotVisualization()
{
	// TODO Auto-generated destructor stub
}

vtkSmartPointer<vtkActor> RobotVisualization::getBottom()
{
	return bottom;
}

void RobotVisualization::setBottom(vtkSmartPointer<vtkActor> bottom)
{
	this->bottom = bottom;
}

void RobotVisualization::setNameActor(vtkSmartPointer<vtkActor> nameActor)
{
	this->nameActor = nameActor;
}

vtkSmartPointer<vtkActor> RobotVisualization::getNameActor()
{
	return this->nameActor;
}

vtkSmartPointer<vtkActor> RobotVisualization::getTop()
{
	return top;
}

void RobotVisualization::setTop(vtkSmartPointer<vtkActor> top)
{
	this->top = top;
}
int RobotVisualization::getId()
{
	return id;
}

void RobotVisualization::setId(int id)
{
	this->id = id;
}

 std::string RobotVisualization::getName()
{
	return name;
}

void RobotVisualization::setName(std::string name)
{
	this->name = name;
}

int RobotVisualization::getSenderId()
{
	return senderId;
}

void RobotVisualization::setSenderId(int senderId)
{
	this->senderId = senderId;
}

vtkActor* RobotVisualization::getBall()
{
	return ball;
}

void RobotVisualization::setBall(vtkActor* ball)
{
	this->ball = ball;
}

vtkSmartPointer<vtkLineSource> RobotVisualization::getBallVelocity()
{
	return ballVelocity;
}

void RobotVisualization::setBallVelocity(vtkSmartPointer<vtkLineSource> ballVelocity)
{
	this->ballVelocity = ballVelocity;
}

vtkSmartPointer<vtkActor> RobotVisualization::getBallVelocityActor()
{
	return ballVelocityActor;
}

void RobotVisualization::setBallVelocityActor(vtkSmartPointer<vtkActor> ballVelocityActor)
{
	this->ballVelocityActor = ballVelocityActor;
}

vtkSmartPointer<vtkActor> RobotVisualization::getSharedBall()
{
	return sharedBall;
}

void RobotVisualization::setSharedBall(vtkSmartPointer<vtkActor> sharedBall)
{
	this->sharedBall = sharedBall;
}
