/*
 * RobotVisualization.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: Stefan Jakob
 */

#include <tools/basestation/RobotVisualization/RobotVisualization.h>

RobotVisualization::RobotVisualization()
{
	// TODO Auto-generated constructor stub

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

