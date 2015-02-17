/*
 * RobotVisualization.h
 *
 *  Created on: Feb 11, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_ROBOTVISUALIZATION_H_
#define CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_ROBOTVISUALIZATION_H_

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <string>
#include <vtkLineSource.h>
#include <vtkRegularPolygonSource.h>


class RobotVisualization
{
public:
	RobotVisualization();
	virtual ~RobotVisualization();
	vtkSmartPointer<vtkActor> getBottom();
	void setBottom(vtkSmartPointer<vtkActor> bottom);
	vtkSmartPointer<vtkActor> getTop();
	void setTop(vtkSmartPointer<vtkActor> top);
	int getId();
	void setId(int id);
	std::string getName();
	void setName(std::string name);
	int getSenderId();
	void setSenderId(int senderId);
	vtkActor* getBall();
	void setBall(vtkActor* ball);
	vtkSmartPointer<vtkLineSource> getBallVelocity();
	void setBallVelocity(vtkSmartPointer<vtkLineSource> ballVelocity);
	vtkSmartPointer<vtkActor> getBallVelocityActor();
	void setBallVelocityActor(vtkSmartPointer<vtkActor> ballVelocityActor);
	vtkSmartPointer<vtkRegularPolygonSource> getPolygonSource();
	void setPolygonSource(vtkSmartPointer<vtkRegularPolygonSource> polygonSource);

private:
	std::string name = "";
	int id = 0;
	int senderId = 0;
	vtkSmartPointer<vtkActor> top = nullptr;
	vtkSmartPointer<vtkActor> bottom = nullptr;
    vtkActor* ball = nullptr;
    vtkSmartPointer<vtkLineSource> ballVelocity = nullptr;
	vtkSmartPointer<vtkActor> ballVelocityActor = nullptr;
	vtkSmartPointer<vtkRegularPolygonSource> polygonSource = nullptr;
};

#endif /* CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_ROBOTVISUALIZATION_H_ */
