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

private:
	std::string name;
	int id;
	vtkSmartPointer<vtkActor> top;
	vtkSmartPointer<vtkActor> bottom;
};

#endif /* CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_ROBOTVISUALIZATION_H_ */
