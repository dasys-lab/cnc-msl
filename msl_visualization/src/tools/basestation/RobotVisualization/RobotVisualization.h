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
#include <vtkTextActor.h>
#include <vtkRegularPolygonSource.h>
#include <map>
#include <memory>
#include <list>

class RobotInfo;
class FieldWidget3D;

class RobotVisualization
{
public:
	RobotVisualization(RobotInfo* robot, FieldWidget3D* field);
	virtual ~RobotVisualization();
	vtkSmartPointer<vtkActor> getBottom();
	void setBottom(vtkSmartPointer<vtkActor> bottom);
	vtkSmartPointer<vtkActor> getTop();
	void setTop(vtkSmartPointer<vtkActor> top);
	void setNameActor(vtkSmartPointer<vtkActor> nameActor);
	vtkSmartPointer<vtkActor> getNameActor();
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
	vtkSmartPointer<vtkActor> getSharedBall();
	void setSharedBall(vtkSmartPointer<vtkActor> sharedBall);

	void hide(vtkRenderer *renderer);
	void show(vtkRenderer *renderer);
        void init(vtkRenderer *renderer);
	void updatePosition(vtkRenderer *renderer);
        void updateBall(vtkRenderer *renderer);
        void updateSharedBall(vtkRenderer *renderer);
        void updateOpponents(vtkRenderer *renderer);

        void updatePathPlannerDebug(vtkRenderer *renderer, bool show);
        void updateCorridorDebug(vtkRenderer *renderer, bool show);
        void updateVoronoiNetDebug(vtkRenderer *renderer, bool showVoronoi, bool showSitePoints);

private:
        void move(double x, double y, double z);
        void turn(double angle);
        void drawOpponent(vtkRenderer *renderer, double x, double y, double z);
        std::array<double,3>& getColor();

private:
	RobotInfo* robot;
        FieldWidget3D* field;

	std::string name = "";
	int id = 0;
	int senderId = 0;
	vtkSmartPointer<vtkActor> top = nullptr;
	vtkSmartPointer<vtkActor> bottom = nullptr;
	vtkSmartPointer<vtkActor> nameActor = nullptr;
        vtkActor* ball = nullptr;
        vtkSmartPointer<vtkLineSource> ballVelocity = nullptr;
	vtkSmartPointer<vtkActor> ballVelocityActor = nullptr;
	vtkSmartPointer<vtkActor> sharedBall = nullptr;
	std::list<std::shared_ptr<RobotVisualization>> obstacles;
        std::vector<vtkSmartPointer<vtkActor>> pathLines;
        std::vector<vtkSmartPointer<vtkActor>> netLines;
        std::vector<vtkSmartPointer<vtkActor>> corridorLines;
        std::vector<vtkSmartPointer<vtkActor>> sitePoints;


};

#endif /* CNC_MSL_MSL_VISUALIZATION_SRC_TOOLS_BASESTATION_ROBOTVISUALIZATION_H_ */
