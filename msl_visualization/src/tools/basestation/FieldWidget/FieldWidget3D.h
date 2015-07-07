/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA BASESTATION
 *
 * CAMBADA BASESTATION is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA BASESTATION is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FIELDWIDGET3D_H
#define FIELDWIDGET3D_H

#include <QTimer>
#include <QFile>
#include <QtGui>
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include <mutex>
#include <ros/ros.h>
#include <list>
#include "src/tools/basestation/RobotVisualization/RobotVisualization.h"
#include <thread>
#include <atomic>
#include <math.h>

#include <QVTKWidget.h>

#include <vtkCallbackCommand.h>
#include <vtkSmartPointer.h>
#include <vtkLineSource.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextSource.h>
#include <vtkCylinderSource.h>
#include <vtkVectorText.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkOBJReader.h>
#include <vtkTriangleStrip.h>
#include <vtkCellArray.h>
#include <vtkDataSetMapper.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTextWidget.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkViewport.h>
#include <vtkPointWidget.h>
#include <vtkPolygon.h>
#include <vtkPNGReader.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageData.h>
#include <vtkImageMapper.h>
#include <vtkPropPicker.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkPropCollection.h>
#include <vtkDelaunay2D.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkCubeSource.h>
#include <vtkPyramid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>
#include <vtkRegularPolygonSource.h>
#include <vtkTetra.h>

#include <QVTKInteractor.h>

#include "RobotInfo.h"
#include "msl_msgs/PathPlanner.h"
#include "msl_msgs/CorridorCheck.h"
#include "msl_msgs/VoronoiNetInfo.h"
#include <SystemConfig.h>
#include <vtkArrowSource.h>

#define OBSTACLE_HEIGHT 0.2

class FieldWidget3D : public QVTKWidget
{
    Q_OBJECT
public:
    explicit FieldWidget3D(QWidget *parent = 0);
    list<boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> > getSavedSharedWorldInfo();


    vtkRenderWindow *renderWindow = nullptr;
    vtkRenderer *renderer = nullptr;
    vtkCamera* camera = nullptr;

    vtkActor* field = nullptr;

    bool heightVisible;
    bool heightColor;
    bool height3D;
    bool top;
    bool lockCam;
    void setTop(bool top);


private:
    ros::AsyncSpinner* spinner;
    void onSharedWorldInfo(boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> info);
    void onPathPlannerMsg(boost::shared_ptr<msl_msgs::PathPlanner> info);
    void onVoronoiNetMsg(boost::shared_ptr<msl_msgs::VoronoiNetInfo> info);
    void onCorridorCheckMsg(boost::shared_ptr<msl_msgs::CorridorCheck> info);
    void moveBall(shared_ptr<RobotVisualization> robot, boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> info, double x, double y, double z);
    void moveSharedBall(shared_ptr<RobotVisualization> robot, double x, double y, double z);
    void drawOpponent(double x, double y, double z);
    void drawTeamRobot(shared_ptr<RobotVisualization> robot, double x, double y, double z);
    list<shared_ptr<RobotVisualization>> obstacles;
    list<shared_ptr<RobotVisualization>> team;
    list<shared_ptr<RobotInfo>> latestInfo;
    bool showPath;
    bool showVoronoi;
    bool showCorridor;
    bool showSitePoints;
    bool showAllComponents;
    void removeObstacles(vtkRenderer* renderer);
    void moveRobot(shared_ptr<RobotVisualization> robot, double x, double y, double z);
    void turnRobot(shared_ptr<RobotVisualization> robot, double angle);
    mutex swmMutex;
    mutex pathMutex;
    mutex voronoiMutex;
    mutex corridorMutex;
	list<boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo>> savedSharedWorldInfo;
    list<boost::shared_ptr<msl_msgs::PathPlanner>> pathPlannerInfo;
    list<boost::shared_ptr<msl_msgs::VoronoiNetInfo>> voronoiNetInfo;
    list<boost::shared_ptr<msl_msgs::CorridorCheck>> corridorCheckInfo;
    vector<vtkActor*> pathLines;
    vector<vtkActor*> netLines;
    vector<vtkActor*> corridorLines;
    vector<vtkActor*> sitePoints;
	ros::Subscriber sharedWorldInfoSubscriber;
	ros::Subscriber pathPlannerSubscriber;
	ros::Subscriber voronoiSitesSubscriber;
	ros::Subscriber corridorCheckSubscriber;
	ros::NodeHandle* rosNode;
	int ringBufferLength = 10;
    QWidget* parent;
    double _FIELD_LENGTH;
    double _FIELD_WIDTH;
    double _LINE_THICKNESS;
    double _GOAL_AREA_LENGTH;
    double _GOAL_AREA_WIDTH;
    double _PENALTY_AREA_LENGTH;
    double _PENALTY_AREA_WIDTH;
    double _CENTER_CIRCLE_RADIUS;
    double _BALL_DIAMETER;
    double _CORNER_CIRCLE_RADIUS;
    double _PENALTY_MARK_DISTANCE;
    double _BLACK_POINT_WIDTH;
    double _BLACK_POINT_LENGTH;
    double _ROBOT_RADIUS;


    vtkSmartPointer<vtkActor> createLine(float x1, float y1, float z1, float x2, float y2, float z2);
    vtkSmartPointer<vtkActor> createColoredDashedLine(float x1, float y1, float z1, float x2, float y2, float z2, double r, double g, double b);
    vtkSmartPointer<vtkActor> createColoredDot(float x, float y, float radius, double r, double g, double b);
    void addArc(vtkRenderer* renderer, float x, float y, float radius, float startDeg, float endDeg);
    void drawField(vtkRenderer* renderer);
    void drawGoals(vtkRenderer* renderer);
    void initBall(shared_ptr<RobotVisualization> robot, vtkRenderer* renderer);
    void initSharedBall(shared_ptr<RobotVisualization> robot, vtkRenderer* renderer);
    void initGridView();
    void updateGridView();
    void deleteGridView();

    vtkActor* createText(QString text);
    vtkActor* createObstacle();
    vtkActor* createDebugPt();
    vtkActor* createDashedLine(float x1, float y1, float z1, float x2, float y2, float z2);
    void createDot(vtkRenderer* renderer, float x, float y, bool black, float radius=0.05);
    pair<double, double> transform(double x, double y);

    vtkPoints* heightPoints;
    vtkPolyData* heightPolyData;
    vtkDelaunay2D* heightDelaunay;
    vtkPolyData* heightPolyDataAfterInterp;
    vtkActor* heightActor;

    // Timer to update objects positions
    QTimer *Update_timer;

    bool option_draw_debug[10];
    bool option_draw_obstacles[10];

Q_SIGNALS:
public Q_SLOTS:
    void flip(void);
    void lock(bool);
    void showPathPoints(void);
    void showVoronoiNet(void);
    void showCorridorCheck(void);
    void showSites(void);
    void showAll(void);
    void update_robot_info(void);

    void obstacles_point_flip(unsigned int Robot_no, bool on_off);
    void obstacles_point_flip_r0 (bool on_off)
        {obstacles_point_flip (0, on_off);}
    void obstacles_point_flip_r1 (bool on_off)
        {obstacles_point_flip (1, on_off);}
    void obstacles_point_flip_r2 (bool on_off)
        {obstacles_point_flip (2, on_off);}
    void obstacles_point_flip_r3 (bool on_off)
        {obstacles_point_flip (3, on_off);}
    void obstacles_point_flip_r4 (bool on_off)
        {obstacles_point_flip (4, on_off);}
    void obstacles_point_flip_r5 (bool on_off)
        {obstacles_point_flip (5, on_off);}
    void obstacles_point_flip_r6 (bool on_off)
        {obstacles_point_flip (6, on_off);}
    void obstacles_point_flip_all (bool on_off);

    //Debug Points
    void debug_point_flip (unsigned int Robot_no, bool on_off);
    void debug_point_flip_r0 (bool on_off)
        {debug_point_flip (0, on_off);}
    void debug_point_flip_r1 (bool on_off)
        {debug_point_flip (1, on_off);}
    void debug_point_flip_r2 (bool on_off)
        {debug_point_flip (2, on_off);}
    void debug_point_flip_r3 (bool on_off)
        {debug_point_flip (3, on_off);}
    void debug_point_flip_r4 (bool on_off)
        {debug_point_flip (4, on_off);}
    void debug_point_flip_r5 (bool on_off)
        {debug_point_flip (5, on_off);}
    void debug_point_flip_r6 (bool on_off)
        {debug_point_flip (6, on_off);}
    void debug_point_flip_all (bool on_off);

    // Heightmap
    void setHeightMapVisible(bool v){
        heightVisible = v;
    }
    void setHeightMap3D(bool v){
        height3D = v;
    }
    void setHeightMapColor(bool v){
        heightColor = v;
    }

    
};

#endif // FIELDWIDGET3D_H
