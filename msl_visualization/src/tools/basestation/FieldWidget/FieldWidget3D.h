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
#include <vtkLineWidget.h>
#include <vtkFollower.h>
#include <vtkDiskSource.h>
#include <vtkArcSource.h>

#include <QVTKInteractor.h>

#include "RobotInfo.h"
#include <msl_msgs/PathPlanner.h>
#include <msl_msgs/CorridorCheck.h>
#include <msl_msgs/VoronoiNetInfo.h>
#include <msl_helper_msgs/PassMsg.h>
#include <SystemConfig.h>
#include <vtkArrowSource.h>

#define OBSTACLE_HEIGHT 0.2

class MWind;

struct Line {
        Line(vtkSmartPointer<vtkActor> actor, vtkSmartPointer<vtkLineSource> source)
        {
          this->actor = actor;
          this->source = source;
        }

       vtkSmartPointer<vtkActor> actor;
       vtkSmartPointer<vtkLineSource> source;
};

class FieldWidget3D : public QVTKWidget
{
    Q_OBJECT
public:
    static vtkSmartPointer<vtkActor> createLine(float x1, float y1, float z1, float x2, float y2, float z2, float width, std::array<double,3> color = {1.0,1.0,1.0});
    static void updateLine(vtkSmartPointer<vtkActor> actor, float x1, float y1, float z1, float x2, float y2, float z2);
    static std::shared_ptr<Line> createDashedLine(float x1, float y1, float z1, float x2, float y2, float z2, float width, int pattern, std::array<double,3> color = {1.0,1.0,1.0});
    static vtkSmartPointer<vtkActor> createDot(float x, float y, float radius, std::array<double,3> color = {1.0,1.0,1.0});
    static vtkSmartPointer<vtkActor> addArc(float x, float y, float radius, float startDeg, float endDeg);
    static vtkSmartPointer<vtkActor> addCircle(float x, float y, float outerRadius, float innerRadius);
    static vtkSmartPointer<vtkActor> createText(QString text);

public:
    explicit FieldWidget3D(QWidget *parent = 0);
    pair<double, double> transformToGuiCoords(double x, double y);

    MWind* mainWindow;
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
    list<shared_ptr<RobotInfo>>* getRobots();

    // field configuration
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
    string currentField;

    // mutex
    mutex swmMutex;
    mutex pathMutex;
    mutex voronoiMutex;
    mutex corridorMutex;
    mutex debugMutex;
    mutex passMutex;


private:
    void onSharedWorldInfo(boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> info);
    void onPathPlannerMsg(boost::shared_ptr<msl_msgs::PathPlanner> info);
    void onVoronoiNetMsg(boost::shared_ptr<msl_msgs::VoronoiNetInfo> info);
    void onCorridorCheckMsg(boost::shared_ptr<msl_msgs::CorridorCheck> info);
    void onDebugMsg(boost::shared_ptr<msl_helper_msgs::DebugMsg> info);
    void onPassMsg(boost::shared_ptr<msl_helper_msgs::PassMsg> info);

    void drawField(vtkRenderer* renderer);
    void drawFieldLine(vtkRenderer* renderer, float x1, float y1, float z1, float x2, float y2, float z2);
    void drawGoals(vtkRenderer* renderer);
    void initGridView();
    void updateGridView();
    void deleteGridView();
    std::shared_ptr<RobotInfo> getRobotById(int id);

    vtkSmartPointer<vtkActor> createObstacle();
    vtkSmartPointer<vtkActor> createDebugPt();

private:
    // ros stuff
    ros::NodeHandle* rosNode;
    ros::AsyncSpinner* spinner;
    ros::Subscriber sharedWorldInfoSubscriber;
    ros::Subscriber pathPlannerSubscriber;
    ros::Subscriber voronoiSidesSubscriber;
    ros::Subscriber corridorCheckSubscriber;
    ros::Subscriber debugMsgSubscriber;
    ros::Subscriber passMsgSubscriber;

    // own data structure
    list<shared_ptr<RobotInfo>> robots;

    // debug stuff
    bool showDebugPoints;

    // path planner stuff
    // bool showPath;
    // bool showVoronoiNet;
    // bool showCorridorCheck;
    // bool showSitePoints;
    // bool showPathPlannerAll;

    // ui stuff
    QWidget* parent;

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
    void update_robot_info(void);

    // Debug
    void showDebugPointsToggle(void);

    // Path Planner
    //void showPathToggle(void);
    //void showVoronoiNetToggle(void);
    //void showCorridorCheckToggle(void);
    //void showSidePointsToggle(void);
    //void showPathPlannerAllToggle(void);
    //void updatePathPlannerAll(void);


//    void obstacles_point_flip(unsigned int Robot_no, bool on_off);
//    void obstacles_point_flip_r0 (bool on_off)
//        {obstacles_point_flip (0, on_off);}
//    void obstacles_point_flip_r1 (bool on_off)
//        {obstacles_point_flip (1, on_off);}
//    void obstacles_point_flip_r2 (bool on_off)
//        {obstacles_point_flip (2, on_off);}
//    void obstacles_point_flip_r3 (bool on_off)
//        {obstacles_point_flip (3, on_off);}
//    void obstacles_point_flip_r4 (bool on_off)
//        {obstacles_point_flip (4, on_off);}
//    void obstacles_point_flip_r5 (bool on_off)
//        {obstacles_point_flip (5, on_off);}
//    void obstacles_point_flip_r6 (bool on_off)
//        {obstacles_point_flip (6, on_off);}
//    void obstacles_point_flip_all (bool on_off);

    //Debug Points
//    void debug_point_flip (unsigned int Robot_no, bool on_off);
//    void debug_point_flip_r0 (bool on_off)
//        {debug_point_flip (0, on_off);}
//    void debug_point_flip_r1 (bool on_off)
//        {debug_point_flip (1, on_off);}
//    void debug_point_flip_r2 (bool on_off)
//        {debug_point_flip (2, on_off);}
//    void debug_point_flip_r3 (bool on_off)
//        {debug_point_flip (3, on_off);}
//    void debug_point_flip_r4 (bool on_off)
//        {debug_point_flip (4, on_off);}
//    void debug_point_flip_r5 (bool on_off)
//        {debug_point_flip (5, on_off);}
//    void debug_point_flip_r6 (bool on_off)
//        {debug_point_flip (6, on_off);}
//    void debug_point_flip_all (bool on_off);

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
