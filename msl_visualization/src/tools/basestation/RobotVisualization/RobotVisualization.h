#pragma once

#include <memory>
#include <string>
#include <map>
#include <vtkActor.h>
#include <vtkLineSource.h>
#include <vtkSmartPointer.h>

class RobotInfo;
class FieldWidget3D;
class Line;

class RobotVisualization
{
  public:
    RobotVisualization(RobotInfo *robot, FieldWidget3D *field);
    virtual ~RobotVisualization();
    vtkSmartPointer<vtkActor> getRobotBox();
    void setRobotBox(vtkSmartPointer<vtkActor> robotBox);
    vtkSmartPointer<vtkActor> getRobotTopTriangle();
    void setRobotTopTriangle(vtkSmartPointer<vtkActor> robotTopTriangle);
    void setNameActor(vtkSmartPointer<vtkActor> nameActor);
    vtkSmartPointer<vtkActor> getNameActor();
    int getId();
    void setId(int id);
    std::string getName();
    void setName(std::string name);
    bool getBallOnly();
    void setBallOnly(bool ballOnly);
    int getSenderId();
    void setSenderId(int senderId);
    vtkActor *getBall();
    void setBall(vtkActor *ball);
    vtkSmartPointer<vtkLineSource> getBallVelocity();
    void setBallVelocity(vtkSmartPointer<vtkLineSource> ballVelocity);
    vtkSmartPointer<vtkActor> getBallVelocityActor();
    void setBallVelocityActor(vtkSmartPointer<vtkActor> ballVelocityActor);
    vtkSmartPointer<vtkActor> getSharedBall();
    void setSharedBall(vtkSmartPointer<vtkActor> sharedBall);

    void remove(vtkRenderer *renderer);
    void init(vtkRenderer *renderer, int id);
    void updatePosition(vtkRenderer *renderer);
    void updateBall(vtkRenderer *renderer);
    void updateSharedBall(vtkRenderer *renderer);
    void updateObstacles(vtkRenderer *renderer, bool show);
    void updateMergedOpponents(vtkRenderer *renderer);
    void updateMergedOpponentsVis(vtkRenderer *renderer, bool show);

    void updatePathPlannerDebug(vtkRenderer *renderer, bool show);
    void updateCorridorDebug(vtkRenderer *renderer, bool show);
    void updateVoronoiNetDebug(vtkRenderer *renderer, bool showVoronoi, bool showSidePoints);
    void updateDebugPoints(vtkRenderer *renderer, bool showDebugPoints);
    void updatePassMsg(vtkRenderer *renderer, bool showPassing);

  private:
    void drawObstacleDisc(vtkRenderer *renderer, double x, double y);
    void drawMergedOppBase(vtkRenderer *renderer, double x, double y);
    void drawMergedOppTop(vtkRenderer *renderer, double x, double y,std::vector<int> supporters);
    void updateMergedOppTop(std::vector<vtkSmartPointer<vtkActor>> top, double x, double y,std::vector<int> supporters);
    void addToPieceMap(int myId, std::vector<vtkSmartPointer<vtkActor>> piece);
    std::array<double, 3> &getColor();
    int getDashedPattern();

  private:
    RobotInfo *robot;
    FieldWidget3D *field;
    bool visible;

    std::string name = "";
    int id = 0;
    int senderId = 0;
    vtkSmartPointer<vtkActor> robotTopTriangle = nullptr;
    vtkSmartPointer<vtkActor> robotBox = nullptr;
    vtkSmartPointer<vtkActor> nameActor = nullptr;
    vtkSmartPointer<vtkActor> ball = nullptr;
    vtkSmartPointer<vtkLineSource> ballVelocity = nullptr;
    vtkSmartPointer<vtkActor> ballVelocityActor = nullptr;
    vtkSmartPointer<vtkActor> sharedBall = nullptr;
    vtkSmartPointer<vtkLineSource> pass = nullptr;
    vtkSmartPointer<vtkActor> passActor = nullptr;
    vtkSmartPointer<vtkActor> passPointActor = nullptr;
    std::vector<vtkSmartPointer<vtkActor>> obstacleDiscs;
    static std::vector<vtkSmartPointer<vtkActor>> mergedOppsBases;
    static std::vector<std::vector<vtkSmartPointer<vtkActor>>> mergedOppsTops;
    static std::map<int, bool> showMergedOppTopMap;
    std::vector<vtkSmartPointer<vtkActor>> robotPieces;


    std::vector<vtkSmartPointer<vtkActor>> pathLines;
    std::vector<std::shared_ptr<Line>> netLines;
    std::shared_ptr<Line> corridorLine1;
    std::shared_ptr<Line> corridorLine2;
    std::shared_ptr<Line> corridorLine3;
    std::shared_ptr<Line> corridorLine4;
    std::vector<vtkSmartPointer<vtkActor>> sidePoints;
    std::vector<vtkSmartPointer<vtkActor>> debugPoints;
};
