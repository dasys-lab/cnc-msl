/*
 * RobotVisualization.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: Stefan Jakob
 */

#include <tools/basestation/RobotVisualization/RobotVisualization.h>

#include <vtkActor.h>
#include <vtkArcSource.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkDataSetMapper.h>
#include <vtkDataSetMapper.h>
#include <vtkDelaunay2D.h>
#include <vtkDiskSource.h>
#include <vtkFloatArray.h>
#include <vtkFollower.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageData.h>
#include <vtkImageMapper.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkLineSource.h>
#include <vtkLineWidget.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkOBJReader.h>
#include <vtkObjectFactory.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPointWidget.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkPropCollection.h>
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkPyramid.h>
#include <vtkRegularPolygonSource.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTetra.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTextSource.h>
#include <vtkTextWidget.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTriangleStrip.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVectorText.h>
#include <vtkViewport.h>

#include "FieldWidget3D.h"
#include "RobotInfo.h"

struct Color
{
    static std::map<std::string, std::array<double, 3>> create_map()
    {
        std::map<std::string, std::array<double, 3>> m;
        m["default"] = {0.0, 0.0, 0.0};
        m["mops"] = {1.0, 0.0, 0.0};
        m["hairy"] = {1.0, 1.0, 0.0};
        m["nase"] = {0.0, 1.0, 0.0};
        m["savvy"] = {0.0, 1.0, 1.0};
        m["myo"] = {0.0, 0.0, 1.0};
        m["brain"] = {1.0, 0.0, 1.0};
        m["path"] = {1.0, 1.0, 1.0};
        return m;
    }
    static std::map<std::string, std::array<double, 3>> map;

    static std::array<double, 3> &getColor(int id)
    {
        if (id == 1)
            return map["mops"];
        else if (id == 8)
            return map["hairy"];
        else if (id == 9)
            return map["nase"];
        else if (id == 10)
            return map["savvy"];
        else if (id == 11)
            return map["myo"];
        else if (id == 100)
            return map["brain"];

        return map["default"];
    }
};

std::map<std::string, std::array<double, 3>> Color::map = Color::create_map();
float robotPos[101][2];
int robotIds[6] = {1, 8, 9, 10, 11, 100};

RobotVisualization::RobotVisualization(RobotInfo *robot, FieldWidget3D *field)
    : robot(robot)
    , field(field)
{
    id = 0;
    senderId = 0;
    visible = false;
}

RobotVisualization::~RobotVisualization()
{
    // nothing to do here
    // test
}

vtkSmartPointer<vtkActor> RobotVisualization::getRobotBox()
{
    return robotBox;
}

void RobotVisualization::setRobotBox(vtkSmartPointer<vtkActor> robotBox)
{
    this->robotBox = robotBox;
}

void RobotVisualization::setNameActor(vtkSmartPointer<vtkActor> nameActor)
{
    this->nameActor = nameActor;
}

vtkSmartPointer<vtkActor> RobotVisualization::getNameActor()
{
    return this->nameActor;
}

vtkSmartPointer<vtkActor> RobotVisualization::getRobotTopTriangle()
{
    return robotTopTriangle;
}

void RobotVisualization::setRobotTopTriangle(vtkSmartPointer<vtkActor> robotTopTriangle)
{
    this->robotTopTriangle = robotTopTriangle;
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

vtkActor *RobotVisualization::getBall()
{
    return ball;
}

void RobotVisualization::setBall(vtkActor *ball)
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

void RobotVisualization::remove(vtkRenderer *renderer)
{
    this->visible = false;

    robotPos[this->id][0] = -100000;
    robotPos[this->id][1] = -100000;

    this->robotTopTriangle->SetVisibility(false);
    this->robotBox->SetVisibility(false);
    this->nameActor->SetVisibility(false);
    this->ball->SetVisibility(false);
    this->ballVelocityActor->SetVisibility(false);
    this->sharedBall->SetVisibility(false);

    //    for (vtkSmartPointer<vtkActor> actor : objectsTop)
    //    {
    //        actor->SetVisibility(false);
    //    }
    //
    //    for (vtkSmartPointer<vtkActor> actor : objectsBox)
    //    {
    //        actor->SetVisibility(false);
    //    }

    for (vtkSmartPointer<vtkActor> actor : obstacleDiscs)
    {
        actor->SetVisibility(false);
    }

    for (vtkSmartPointer<vtkActor> actor : pathLines)
    {
        if (actor != nullptr)
        {
            renderer->RemoveActor(actor);
        }
    }
    pathLines.clear();

    // path planner corridor check
    this->corridorLine1->actor->SetVisibility(false);
    this->corridorLine2->actor->SetVisibility(false);
    this->corridorLine3->actor->SetVisibility(false);
    this->corridorLine4->actor->SetVisibility(false);

    // voronoi net
    for (int i = 0; i < this->netLines.size(); ++i)
    {
        this->netLines.at(i)->actor->SetVisibility(false);
    }

    for (vtkSmartPointer<vtkActor> actor : sidePoints)
    {
        if (actor != nullptr)
        {
            renderer->RemoveActor(actor);
        }
    }
    sidePoints.clear();

    // debug points
    for (vtkSmartPointer<vtkActor> actor : this->debugPoints)
    {
        if (actor != nullptr)
        {
            renderer->RemoveActor(actor);
        }
    }
    debugPoints.clear();

    // pass msg
    this->passActor->SetVisibility(false);
    this->passPointActor->SetVisibility(false);
}

void RobotVisualization::init(vtkRenderer *renderer, int id)
{
    auto color = Color::getColor(this->robot->getId());

    this->setId(id);
    this->setName(std::to_string(id));
    this->setBall(nullptr);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    float p0[3] = {0.26, 0, 0};
    float p1[3] = {-0.26, 0.26, 0};
    float p2[3] = {-0.26, -0.26, 0};
    float p3[3] = {0.0, 0.0, 0.4};

    points->InsertNextPoint(p0);
    points->InsertNextPoint(p1);
    points->InsertNextPoint(p2);
    points->InsertNextPoint(p3);

    vtkSmartPointer<vtkTetra> tetra = vtkSmartPointer<vtkTetra>::New();
    tetra->GetPointIds()->SetId(0, 0);
    tetra->GetPointIds()->SetId(1, 1);
    tetra->GetPointIds()->SetId(2, 2);
    tetra->GetPointIds()->SetId(3, 3);

    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(tetra);

    vtkSmartPointer<vtkUnstructuredGrid> ug = vtkSmartPointer<vtkUnstructuredGrid>::New();
    ug->SetPoints(points);
    ug->InsertNextCell(tetra->GetCellType(), tetra->GetPointIds());

    vtkSmartPointer<vtkDataSetMapper> teamTopMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    teamTopMapper->SetInputData(ug);

    vtkSmartPointer<vtkCubeSource> cubeSrc = vtkSmartPointer<vtkCubeSource>::New();
    cubeSrc->SetXLength(0.52);
    cubeSrc->SetYLength(0.52);
    cubeSrc->SetZLength(0.4);

    vtkSmartPointer<vtkPolyDataMapper> teamBoxMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    teamBoxMapper->SetInputConnection(cubeSrc->GetOutputPort());

    vtkSmartPointer<vtkActor> teamBox = vtkSmartPointer<vtkActor>::New();
    teamBox->SetMapper(teamBoxMapper);
    teamBox->SetPosition(0, 0, 0.2);

    float rColor = 1.0, gColor = 1.0, bColor = 1.0;

    // unfinished business: inactive teammate has slightly different box color
    /*
    if (!this->robot->getVisStatus())
    {
            rColor = 0.7;
            gColor = 0.7;
            bColor = 1.0;
    }
    */

    teamBox->GetProperty()->SetColor(rColor, gColor, bColor);
    teamBox->GetProperty()->SetDiffuse(0.4);
    teamBox->GetProperty()->SetAmbient(0.8);
    renderer->AddActor(teamBox);

    vtkSmartPointer<vtkActor> teamTop = vtkSmartPointer<vtkActor>::New();
    teamTop->SetMapper(teamTopMapper);
    teamTop->SetPosition(0, 0, 0.4);
    auto c = Color::getColor(this->robot->getId());
    teamTop->GetProperty()->SetColor(c[0], c[1], c[2]);
    teamTop->GetProperty()->SetDiffuse(0.4);
    teamTop->GetProperty()->SetAmbient(0.8);
    renderer->AddActor(teamTop);

    // Name Actor
    // Create text
    vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New();
    textSource->SetText((this->name).c_str());
    textSource->Update();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> nameMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    nameMapper->SetInputConnection(textSource->GetOutputPort());

    vtkSmartPointer<vtkFollower> nameActor = vtkSmartPointer<vtkFollower>::New();
    nameActor->SetMapper(nameMapper);
    nameActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    nameActor->GetProperty()->SetDiffuse(0.4);
    nameActor->GetProperty()->SetAmbient(0.8);
    nameActor->SetPosition(0, 0, 1);
    nameActor->SetScale(0.5);

    renderer->AddActor(nameActor);
    nameActor->SetCamera(renderer->GetActiveCamera());

    this->robotTopTriangle = teamTop;
    this->robotBox = teamBox;
    this->nameActor = nameActor;

    this->robotTopTriangle->SetVisibility(false);
    this->robotBox->SetVisibility(false);
    this->nameActor->SetVisibility(false);

    // ball
    vtkSmartPointer<vtkSphereSource> sphereSrc = vtkSmartPointer<vtkSphereSource>::New();
    sphereSrc->SetRadius(field->_BALL_DIAMETER / 2);
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSrc->GetOutputPort());
    this->ball = vtkSmartPointer<vtkActor>::New();
    this->ball->SetMapper(sphereMapper);
    this->ball->GetProperty()->SetRepresentationToSurface();
    this->ball->GetProperty()->SetColor(255, 0, 0);
    this->ball->SetPosition(0, 0, this->field->_BALL_DIAMETER / 2);
    renderer->AddActor(this->ball);

    this->ballVelocity = vtkSmartPointer<vtkLineSource>::New();
    this->ballVelocity->SetPoint1(0, 0, 0);
    this->ballVelocity->SetPoint2(0, 0, 0);

    vtkSmartPointer<vtkPolyDataMapper> velocityMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    velocityMapper->SetInputConnection(this->ballVelocity->GetOutputPort());
    this->ballVelocityActor = vtkSmartPointer<vtkActor>::New();
    this->ballVelocityActor->SetMapper(velocityMapper);
    this->ballVelocityActor->GetProperty()->SetLineWidth(this->field->_LINE_THICKNESS / 2);
    this->ballVelocityActor->GetProperty()->SetColor(1, 0, 0);
    this->ballVelocityActor->GetProperty()->SetDiffuse(0.4);
    this->ballVelocityActor->GetProperty()->SetAmbient(0.8);

    renderer->AddActor(this->ballVelocityActor);
    this->ball->SetVisibility(false);
    this->ballVelocityActor->SetVisibility(false);

    // shared ball
    vtkSmartPointer<vtkRegularPolygonSource> polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
    polygonSource->SetNumberOfSides(50);
    polygonSource->SetRadius((this->field->_BALL_DIAMETER / 2) * 1.9);
    polygonSource->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(polygonSource->GetOutputPort());
    this->sharedBall = vtkSmartPointer<vtkActor>::New();
    this->sharedBall->SetMapper(mapper);
    this->sharedBall->GetProperty()->SetColor(0, 0, 0.5);
    this->sharedBall->GetProperty()->SetDiffuse(0.4);
    this->sharedBall->GetProperty()->SetAmbient(0.8);

    renderer->AddActor(this->sharedBall);
    this->sharedBall->SetVisibility(false);

    // corridor
    this->corridorLine1 = FieldWidget3D::createDashedLine(0, 0, 0.01, 0, 0, 0.01, 3, this->getDashedPattern(), color);
    this->corridorLine2 = FieldWidget3D::createDashedLine(0, 0, 0.01, 0, 0, 0.01, 3, this->getDashedPattern(), color);
    this->corridorLine3 = FieldWidget3D::createDashedLine(0, 0, 0.01, 0, 0, 0.01, 3, this->getDashedPattern(), color);
    this->corridorLine4 = FieldWidget3D::createDashedLine(0, 0, 0.01, 0, 0, 0.01, 3, this->getDashedPattern(), color);

    this->corridorLine1->actor->SetVisibility(false);
    this->corridorLine2->actor->SetVisibility(false);
    this->corridorLine3->actor->SetVisibility(false);
    this->corridorLine4->actor->SetVisibility(false);

    renderer->AddActor(this->corridorLine1->actor);
    renderer->AddActor(this->corridorLine2->actor);
    renderer->AddActor(this->corridorLine3->actor);
    renderer->AddActor(this->corridorLine4->actor);

    // pass msg
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
    line->SetPoint1(0, 0, 0);
    line->SetPoint2(0, 0, 0);
    lineMapper->SetInputConnection(line->GetOutputPort());
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetLineWidth(3);
    lineActor->GetProperty()->SetColor(c[0], c[1], c[2]);
    lineActor->GetProperty()->SetPointSize(1);

    this->pass = line;
    this->passActor = lineActor;
    this->passActor->SetVisibility(false);
    this->passPointActor = FieldWidget3D::createDot(0, 0, 0.22, c);
    this->passPointActor->SetVisibility(false);

    renderer->AddActor(this->passActor);
    renderer->AddActor(this->passPointActor);
}

void RobotVisualization::updatePosition(vtkRenderer *renderer)
{
    auto pos = this->field->transformToGuiCoords(robot->getSharedWorldInfo()->odom.position.x,
                                                 robot->getSharedWorldInfo()->odom.position.y);

    robotPos[this->id][0] = pos.first;
    robotPos[this->id][1] = pos.second;

    this->robotTopTriangle->SetPosition(pos.first, pos.second, 0.4);
    this->robotBox->SetPosition(pos.first, pos.second, 0.2);
    this->nameActor->SetPosition(pos.first, pos.second, 1);

    this->robotTopTriangle->SetOrientation(0, 0, robot->getSharedWorldInfo()->odom.position.angle * (180.0 / (double)M_PI) + 90);
    this->robotBox->SetOrientation(0, 0, robot->getSharedWorldInfo()->odom.position.angle * (180.0 / (double)M_PI) + 90);

    if (this->robot->getVisStatus())
        this->robotTopTriangle->SetVisibility(true);
    else
        this->robotTopTriangle->SetVisibility(false);
    this->robotBox->SetVisibility(true);
    this->nameActor->SetVisibility(true);
}

void RobotVisualization::updateBall(vtkRenderer *renderer)
{
    if (!robot->getVisStatus() || robot->getSharedWorldInfo()->ball.confidence <= 0.0001)
    {
        this->ball->SetVisibility(false);
        this->ballVelocityActor->SetVisibility(false);
        return;
    }

    auto pos = this->field->transformToGuiCoords(robot->getSharedWorldInfo()->ball.point.x,
                                                 robot->getSharedWorldInfo()->ball.point.y);

    this->ball->SetPosition(pos.first, pos.second, robot->getSharedWorldInfo()->ball.point.z / 1000 + 0.02);
    this->ballVelocity->SetPoint1(pos.first, pos.second, robot->getSharedWorldInfo()->ball.point.z / 1000);
    auto ballVelTrans = this->field->transformToGuiCoords(robot->getSharedWorldInfo()->ball.velocity.vx,
                                                          robot->getSharedWorldInfo()->ball.velocity.vy);
    this->ballVelocity->SetPoint2(pos.first + ballVelTrans.first, pos.second + ballVelTrans.second,
                                  robot->getSharedWorldInfo()->ball.point.z / 1000 +
                                      robot->getSharedWorldInfo()->ball.velocity.vz / 1000);

    this->ball->SetVisibility(true);
    this->ballVelocityActor->SetVisibility(true);
}

void RobotVisualization::updateSharedBall(vtkRenderer *renderer)
{
    if (!robot->getVisStatus() || robot->getSharedWorldInfo()->sharedBall.confidence <= 0.0001)
    {
        this->sharedBall->SetVisibility(false);
        return;
    }

    auto pos = this->field->transformToGuiCoords(robot->getSharedWorldInfo()->sharedBall.point.x,
                                                 robot->getSharedWorldInfo()->sharedBall.point.y);
    this->sharedBall->SetPosition(pos.first, pos.second, robot->getSharedWorldInfo()->sharedBall.point.z / 1000 + 0.02);
    this->sharedBall->SetVisibility(true);
}

void RobotVisualization::updateObstacles(vtkRenderer *renderer)
{
    bool found = false;
    int objectCount = 0;

    if (!robot->getBallOnly())
    {
        for (auto myObject : robot->getSharedWorldInfo()->obstacles)
        {
            found = false;
            auto pos = this->field->transformToGuiCoords(myObject.x, myObject.y);

            // don't draw the obstacle if there is an object already drawn on the field
            for (auto member : *this->field->getRobots())
            {
                auto mb = member->getVisualization()->getRobotBox();
                if (mb == nullptr)
                    continue;

                if (abs(mb->GetPosition()[0] - pos.first) < 0.25 && abs(mb->GetPosition()[1] - pos.second) < 0.25)
                {
                    found = true;
                    break;
                }
            }

            if (found)
                continue;

            // don't draw object boxes on teammates
            bool aTeammate = false;
            for (int i = 0; i < 6; i++)
            {
                if (abs(robotPos[robotIds[i]][0] - pos.first) < 0.4 && abs(robotPos[robotIds[i]][1] - pos.second) < .4)
                    aTeammate = true;
            }

            if (!aTeammate && this->robot->getVisStatus())
            {
                // move object boxes instead of drawing new ones, as long as possible
                if (objectCount < this->obstacleDiscs.size())
                {
                    this->obstacleDiscs.at(objectCount)->SetPosition(pos.first, pos.second, 0.2);
                    this->obstacleDiscs.at(objectCount)->SetVisibility(true);
                }
                else
                {
                    drawObstacleDisc(renderer, pos.first, pos.second);
                }
                objectCount++;
            }
        }
    }

    // hide unused object boxes
    if (objectCount < this->obstacleDiscs.size())
    {
        for (int i = objectCount; i < this->obstacleDiscs.size(); ++i)
        {
            this->obstacleDiscs.at(i)->SetVisibility(false);
        }
    }
}

void RobotVisualization::drawObstacleDisc(vtkRenderer *renderer, double x, double y)
{
    vtkSmartPointer<vtkDiskSource> diskSrc = vtkSmartPointer<vtkDiskSource>::New();
    diskSrc->SetOuterRadius(0.26); // like a robot
    diskSrc->SetInnerRadius(0.0);
    diskSrc->SetCircumferentialResolution(24);

    // Create a mapper and actor.
    vtkSmartPointer<vtkPolyDataMapper> obstacleDiscMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    obstacleDiscMapper->SetInputConnection(diskSrc->GetOutputPort());

    vtkSmartPointer<vtkActor> obstacleDisc = vtkSmartPointer<vtkActor>::New();
    obstacleDisc->SetMapper(obstacleDiscMapper);
    obstacleDisc->SetPosition(x, y, 0.01); // 0.01 ~ a little bit above the field

    auto c = Color::getColor(this->robot->getId());
    obstacleDisc->GetProperty()->SetColor(c[0], c[1], c[2]);

    // The transparency should make it possible to see different disc at one spot
    obstacleDisc->GetProperty()->SetOpacity(0.6);

    obstacleDisc->GetProperty()->SetDiffuse(0.4);
    obstacleDisc->GetProperty()->SetAmbient(0.8);
    renderer->AddActor(obstacleDisc);

    obstacleDiscs.push_back(obstacleDisc);
}

void RobotVisualization::updateMergedOpponents(vtkRenderer *renderer)
{
    bool found = false;
    int objectCount = 0;

    if (!robot->getBallOnly())
    {
        for (auto mergedOpp : robot->getSharedWorldInfo()->mergedOpponents)
        {
            found = false;
            auto pos = this->field->transformToGuiCoords(mergedOpp.x, mergedOpp.y);

            // we should draw merged Opponents onto teammates, because this indicates a failure!

//            // don't draw the obstacle if there is a teammate already drawn on the field
//            for (auto member : *this->field->getRobots())
//            {
//                auto mb = member->getVisualization()->getRobotBox();
//                if (mb == nullptr)
//                    continue;
//
//                if (abs(mb->GetPosition()[0] - pos.first) < 0.25 && abs(mb->GetPosition()[1] - pos.second) < 0.25)
//                {
//                    found = true;
//                    break;
//                }
//            }
//
//            if (found)
//                continue;

//            // don't draw object boxes on teammates
//            bool aTeammate = false;
//            for (int i = 0; i < 6; i++)
//            {
//                if (abs(robotPos[robotIds[i]][0] - pos.first) < 0.4 && abs(robotPos[robotIds[i]][1] - pos.second) < .4)
//                    aTeammate = true;
//            }

            if (this->robot->getVisStatus())
            {
                // move object boxes instead of drawing new ones, as long as possible
                if (objectCount < this->mergedOppsBases.size())
                {
                    this->mergedOppsBases.at(objectCount)->SetPosition(pos.first, pos.second, 0.2);
                    this->mergedOppsBases.at(objectCount)->SetVisibility(true);
                    this->mergedOppsTops.at(objectCount)->SetPosition(pos.first, pos.second, 0.4);
                    this->mergedOppsTops.at(objectCount)->SetVisibility(true);
                }
                else
                {
                    drawMergedOppBase(renderer, pos.first, pos.second);
                    drawMergedOppTop(renderer, pos.first, pos.second);
                }
                objectCount++;
            }
        }
    }

    // hide unused object boxes
    if (objectCount < this->mergedOppsBases.size())
    {
        for (int i = objectCount; i < this->mergedOppsBases.size(); ++i)
        {
            this->mergedOppsBases.at(i)->SetVisibility(false);
            this->mergedOppsTops.at(i)->SetVisibility(false);
        }
    }
}

void RobotVisualization::drawMergedOppBase(vtkRenderer *renderer, double x, double y)
{
    vtkSmartPointer<vtkCylinderSource> cylinderSrc = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSrc->SetCenter(0, 0, 0);
    cylinderSrc->SetRadius(0.26); // like a robot
    cylinderSrc->SetHeight(0.7); // almost the height of a robot, because the coloured top has a 0.1m height
    cylinderSrc->SetResolution(100);

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> oppBaseMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    oppBaseMapper->SetInputConnection(cylinderSrc->GetOutputPort());

    vtkSmartPointer<vtkActor> oppBase = vtkSmartPointer<vtkActor>::New();
    oppBase->SetMapper(oppBaseMapper);
    oppBase->SetPosition(x, y, 0.01); // 0.01 ~ a little bit above the field

    oppBase->GetProperty()->SetColor(0.8, 0.8, 0.8);

    oppBase->GetProperty()->SetDiffuse(0.4);
    oppBase->GetProperty()->SetAmbient(0.8);
    renderer->AddActor(oppBase);

    this->mergedOppsBases.push_back(oppBase);
}

void RobotVisualization::drawMergedOppTop(vtkRenderer *renderer, double x, double y)
{
  // TODO
}

void RobotVisualization::updatePathPlannerDebug(vtkRenderer *renderer, bool show)
{
    // Check last message
    boost::shared_ptr<msl_msgs::PathPlanner> ppi;
    bool timeout = false;
    {
        lock_guard<mutex> lock(this->field->pathMutex);
        ppi = this->robot->getPathPlannerInfo();
        timeout = this->robot->isPathPlannerMsgTimeout();

        if (false == ppi)
            return;
    }

    // Remove old stuff
    for (vtkSmartPointer<vtkActor> actor : pathLines)
    {
        if (actor != nullptr)
        {
            renderer->RemoveActor(actor);
        }
    }
    pathLines.clear();

    if (false == show || timeout)
        return;

    // Draw new
    for (int i = 1; i < ppi->pathPoints.size(); i++)
    {
        pair<double, double> point1 =
            this->field->transformToGuiCoords(ppi->pathPoints.at(i - 1).x, ppi->pathPoints.at(i - 1).y);
        pair<double, double> point2 =
            this->field->transformToGuiCoords(ppi->pathPoints.at(i).x, ppi->pathPoints.at(i).y);

        vtkSmartPointer<vtkActor> actor =
            FieldWidget3D::createLine(point1.first, point1.second, 0.01, point2.first, point2.second, 0.01, 3,
                                      Color::getColor(this->robot->getId()));
        pathLines.push_back(actor);
        renderer->AddActor(actor);
    }
}

void RobotVisualization::updateCorridorDebug(vtkRenderer *renderer, bool show)
{
    // Check last message
    boost::shared_ptr<msl_msgs::CorridorCheck> cc;
    bool timeout = false;
    {
        lock_guard<mutex> lock(this->field->corridorMutex);
        cc = this->robot->getCorridorCheckInfo();
        timeout = this->robot->isCorridorCheckMsgTimeout();

        if (false == cc)
            return;
    }

    if (false == show || timeout)
    {
        this->corridorLine1->actor->SetVisibility(false);
        this->corridorLine2->actor->SetVisibility(false);
        this->corridorLine3->actor->SetVisibility(false);
        this->corridorLine4->actor->SetVisibility(false);
        return;
    }

    pair<double, double> point0 =
        this->field->transformToGuiCoords(cc->corridorPoints.at(0).x, cc->corridorPoints.at(0).y);
    pair<double, double> point1 =
        this->field->transformToGuiCoords(cc->corridorPoints.at(1).x, cc->corridorPoints.at(1).y);
    pair<double, double> point2 =
        this->field->transformToGuiCoords(cc->corridorPoints.at(2).x, cc->corridorPoints.at(2).y);
    pair<double, double> point3 =
        this->field->transformToGuiCoords(cc->corridorPoints.at(3).x, cc->corridorPoints.at(3).y);

    // Draw new
    this->corridorLine1->source->SetPoint1(point0.first, point0.second, 0.01);
    this->corridorLine1->source->SetPoint2(point1.first, point1.second, 0.01);

    this->corridorLine2->source->SetPoint1(point1.first, point1.second, 0.01);
    this->corridorLine2->source->SetPoint2(point2.first, point2.second, 0.01);

    this->corridorLine3->source->SetPoint1(point2.first, point2.second, 0.01);
    this->corridorLine3->source->SetPoint2(point3.first, point3.second, 0.01);

    this->corridorLine4->source->SetPoint1(point3.first, point3.second, 0.01);
    this->corridorLine4->source->SetPoint2(point0.first, point0.second, 0.01);

    this->corridorLine1->actor->SetVisibility(true);
    this->corridorLine2->actor->SetVisibility(true);
    this->corridorLine3->actor->SetVisibility(true);
    this->corridorLine4->actor->SetVisibility(true);
}

void RobotVisualization::updateVoronoiNetDebug(vtkRenderer *renderer, bool showVoronoi, bool showSidePoints)
{
    // Check last message
    boost::shared_ptr<msl_msgs::VoronoiNetInfo> vni;
    bool timeout = false;
    {
        lock_guard<mutex> lock(this->field->voronoiMutex);
        vni = this->robot->getVoronoiNetInfo();
        timeout = this->robot->isVoronoiNetMsgTimeout();

        if (false == vni)
            return;
    }

    // Remove old stuff
    for (vtkSmartPointer<vtkActor> actor : sidePoints)
    {
        if (actor != nullptr)
        {
            renderer->RemoveActor(actor);
        }
    }
    sidePoints.clear();

    if (false == showVoronoi || timeout)
    {
        for (int i = 0; i < this->netLines.size(); ++i)
        {
            this->netLines.at(i)->actor->SetVisibility(false);
        }
    }

    if ((false == showVoronoi && false == showSidePoints) || timeout)
        return;

    int used = 0;
    vtkSmartPointer<vtkActor> actor;
    auto color = Color::getColor(this->robot->getId());

    // Draw voronoi net
    if (showVoronoi)
    {
        for (int i = 1; i < vni->linePoints.size(); i += 2)
        {
            pair<double, double> point1 =
                this->field->transformToGuiCoords(vni->linePoints.at(i - 1).x, vni->linePoints.at(i - 1).y);
            pair<double, double> point2 =
                this->field->transformToGuiCoords(vni->linePoints.at(i).x, vni->linePoints.at(i).y);

            if (used >= this->netLines.size())
            {
                auto line = FieldWidget3D::createDashedLine(point1.first, point1.second, 0.01, point2.first,
                                                            point2.second, 0.01, 3, this->getDashedPattern(), color);

                renderer->AddActor(line->actor);
                this->netLines.push_back(line);
            }
            else
            {
                auto line = this->netLines.at(used);
                line->actor->SetVisibility(true);
                line->source->SetPoint1(point1.first, point1.second, 0.01);
                line->source->SetPoint2(point2.first, point2.second, 0.01);
            }
            ++used;
        }

        if (used < this->netLines.size())
        {
            for (int i = used; i < this->netLines.size(); ++i)
            {
                this->netLines.at(i)->actor->SetVisibility(false);
            }
        }
    }

    // Draw side points
    if (showSidePoints)
    {
        for (vtkSmartPointer<vtkActor> actor : sidePoints)
        {
            if (actor != nullptr)
            {
                renderer->RemoveActor(actor);
            }
        }
        sidePoints.clear();

        for (int i = 0; i < vni->sites.size(); i++)
        {
            pair<double, double> point = this->field->transformToGuiCoords(vni->sites.at(i).x, vni->sites.at(i).y);

            vtkSmartPointer<vtkActor> actor =
                FieldWidget3D::createDot(point.first, point.second, 0.3, Color::getColor(this->robot->getId()));
            sidePoints.push_back(actor);
            renderer->AddActor(actor);
        }
    }
}

void RobotVisualization::updateDebugPoints(vtkRenderer *renderer, bool showDebugPoints)
{
    // Check last message
    vector<boost::shared_ptr<msl_helper_msgs::DebugMsg>> msgs;
    int count;
    {
        lock_guard<mutex> lock(this->field->debugMutex);
        count = this->robot->getDebugMsgs(msgs);
    }

    // Remove old objects if show path is disabled
    for (vtkSmartPointer<vtkActor> actor : this->debugPoints)
    {
        if (actor != nullptr)
        {
            renderer->RemoveActor(actor);
        }
    }
    debugPoints.clear();

    // Return if nothing should be drawn
    if (false == showDebugPoints || count == 0)
    {
        return;
    }

    // Draw debug points
    for (auto debugMsg : msgs)
    {
        for (int i = 0; i < debugMsg->points.size(); i++)
        {
            auto pointDbg = debugMsg->points.at(i);
            std::array<double, 3> color = {pointDbg.red / 255.0, pointDbg.green / 255.0, pointDbg.blue / 255.0};
            pair<double, double> point = this->field->transformToGuiCoords(pointDbg.point.x, pointDbg.point.y);

            vtkSmartPointer<vtkActor> actor =
                FieldWidget3D::createDot(point.first, point.second, pointDbg.radius, color);
            debugPoints.push_back(actor);
            renderer->AddActor(actor);
        }
    }
}

void RobotVisualization::updatePassMsg(vtkRenderer *renderer, bool showPassing)
{
    // Check last message
    boost::shared_ptr<msl_helper_msgs::PassMsg> passMsg;
    bool timeout = false;
    {
        lock_guard<mutex> lock(this->field->passMutex);
        passMsg = this->robot->getPassMsg();
        timeout = this->robot->isPassMsgTimeout();
    }

    // Return if nothing should be drawn
    if (false == passMsg || timeout)
    {
        this->passActor->SetVisibility(false);
        this->passPointActor->SetVisibility(false);
        return;
    }

    // Draw debug points
    if (showPassing)
    {
        this->passActor->SetVisibility(true);
        this->passPointActor->SetVisibility(true);
        auto origin = this->field->transformToGuiCoords(passMsg->origin.x, passMsg->origin.y);
        auto dest = this->field->transformToGuiCoords(passMsg->destination.x, passMsg->destination.y);

        this->pass->SetPoint1(origin.first, origin.second, 0.01);
        this->pass->SetPoint2(dest.first, dest.second, 0.01);

        this->passPointActor->SetPosition(dest.first, dest.second, 0.01);
    }
}

int RobotVisualization::getDashedPattern()
{
    return 0x66 << this->robot->getId();
}
