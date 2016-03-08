/*
 * RobotVisualization.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: Stefan Jakob
 */

#include <tools/basestation/RobotVisualization/RobotVisualization.h>

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

#include "RobotInfo.h"
#include "FieldWidget3D.h"

struct Color{
    static std::map<std::string, std::array<double,3>> create_map()
        {
      std::map<std::string, std::array<double,3>> m;
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
    static std::map<std::string, std::array<double,3>> map;

    static std::array<double,3>& getColor(int id)
    {
      if (id == 1)
        return map["mops"];
      else if (id == 8)
        return map["hairy"];
      else if (id == 9)
        return map["nase"];
      else if( id == 10)
        return map["savvy"];
      else if( id == 11)
        return map["myo"];
      else if( id == 100)
        return map["brain"];

      return map["default"];
    }
};

std::map<std::string, std::array<double,3>> Color::map =  Color::create_map();

RobotVisualization::RobotVisualization(RobotInfo* robot, FieldWidget3D* field) : robot(robot), field(field)
{
	id = 0;
	senderId = 0;
	visible = false;
}

RobotVisualization::~RobotVisualization()
{
	// nothing to do here
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

void RobotVisualization::remove(vtkRenderer *renderer)
{
        this->visible = false;

        renderer->RemoveActor(this->top);
        renderer->RemoveActor(this->bottom);
        renderer->RemoveActor(this->nameActor);
        renderer->RemoveActor(this->ball);
        renderer->RemoveActor(this->ballVelocityActor);
        renderer->RemoveActor(this->sharedBall);

        this->ballVelocity = nullptr;

        for (vtkSmartPointer<vtkActor> actor : pathLines)
        {
                if (actor != nullptr)
                {
                        renderer->RemoveActor(actor);
                }
        }
        pathLines.clear();

        for (vtkSmartPointer<vtkActor> actor : corridorLines)
        {
                if (actor != nullptr)
                {
                        renderer->RemoveActor(actor);
                }
       }
       corridorLines.clear();for (vtkSmartPointer<vtkActor> actor : this->netLines)
       {
                if (actor != nullptr)
                {
                        renderer->RemoveActor(actor);
                }
        }
        netLines.clear();

        for (vtkSmartPointer<vtkActor> actor : sitePoints)
        {
                if (actor != nullptr)
                {
                        renderer->RemoveActor(actor);
                }
        }
}

void RobotVisualization::init(vtkRenderer *renderer)
{
        this->visible = true;

        this->setId(robot->getSharedWorldInfo()->senderID);
        this->setName(std::to_string(robot->getSharedWorldInfo()->senderID));
        this->setBall(nullptr);

        auto pos = FieldWidget3D::transform(robot->getSharedWorldInfo()->odom.position.x, robot->getSharedWorldInfo()->odom.position.y);
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

        int x = pos.first / 1000;
        int y = pos.second / 1000;
        int z = 0;

        float p0[3] = {0.26, 0, 0};
        float p1[3] = {-0.26, 0.26, 0};
        float p2[3] = {-0.26, -0.26, 0};
        float p3[3] = {0.0, 0.0, 0.4};

        points->InsertNextPoint(p0);
        points->InsertNextPoint(p1);
        points->InsertNextPoint(p2);
        points->InsertNextPoint(p3);

        vtkSmartPointer<vtkTetra> tetra = vtkSmartPointer<vtkTetra>::New();
//      vtkSmartPointer<vtkPyramid> pyramid = vtkSmartPointer<vtkPyramid>::New();
        tetra->GetPointIds()->SetId(0, 0);
        tetra->GetPointIds()->SetId(1, 1);
        tetra->GetPointIds()->SetId(2, 2);
        tetra->GetPointIds()->SetId(3, 3);
//      pyramid->GetPointIds()->SetId(4, 4);

        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
        cells->InsertNextCell(tetra);

        vtkSmartPointer<vtkUnstructuredGrid> ug = vtkSmartPointer<vtkUnstructuredGrid>::New();
        ug->SetPoints(points);
        ug->InsertNextCell(tetra->GetCellType(), tetra->GetPointIds());

        vtkSmartPointer<vtkDataSetMapper> teamTopMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        teamTopMapper->SetInput(ug);

        vtkSmartPointer<vtkCubeSource> cubeSrc = vtkSmartPointer<vtkCubeSource>::New();
        cubeSrc->SetXLength(0.52);
        cubeSrc->SetYLength(0.52);
        cubeSrc->SetZLength(0.4);

        vtkSmartPointer<vtkPolyDataMapper> teamBottomMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        teamBottomMapper->SetInputConnection(cubeSrc->GetOutputPort());

        vtkSmartPointer<vtkActor> teamBottom = vtkSmartPointer<vtkActor>::New();
        teamBottom->SetMapper(teamBottomMapper);
        teamBottom->SetPosition(x, y, z + 0.2);
        teamBottom->GetProperty()->SetColor(1, 1, 1);
        teamBottom->GetProperty()->SetDiffuse(0.4);
        teamBottom->GetProperty()->SetAmbient(0.8);
        renderer->AddActor(teamBottom);

        vtkSmartPointer<vtkActor> teamTop = vtkSmartPointer<vtkActor>::New();
        teamTop->SetMapper(teamTopMapper);
        teamTop->SetPosition(x, y, z + 0.4);
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
        nameActor->SetPosition(x, y, z + 1);
        nameActor->SetScale(0.5);

        renderer->AddActor(nameActor);
        nameActor->SetCamera( renderer->GetActiveCamera() );

        this->top = teamTop;
        this->bottom = teamBottom;
        this->nameActor = nameActor;
}

void RobotVisualization::updatePosition(vtkRenderer *renderer)
{
        if (false == visible)
          this->init(renderer);

        auto pos = FieldWidget3D::transform(robot->getSharedWorldInfo()->odom.position.x, robot->getSharedWorldInfo()->odom.position.y);
        move(pos.first / 1000, pos.second / 1000, 0);
        turn(robot->getSharedWorldInfo()->odom.position.angle);
}


void RobotVisualization::updateBall(vtkRenderer *renderer)
{
        if (false == visible)
          this->init(renderer);

        if (this->ball == nullptr && robot->getSharedWorldInfo()->ball.confidence > 0)
        {
                cout << "FieldWidget no ball 2" << endl;
                vtkSmartPointer<vtkSphereSource> sphereSrc = vtkSmartPointer<vtkSphereSource>::New();
                sphereSrc->SetRadius(field->_BALL_DIAMETER / 2);
                vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                sphereMapper->SetInput(sphereSrc->GetOutput());
                this->ball = vtkActor::New(); // TODO memory leak!
                this->ball->SetMapper(sphereMapper);
                this->ball->GetProperty()->SetRepresentationToSurface();
                this->ball->GetProperty()->SetColor(255, 0, 0);
                this->ball->SetPosition(1000, 1000, this->field->_BALL_DIAMETER / 2);
                renderer->AddActor(this->ball);

                vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
                line->SetPoint1(this->ball->GetPosition()[0], this->ball->GetPosition()[1],
                                this->ball->GetPosition()[2]);
                line->SetPoint2(this->ball->GetPosition()[0], this->ball->GetPosition()[1],
                                this->ball->GetPosition()[2]);
                this->ballVelocity = line;

                vtkSmartPointer<vtkPolyDataMapper> velocityMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                velocityMapper->SetInput(line->GetOutput());
                vtkSmartPointer<vtkActor> velocity = vtkSmartPointer<vtkActor>::New();
                velocity->SetMapper(velocityMapper);
                velocity->GetProperty()->SetLineWidth(this->field->_LINE_THICKNESS / 2);
                velocity->GetProperty()->SetColor(1, 0, 0);
                velocity->GetProperty()->SetDiffuse(0.4);
                velocity->GetProperty()->SetAmbient(0.8);
                this->ballVelocityActor = velocity;
                renderer->AddActor(velocity);
        }
        else if (this->ball != nullptr && robot->getSharedWorldInfo()->ball.confidence > 0)
        {
                auto pos = FieldWidget3D::transform(robot->getSharedWorldInfo()->ball.point.x, robot->getSharedWorldInfo()->ball.point.y);

                this->ball->SetPosition(pos.first / 1000, pos.second / 1000, robot->getSharedWorldInfo()->ball.point.z / 1000 );
                this->ballVelocity->SetPoint1(pos.first / 1000, pos.second / 1000, robot->getSharedWorldInfo()->ball.point.z / 1000 );
                auto ballVelTrans = FieldWidget3D::transform(robot->getSharedWorldInfo()->ball.velocity.vx, robot->getSharedWorldInfo()->ball.velocity.vy);
                this->ballVelocity->SetPoint2(pos.first / 1000 + ballVelTrans.first / 1000, pos.second / 1000 + ballVelTrans.second / 1000,
                                                    robot->getSharedWorldInfo()->ball.point.z / 1000 + robot->getSharedWorldInfo()->ball.velocity.vz / 1000);
        }
        else if (this->ball != nullptr && robot->getSharedWorldInfo()->ball.confidence == 0)
        {
                renderer->RemoveActor(this->ball);
                renderer->RemoveActor(this->ballVelocityActor);
                this->ball = nullptr;
                this->ballVelocityActor = nullptr;
        }
}


void RobotVisualization::updateSharedBall(vtkRenderer *renderer)
{
        if (false == visible)
          this->init(renderer);

        // Draw shared ball
        if (this->sharedBall == nullptr && robot->getSharedWorldInfo()->sharedBall.confidence > 0)
        {
                vtkSmartPointer<vtkRegularPolygonSource> polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();

                //polygonSource->GeneratePolygonOff();
                polygonSource->SetNumberOfSides(50);
                polygonSource->SetRadius(this->field->_BALL_DIAMETER * 1.5);
                polygonSource->Update();

                // Visualize
                vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                mapper->SetInputConnection(polygonSource->GetOutputPort());
                vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
                actor->SetMapper(mapper);
                actor->GetProperty()->SetColor(0, 0, 0.5);
                actor->GetProperty()->SetDiffuse(0.4);
                actor->GetProperty()->SetAmbient(0.8);

                this->sharedBall = actor;
                renderer->AddActor(actor);
        }
        else if (this->sharedBall != nullptr && robot->getSharedWorldInfo()->sharedBall.confidence > 0)
        {
                auto pos = FieldWidget3D::transform(robot->getSharedWorldInfo()->sharedBall.point.x, robot->getSharedWorldInfo()->sharedBall.point.y);
                this->sharedBall->SetPosition(pos.first / 1000, pos.second / 1000, robot->getSharedWorldInfo()->sharedBall.point.z / 1000 + 0.01);
        }
        else if (this->sharedBall != nullptr && robot->getSharedWorldInfo()->sharedBall.confidence == 0)
        {
                renderer->RemoveActor(this->sharedBall);
                this->sharedBall = nullptr;
        }
}

void RobotVisualization::updateOpponents(vtkRenderer *renderer)
{
        bool alreadyIn = false;

        for (shared_ptr<RobotVisualization> actor : obstacles)
        {
                renderer->RemoveActor(actor->getTop());
                renderer->RemoveActor(actor->getBottom());
        }
        obstacles.clear();

        for (auto x : robot->getSharedWorldInfo()->obstacles)
        {
                auto pos = FieldWidget3D::transform(x.x, x.y);
                for (auto member : *this->field->getRobots())
                {
                        auto mb = member->getVisualization()->getBottom();
                        if (mb == nullptr)
                          continue;

                        if (abs(mb->GetPosition()[0] - pos.first / 1000) < 0.25
                                        && abs(mb->GetPosition()[1] - pos.second / 1000) < 0.25)
                        {
                                alreadyIn = true;
                        }
                }

                if (!alreadyIn)
                {
                        drawOpponent(renderer, pos.first / 1000, pos.second / 1000, 0);
                }
                alreadyIn = false;
        }
}

void RobotVisualization::turn(double angle)
{
        this->top->SetOrientation(0, 0, angle * (180.0 / (double)M_PI) + 90);
        this->bottom->SetOrientation(0, 0, angle * (180.0 / (double)M_PI) + 90);
}

void RobotVisualization::move(double x, double y, double z)
{
        this->top->SetPosition(x, y, z + 0.4);
        this->bottom->SetPosition(x, y, z + 0.2);
        this->nameActor->SetPosition(x, y, z + 1);
}


void RobotVisualization::drawOpponent(vtkRenderer *renderer, double x, double y, double z)
{
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

        float p0[3] = {0.26, 0.26, 0};
        float p1[3] = {-0.26, 0.26, 0};
        float p2[3] = {-0.26, -0.26, 0};
        float p3[3] = {0.26, -0.26, 0};
        float p4[3] = {0.0, 0.0, 0.4};

        points->InsertNextPoint(p0);
        points->InsertNextPoint(p1);
        points->InsertNextPoint(p2);
        points->InsertNextPoint(p3);
        points->InsertNextPoint(p4);

        vtkSmartPointer<vtkPyramid> pyramid = vtkSmartPointer<vtkPyramid>::New();
        pyramid->GetPointIds()->SetId(0, 0);
        pyramid->GetPointIds()->SetId(1, 1);
        pyramid->GetPointIds()->SetId(2, 2);
        pyramid->GetPointIds()->SetId(3, 3);
        pyramid->GetPointIds()->SetId(4, 4);

        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
        cells->InsertNextCell(pyramid);

        vtkSmartPointer<vtkUnstructuredGrid> ug = vtkSmartPointer<vtkUnstructuredGrid>::New();
        ug->SetPoints(points);
        ug->InsertNextCell(pyramid->GetCellType(), pyramid->GetPointIds());

        vtkSmartPointer<vtkCubeSource> cubeSrc = vtkSmartPointer<vtkCubeSource>::New();
        cubeSrc->SetXLength(0.52);
        cubeSrc->SetYLength(0.52);
        cubeSrc->SetZLength(0.4);

        vtkSmartPointer<vtkPolyDataMapper> obstacleBottomMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        obstacleBottomMapper->SetInputConnection(cubeSrc->GetOutputPort());

        vtkSmartPointer<vtkDataSetMapper> obstacleTopMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        obstacleTopMapper->SetInput(ug);

        vtkSmartPointer<vtkActor> obstacleBottom = vtkSmartPointer<vtkActor>::New();
        obstacleBottom->SetMapper(obstacleBottomMapper);
        obstacleBottom->SetPosition(x, y, z + 0.2);
        obstacleBottom->GetProperty()->SetColor(0, 0, 0);
        obstacleBottom->GetProperty()->SetDiffuse(0.4);
        obstacleBottom->GetProperty()->SetAmbient(0.8);
        renderer->AddActor(obstacleBottom);

        vtkSmartPointer<vtkActor> obstacleTop = vtkSmartPointer<vtkActor>::New();
        obstacleTop->SetMapper(obstacleTopMapper);
        obstacleTop->SetPosition(x, y, z + 0.4);
        obstacleTop->GetProperty()->SetColor(0, 0, 0);
        obstacleTop->GetProperty()->SetDiffuse(0.4);
        obstacleTop->GetProperty()->SetAmbient(0.8);
        renderer->AddActor(obstacleTop);

        shared_ptr<RobotVisualization> robot = make_shared<RobotVisualization>(this->robot, this->field); // TODO create own obstacle object
        robot->setTop(obstacleTop);
        robot->setBottom(obstacleBottom);
        obstacles.push_front(robot);
}

void RobotVisualization::updatePathPlannerDebug(vtkRenderer *renderer, bool show)
{
        // Remove old objects if show path is disabled
        if (false == show)
        {
                for (vtkSmartPointer<vtkActor> actor : pathLines)
                {
                        if (actor != nullptr)
                        {
                                renderer->RemoveActor(actor);
                        }
                }
                pathLines.clear();
                return;
        }

        // Check last message
        boost::shared_ptr<msl_msgs::PathPlanner> ppi;
        {
              lock_guard<mutex> lock(this->field->pathMutex);
              ppi = this->robot->getPathPlannerInfo();
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

        // Draw new
        for (int i = 1; i < ppi->pathPoints.size(); i++)
        {
                vtkSmartPointer<vtkActor> actor = FieldWidget3D::createLine(ppi->pathPoints.at(i - 1).y / 1000,
                                                                                      -ppi->pathPoints.at(i - 1).x / 1000,
                                                                                      0.01, ppi->pathPoints.at(i).y / 1000,
                                                                                      -ppi->pathPoints.at(i).x / 1000, 0.01,
                                                                                      3,
                                                                                      Color::getColor(this->robot->getId()));
                pathLines.push_back(actor);
                renderer->AddActor(actor);
        }
}

void RobotVisualization::updateCorridorDebug(vtkRenderer *renderer, bool show)
{
        // Remove old objects if show path is disabled
        if (false == show)
        {
                for (vtkSmartPointer<vtkActor> actor : corridorLines)
                {
                        if (actor != nullptr)
                        {
                                renderer->RemoveActor(actor);
                        }
                }
                corridorLines.clear();
                return;
        }

        // Check last message
        boost::shared_ptr<msl_msgs::CorridorCheck> cc;
        {
                lock_guard<mutex> lock(this->field->corridorMutex);
                cc = this->robot->getCorridorCheckInfo();

                if (false == cc)
                        return;
        }

        // Remove old stuff
        for (vtkSmartPointer<vtkActor> actor : corridorLines)
        {
                if (actor != nullptr)
                {
                        renderer->RemoveActor(actor);
                }
        }
        corridorLines.clear();

        // Draw new
        vtkSmartPointer<vtkActor> actor = FieldWidget3D::createDashedLine(cc->corridorPoints.at(0).y / 1000,
                                                                                                -cc->corridorPoints.at(0).x / 1000, 0.01,
                                                                                                cc->corridorPoints.at(1).y / 1000,
                                                                                                -cc->corridorPoints.at(1).x / 1000, 0.01,
                                                                                                3, this->getDashedPattern(),
                                                                                                Color::getColor(this->robot->getId()));
        vtkSmartPointer<vtkActor> actor2 = FieldWidget3D::createDashedLine(cc->corridorPoints.at(1).y / 1000,
                                                                                                -cc->corridorPoints.at(1).x / 1000, 0.01,
                                                                                                cc->corridorPoints.at(2).y / 1000,
                                                                                                -cc->corridorPoints.at(2).x / 1000, 0.01,
                                                                                                3, this->getDashedPattern(),
                                                                                                Color::getColor(this->robot->getId()));
        vtkSmartPointer<vtkActor> actor3 = FieldWidget3D::createDashedLine(cc->corridorPoints.at(2).y / 1000,
                                                                                                -cc->corridorPoints.at(2).x / 1000, 0.01,
                                                                                                cc->corridorPoints.at(3).y / 1000,
                                                                                                -cc->corridorPoints.at(3).x / 1000, 0.01,
                                                                                                3, this->getDashedPattern(),
                                                                                                Color::getColor(this->robot->getId()));
        vtkSmartPointer<vtkActor> actor4 = FieldWidget3D::createDashedLine(cc->corridorPoints.at(3).y / 1000,
                                                                                                -cc->corridorPoints.at(3).x / 1000, 0.01,
                                                                                                cc->corridorPoints.at(0).y / 1000,
                                                                                                -cc->corridorPoints.at(0).x / 1000, 0.01,
                                                                                                3, this->getDashedPattern(),
                                                                                                Color::getColor(this->robot->getId()));
        corridorLines.push_back(actor);
        corridorLines.push_back(actor2);
        corridorLines.push_back(actor3);
        corridorLines.push_back(actor4);
        renderer->AddActor(actor);
        renderer->AddActor(actor2);
        renderer->AddActor(actor3);
        renderer->AddActor(actor4);
}

void RobotVisualization::updateVoronoiNetDebug(vtkRenderer *renderer, bool showVoronoi, bool showSitePoints)
{
        // Remove old objects if show path is disabled
        if (false == showVoronoi)
        {
                for (vtkSmartPointer<vtkActor> actor : this->netLines)
                {
                        if (actor != nullptr)
                        {
                                renderer->RemoveActor(actor);
                        }
                }
                netLines.clear();
        }

        if (false == showSitePoints)
        {
                for (vtkSmartPointer<vtkActor> actor : sitePoints)
                {
                        if (actor != nullptr)
                        {
                                renderer->RemoveActor(actor);
                        }
                }
                sitePoints.clear();
        }

        if(false == showVoronoi && false == showSitePoints)
        {
                return;
        }

        // Check last message
        boost::shared_ptr<msl_msgs::VoronoiNetInfo> vni;
        {
                lock_guard<mutex> lock(this->field->voronoiMutex);
                vni = this->robot->getVoronoiNetInfo();
                if (false == vni)
                        return;
        }

        // Remove old stuff
        for (vtkSmartPointer<vtkActor> actor : sitePoints)
        {
                if (actor != nullptr)
                {
                        renderer->RemoveActor(actor);
                }
        }
        sitePoints.clear();

        int used = 0;
        vtkSmartPointer<vtkActor> actor;

        // Draw voronoi net
        if(showVoronoi)
        {
                for (int i = 1; i < vni->linePoints.size(); i += 2)
                {
                        double x1 = vni->linePoints.at(i - 1).y / 1000;
                        double y1 = -vni->linePoints.at(i - 1).x / 1000;
                        double x2 = vni->linePoints.at(i).y / 1000;
                        double y2 =-vni->linePoints.at(i).x / 1000;

                        if (used >= this->netLines.size())
                        {
                                actor = FieldWidget3D::createDashedLine(x1, y1, 0.01, x2, y2, 0.01, 3, this->getDashedPattern(), Color::getColor(this->robot->getId()));
                                this->netLines.push_back(actor);
                                renderer->AddActor(actor);
                        }
                        else
                        {
                                FieldWidget3D::updateLine(this->netLines.at(used), x1, y1, 0.01, x2, y2, 0.01);
                        }
                        ++used;
                }

                if (used < this->netLines.size())
                {
                        for (int i=used; i < this->netLines.size(); ++i)
                        {
                                actor = this->netLines.at(i);
                                if (actor != nullptr)
                                {
                                  renderer->RemoveActor(actor);
                                }
                        }

                        this->netLines.erase(this->netLines.begin() + used, this->netLines.end());
                }
        }

        // Draw side points
        if(showSitePoints)
        {
                for (vtkSmartPointer<vtkActor> actor : sitePoints)
                {
                        if (actor != nullptr)
                        {
                                renderer->RemoveActor(actor);
                        }
                }
                sitePoints.clear();

                for (int i = 0; i < vni->sites.size(); i++)
                {
                        vtkSmartPointer<vtkActor> actor = FieldWidget3D::createDot(vni->sites.at(i).y / 1000,
                                                                                  -vni->sites.at(i).x / 1000,
                                                                                   0.3,
                                                                                   Color::getColor(this->robot->getId()));
                        sitePoints.push_back(actor);
                        renderer->AddActor(actor);
                }
        }
}

int RobotVisualization::getDashedPattern()
{
        return 0x66 << this->robot->getId();
}
