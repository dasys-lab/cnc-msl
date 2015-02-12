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

#include "FieldWidget3D.h"
#include <vtkLineWidget.h>

//#include "ConfigXML.h"

using namespace cambada;

class MouseInteractorStyle : public vtkInteractorStyleTerrain
{
public:
	static MouseInteractorStyle* New();vtkTypeMacro(MouseInteractorStyle, vtkInteractorStyleTerrain)
	;

	// Initialize internal variables
	MouseInteractorStyle()
	{
		LastPickedActor = NULL;
		LastPickedProperty = vtkProperty::New();
		robotIdx = -1;
		FieldCollection = vtkPropCollection::New();

	}
	virtual ~MouseInteractorStyle()
	{
		LastPickedProperty->Delete();
		FieldCollection->Delete();
	}

	void setParent(FieldWidget3D* p)
	{
		this->parent = p;
		FieldCollection->AddItem(parent->field);
	}

	// When the left button of the mouse is pressed
	virtual void OnLeftButtonDown()
	{
		// Decide if drag of a robot or field orientation change
		fprintf(stderr, "LEFT MOUSE BUTTON PRESSED\n");
		// Get the position where the mouse pointer was
		int* clickPos = this->GetInteractor()->GetEventPosition();
		// Create a picker with the clickPos information
		vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
		picker->Pick(clickPos[0], clickPos[1], 0, parent->renderer);
		// Check if the picker returns an actor
		this->LastPickedActor = picker->GetActor();
		if (LastPickedActor != NULL)
		{
			robotIdx = -1;
			for (int i = 0; i < NROBOTS; i++)
			{
				if (this->LastPickedActor == parent->robots[i])
				{
					fprintf(stderr, "ROBOT PICKED = %d\n", i);
					robotIdx = i;
					break;
				}
			}

			if (robotIdx < 0)
			{
				// Foward the event as a vtkInteractorStyleTerrain event
				if (!parent->lockCam && !parent->top)
					vtkInteractorStyleTerrain::OnLeftButtonDown();
			}
			else
			{
				// do something with the robot (drag, etc..)
			}
		}
		else
		{
			if (!parent->lockCam && !parent->top)
				vtkInteractorStyleTerrain::OnLeftButtonDown();
		}
	}

	virtual void OnLeftButtonUp()
	{
		fprintf(stderr, "LEFT MOUSE BUTTON RELEASED\n");

		// Get the position where the mouse pointer was
		int* clickPos = this->GetInteractor()->GetEventPosition();
		// Create a picker with the clickPos information
		vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
		picker->PickProp(clickPos[0], clickPos[1], parent->renderer, FieldCollection);

		//Check if the picker returns an actor
		if (picker->GetActor() != NULL && picker->GetActor() == parent->field)
		{
			double* position = picker->GetPickPosition();
			if (robotIdx >= 0 && robotIdx < NROBOTS)
			{
				Vec pos = Vec(position[0], position[1]);
				parent->db_coach_info->Coach_Info.taxiRobotSN[robotIdx]++;
				parent->db_coach_info->Coach_Info.taxiPos = pos;
			}
			fprintf(stderr, "POS: (%.1lf, %.1lf, %.1lf)\n", position[0], position[1], position[2]);
		}

		robotIdx = -1;

		if (!parent->lockCam && !parent->top)
			vtkInteractorStyleTerrain::OnLeftButtonUp();
	}

	virtual void OnMouseMove()
	{
		// Taxi Follow Test
		if (robotIdx >= 0)
		{
			int* clickPos = this->GetInteractor()->GetEventPosition();
			// Create a picker with the clickPos information
			vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
			picker->Pick(clickPos[0], clickPos[1], 0, parent->renderer);

			if (picker->GetActor() != NULL && picker->GetActor() == parent->field)
			{

				if (parent->taxiFollow)
				{
					// If taxiFollow is activated
					// If the picker returns a position within the field
					double* position = picker->GetPickPosition();
					// Pass the new vec to the db_coach_info struct
					Vec pos = Vec(position[0], position[1]);
					parent->db_coach_info->Coach_Info.taxiRobotSN[robotIdx]++;
					parent->db_coach_info->Coach_Info.taxiPos = pos;
					// Create the vector (actor)
					//double* startPosition = parent->robots[robotIdx]->GetPosition();

				}
			}
		}
		else
		{
			if (!parent->lockCam && !parent->top)
				vtkInteractorStyleTerrain::OnMouseMove();

			if (parent->camera->GetPosition()[2] < 0.0)
				parent->camera->SetPosition(parent->camera->GetPosition()[0], parent->camera->GetPosition()[1], 0.0);
		}
	}

private:
	int robotIdx;
	double* lastFollowPosition;
	vtkActor *LastPickedActor;
	FieldWidget3D* parent;
	vtkProperty *LastPickedProperty;
	vtkPropCollection *FieldCollection;

};
// define the previous class as a new vtk standard
vtkStandardNewMacro(MouseInteractorStyle);

FieldWidget3D::FieldWidget3D(QWidget *parent) :
		QVTKWidget(parent)
{
	this->parent = parent;
	rosNode = new ros::NodeHandle();
	savedSharedWorldInfo = list<boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo>>(ringBufferLength);
	sharedWorldInfoSubscriber = rosNode->subscribe("/SharedWorldInfo", 10, &FieldWidget3D::onSharedWorldInfo,
													(FieldWidget3D*)this);
	spinner = new ros::AsyncSpinner(1);
	spinner->start();
	/*ConfigXML config;
	 if( config.parse("../config/cambada.conf.xml") == false )
	 {
	 cerr << "ERROR " << endl;
	 exit(1);
	 }*/

	Update_timer = new QTimer();
	Update_timer->setInterval(1);
	connect(Update_timer, SIGNAL(timeout()), this, SLOT(update_robot_info()));

	/* Dimensions */
	_FIELD_LENGTH = 18;
	_FIELD_WIDTH = 12;
	_LINE_THICKNESS = 0.1;
	_GOAL_AREA_LENGTH = 2.5;
	_GOAL_AREA_WIDTH = 8;
	_PENALTY_AREA_LENGTH = 1;
	_PENALTY_AREA_WIDTH = 6;
	_CENTER_CIRCLE_RADIUS = 2;
	_BALL_DIAMETER = 0.25;
	_CORNER_CIRCLE_RADIUS = 0.3;
	_PENALTY_MARK_DISTANCE = 3;
	_BLACK_POINT_WIDTH = _FIELD_WIDTH / 4.0;
	_BLACK_POINT_LENGTH = 2;
	_ROBOT_RADIUS = 0.25;

	/* Init Colors */
	float robotsColorR[] = {244.0 / 255.0, 1, 1, 188.0 / 255.0, 201.0 / 255.0, 115.0 / 255.0};
	float robotsColorG[] = {194.0 / 255.0, 216.0 / 255.0, 153.0 / 255.0, 143.0 / 255.0, 160.0 / 255.0, 194.0 / 255.0};
	float robotsColorB[] = {194.0 / 255.0, 0, 153.0 / 255.0, 143.0 / 255.0, 220.0 / 255.0, 251.0 / 255.0};
	for (int i = 0; i < 6; i++)
	{
		this->robotsColorR[i] = robotsColorR[i];
		this->robotsColorG[i] = robotsColorG[i];
		this->robotsColorB[i] = robotsColorB[i];
	}

	renderWindow = vtkRenderWindow::New();
	renderer = vtkRenderer::New();
	renderer->SetBackground(72.0 / 255.0, 72.0 / 255.0, 72.0 / 255.0);

	renderWindow->AddRenderer(renderer);
	this->SetRenderWindow(renderWindow);

	//TODO drawing methods

	drawField(renderer);
	drawGoals(renderer);
	initBall(renderer);

	// Camera properties
	camera = vtkCamera::New();
	camera->SetPosition(_FIELD_WIDTH, 0, 22);
	camera->SetFocalPoint(0, 0, 0);
	camera->SetViewUp(0, 0, 1);
	renderer->SetActiveCamera(camera);

	// Interactor
	QVTKInteractor* iren = this->GetInteractor();
	vtkSmartPointer<MouseInteractorStyle> intStyle = vtkSmartPointer<MouseInteractorStyle>::New();
	intStyle->setParent(this);
	//vtkSmartPointer<vtkInteractorStyleTerrain> intStyle = vtkSmartPointer<vtkInteractorStyleTerrain>::New();
	iren->SetInteractorStyle(intStyle);
	renderWindow->SetInteractor(iren);
	// WIPFIX renderer->Render();

	/* Read CAMBADA model */
	vtkSmartPointer<vtkOBJReader> readerCbd = vtkSmartPointer<vtkOBJReader>::New();
	readerCbd->SetFileName("../config/3DModels/cambada_base.obj");

	vtkSmartPointer<vtkPolyDataMapper> actorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actorMapper->SetInput(readerCbd->GetOutput());

	// WIPFIX renderer->Render();

	velocityLineSrc = vtkLineSource::New();
	vtkSmartPointer<vtkPolyDataMapper> velLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	velLineMapper->SetInput(velocityLineSrc->GetOutput());
	velocityLine = vtkActor::New();
	velocityLine->SetMapper(velLineMapper);
	velocityLine->GetProperty()->SetColor(1.0, 0, 0);
	velocityLine->GetProperty()->SetLineWidth(2);
	velocityLine->GetProperty()->SetOpacity(0);
	renderer->AddActor(velocityLine);

	//renderer->AddLight();

	//vtkTextWidget* textCbd = vtkTextWidget::New();
	//textCbd->SetInteractor;

	// View heightmap
	heightVisible = false;
	heightColor = true;
	height3D = false;
	heightActor = NULL;

	lockCam = false;
	top = false;

	Update_timer->start(50);
}

/**
 Actualiza a informação dos objectos
 */
void FieldWidget3D::update_robot_info(void)
{
	lock_guard<mutex> lock(swmMutex);
	removeObstacles(renderer);
	for (auto robot : latestInfo)
	{

		auto tmp = ros::Time::now();
		unsigned long now = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;
		if((now - robot->getTimeStamp()) > 2000000000)
		{
			shared_ptr<RobotVisualization> toBeRemoved;
			for(auto r : team)
			{
				if(r->getId() == robot->getId())
				{
					renderer->RemoveActor(r->getTop());
					renderer->RemoveActor(r->getBottom());
					toBeRemoved = r;
					break;
				}
			}
			team.remove(toBeRemoved);
			continue;
		}
		for(auto x : robot->getMsg()->mergedOpponents)
		{
			drawOpponent(x.x / 1000, x.y / 1000, 0);
		}
		moveBall(robot->getMsg()->ball.point.x / 1000, robot->getMsg()->ball.point.y / 1000, robot->getMsg()->ball.point.z / 1000 + 0.11);
		bool alreadyIn = false;
		for (auto member : team)
		{
			if (member->getId() == robot->getMsg()->senderID)
			{
				alreadyIn = true;
			}
		}
		if (!alreadyIn)
		{
			shared_ptr<RobotVisualization> r = make_shared<RobotVisualization>();
			r->setId(robot->getMsg()->senderID);
			drawTeamRobot(r, robot->getMsg()->odom.position.x / 1000, robot->getMsg()->odom.position.y / 1000, 0);
		}
		else
		{
			for (auto member : team)
			{
				if (member->getId() == robot->getMsg()->senderID)
				{
					moveRobot(member, robot->getMsg()->odom.position.x / 1000, robot->getMsg()->odom.position.y / 1000, 0);
				}
			}

		}
		alreadyIn = false;
	}
	if (!this->GetRenderWindow()->CheckInRenderStatus())
	{
		this->GetRenderWindow()->Render();
	}
}

void FieldWidget3D::flip(void)
{
	if (top)
	{
		camera->SetPosition((camera->GetPosition()[0] < 0) ? 0.001 : -0.001, 0, 25);
		camera->SetFocalPoint(0, 0, 0);
		camera->SetViewUp((camera->GetPosition()[0] < 0) ? 1 : -1, 0, 0);
	}
	else
	{
		if (camera->GetPosition()[0] < 0)
			camera->SetPosition(_FIELD_WIDTH, 0, 22);
		else
			camera->SetPosition(-_FIELD_WIDTH, 0, 22);

		camera->SetFocalPoint(0, 0, 0);
		camera->SetViewUp(0, 0, 1);
	}
}

void FieldWidget3D::obstacles_point_flip(unsigned int Robot_no, bool on_off)
{
	if (Robot_no < NROBOTS)
		option_draw_obstacles[Robot_no] = on_off;
}

void FieldWidget3D::obstacles_point_flip_all(bool on_off)
{
	for (unsigned int i = 0; i < NROBOTS; i++)
		obstacles_point_flip(i, on_off);
}

void FieldWidget3D::debug_point_flip(unsigned int Robot_no, bool on_off)
{
	if (Robot_no < NROBOTS)
		option_draw_debug[Robot_no] = on_off;
}

void FieldWidget3D::debug_point_flip_all(bool on_off)
{
	for (unsigned int i = 0; i < NROBOTS; i++)
		debug_point_flip(i, on_off);
}

vtkActor* FieldWidget3D::createDashedLine(float x1, float y1, float z1, float x2, float y2, float z2)
{
	vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
	vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkActor* lineActor = vtkActor::New();
	line->SetPoint1(x1, y1, z1);
	line->SetPoint2(x2, y2, z2);
	lineMapper->SetInputConnection(line->GetOutputPort());
	lineActor->SetMapper(lineMapper);
	lineActor->GetProperty()->SetLineWidth(3);
	lineActor->GetProperty()->SetLineStipplePattern(0xf0f0);
	lineActor->GetProperty()->SetLineStippleRepeatFactor(1);
	lineActor->GetProperty()->SetPointSize(1);
	lineActor->GetProperty()->SetLineWidth(3);
	return lineActor;
}

vtkSmartPointer<vtkActor> FieldWidget3D::createLine(float x1, float y1, float z1, float x2, float y2, float z2)
{
	vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
	vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
	line->SetPoint1(x1, y1, z1);
	line->SetPoint2(x2, y2, z2);
	lineMapper->SetInputConnection(line->GetOutputPort());
	lineActor->SetMapper(lineMapper);
	lineActor->GetProperty()->SetLineWidth(3);

	return lineActor;
}

void FieldWidget3D::addArc(vtkRenderer *renderer, float x, float y, float radius, float startDeg, float endDeg)
{
	float x1, y1, x2, y2;
	x2 = x + radius * cos(startDeg * M_PI / 180);
	y2 = y + radius * sin(startDeg * M_PI / 180);
	for (int i = startDeg + 10; i <= endDeg; i += 10)
	{
		x1 = x + radius * cos(i * M_PI / 180);
		y1 = y + radius * sin(i * M_PI / 180);
		renderer->AddActor(createLine(x1, y1, 0, x2, y2, 0));
		x2 = x1;
		y2 = y1;
	}
}

void FieldWidget3D::get_info_pointer(DB_Robot_Info * rw)
{
	DB_Info = rw;
}

void FieldWidget3D::get_coach_pointer(DB_Coach_Info * ci)
{
	db_coach_info = ci;
}

void FieldWidget3D::drawGoals(vtkRenderer* renderer)
{
	// Goals
	// vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
	vtkSmartPointer<vtkCubeSource> cubeSrc = vtkSmartPointer<vtkCubeSource>::New();
	cubeSrc->SetXLength(0.125);
	cubeSrc->SetYLength(0.125);
	cubeSrc->SetZLength(1);
	vtkSmartPointer<vtkCubeSource> cubeSrc2 = vtkSmartPointer<vtkCubeSource>::New();
	cubeSrc2->SetXLength(2 + 2 * 0.125);
	cubeSrc2->SetYLength(0.125);
	cubeSrc2->SetZLength(0.125);
	//  reader->SetFileName("../config/3DModels/goal.obj");
	//reader->Update();

	vtkSmartPointer<vtkPolyDataMapper> goalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	goalMapper->SetInput(cubeSrc->GetOutput());

	vtkSmartPointer<vtkPolyDataMapper> goalMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	goalMapper2->SetInput(cubeSrc2->GetOutput());

	vtkSmartPointer<vtkActor> goalBlue = vtkSmartPointer<vtkActor>::New();
	goalBlue->SetMapper(goalMapper);
	goalBlue->SetPosition(0 - 1 - 0.125 / 2, -_FIELD_LENGTH / 2 - 0.125 / 2, 0.5);
	goalBlue->GetProperty()->SetColor(0.2, 0.2, 1);
	goalBlue->GetProperty()->SetDiffuse(0.4);
	goalBlue->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalBlue);

	vtkSmartPointer<vtkActor> goalBlue2 = vtkSmartPointer<vtkActor>::New();
	goalBlue2->SetMapper(goalMapper);
	goalBlue2->SetPosition(0 + 1 + 0.125 / 2, -_FIELD_LENGTH / 2 - 0.125 / 2, 0.5);
	goalBlue2->GetProperty()->SetColor(0.2, 0.2, 1);
	goalBlue2->GetProperty()->SetDiffuse(0.4);
	goalBlue2->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalBlue2);

	vtkSmartPointer<vtkActor> goalBlue3 = vtkSmartPointer<vtkActor>::New();
	goalBlue3->SetMapper(goalMapper2);
	goalBlue3->SetPosition(0, -_FIELD_LENGTH / 2 - 0.125 / 2, 1);
	goalBlue3->GetProperty()->SetColor(0.2, 0.2, 1);
	goalBlue3->GetProperty()->SetDiffuse(0.4);
	goalBlue3->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalBlue3);

	vtkSmartPointer<vtkActor> goalYellow = vtkSmartPointer<vtkActor>::New();
	goalYellow->SetMapper(goalMapper);
	goalYellow->SetPosition(0 - 1 - 0.125 / 2, _FIELD_LENGTH / 2 + 0.125 / 2, 0.5);
	goalYellow->GetProperty()->SetColor(1, 1, 0.2);
	goalYellow->GetProperty()->SetDiffuse(0.4);
	goalYellow->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalYellow);

	vtkSmartPointer<vtkActor> goalYellow2 = vtkSmartPointer<vtkActor>::New();
	goalYellow2->SetMapper(goalMapper);
	goalYellow2->SetPosition(0 + 1 + 0.125 / 2, _FIELD_LENGTH / 2 + 0.125 / 2, 0.5);
	goalYellow2->GetProperty()->SetColor(1, 1, 0.2);
	goalYellow2->GetProperty()->SetDiffuse(0.4);
	goalYellow2->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalYellow2);

	vtkSmartPointer<vtkActor> goalYellow3 = vtkSmartPointer<vtkActor>::New();
	goalYellow3->SetMapper(goalMapper2);
	goalYellow3->SetPosition(0, _FIELD_LENGTH / 2 + 0.125 / 2, 1);
	goalYellow3->GetProperty()->SetColor(1, 1, 0.2);
	goalYellow3->GetProperty()->SetDiffuse(0.4);
	goalYellow3->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalYellow3);
}

void FieldWidget3D::drawField(vtkRenderer* renderer)
{
	// Draw plane
	vtkSmartPointer<vtkPlaneSource> planeSrc = vtkSmartPointer<vtkPlaneSource>::New();
	planeSrc->SetOrigin(0, 0, 0);
	planeSrc->SetPoint1(+_FIELD_WIDTH + 2.0, 0, 0);
	planeSrc->SetPoint2(0, _FIELD_LENGTH + 2.0, 0);
	vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	planeMapper->SetInput(planeSrc->GetOutput());
	this->field = vtkActor::New();
	this->field->SetMapper(planeMapper);
	this->field->GetProperty()->SetColor(0.278, 0.64, 0.196);
	this->field->SetPosition(-_FIELD_WIDTH / 2 - 1.0, -_FIELD_LENGTH / 2 - 1.0, -0.02);
	this->field->GetProperty()->SetAmbient(1);
	this->field->GetProperty()->SetDiffuse(0);
	this->field->GetProperty()->SetSpecular(0);
	renderer->AddActor(field);

	// Draw Field
	renderer->AddActor(createLine(-_FIELD_WIDTH / 2, 0.0, 0.0, _FIELD_WIDTH / 2, 0.0, 0.0));
	renderer->AddActor(
			createLine(-_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, -_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0));
	renderer->AddActor(createLine(_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0));
	renderer->AddActor(createLine(-_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0));
	renderer->AddActor(
			createLine(-_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0));

	// Goal Areas
	renderer->AddActor(
			createLine(-_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0, _GOAL_AREA_WIDTH / 2,
						-_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(-_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, -_GOAL_AREA_WIDTH / 2,
						-_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _GOAL_AREA_WIDTH / 2,
						-_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(-_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0, _GOAL_AREA_WIDTH / 2,
						_FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(-_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, -_GOAL_AREA_WIDTH / 2,
						_FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _GOAL_AREA_WIDTH / 2,
						_FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0));

	// Penalty Areas
	renderer->AddActor(
			createLine(-_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0,
						_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(-_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, -_PENALTY_AREA_WIDTH / 2,
						-_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _PENALTY_AREA_WIDTH / 2,
						-_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(-_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0, _PENALTY_AREA_WIDTH / 2,
						_FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(-_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, -_PENALTY_AREA_WIDTH / 2,
						_FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));
	renderer->AddActor(
			createLine(_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _PENALTY_AREA_WIDTH / 2,
						_FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));

	// Corner Circles
	addArc(renderer, _FIELD_WIDTH / 2, _FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 180, 270);
	addArc(renderer, -_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 270, 360);
	addArc(renderer, -_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 0, 90);
	addArc(renderer, _FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 90, 180);

	// Center Circle
	addArc(renderer, 0, 0, _CENTER_CIRCLE_RADIUS, 0, 360);

	// Black Dots
	createDot(renderer, _FIELD_WIDTH / 4, 0, true);
	createDot(renderer, -_FIELD_WIDTH / 4, 0, true);

	createDot(renderer, _FIELD_WIDTH / 4, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, true);
	createDot(renderer, -_FIELD_WIDTH / 4, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, true);
	createDot(renderer, 0, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, false);

	createDot(renderer, _FIELD_WIDTH / 4, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, true);
	createDot(renderer, -_FIELD_WIDTH / 4, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, true);
	createDot(renderer, 0, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, false);

	createDot(renderer, 0, 0, false, 0.1);

	renderer->AddActor(
			createLine(_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _PENALTY_AREA_WIDTH / 2,
						_FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));
}

void FieldWidget3D::createDot(vtkRenderer* renderer, float x, float y, bool black, float radius)
{
	vtkSmartPointer<vtkCylinderSource> dot = vtkSmartPointer<vtkCylinderSource>::New();
	dot->SetRadius(radius);
	dot->SetHeight(0.001);
	dot->SetResolution(32);
	vtkSmartPointer<vtkPolyDataMapper> dotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	dotMapper->SetInput(dot->GetOutput());

	vtkSmartPointer<vtkActor> blackDot1 = vtkSmartPointer<vtkActor>::New();
	blackDot1->SetMapper(dotMapper);

	if (black)
		blackDot1->GetProperty()->SetColor(0, 0, 0);
	else
		blackDot1->GetProperty()->SetColor(1, 1, 1);

	blackDot1->SetPosition(x, y, 0.01);
	blackDot1->SetOrientation(90, 0, 0);
	blackDot1->GetProperty()->SetAmbient(1.0);
	renderer->AddActor(blackDot1);
}

void FieldWidget3D::initBall(vtkRenderer* renderer)
{
	vtkSmartPointer<vtkSphereSource> sphereSrc = vtkSmartPointer<vtkSphereSource>::New();
	sphereSrc->SetRadius(0.11);
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInput(sphereSrc->GetOutput());
	ball = vtkActor::New();
	ball->SetMapper(sphereMapper);
	ball->GetProperty()->SetRepresentationToSurface();
	ball->GetProperty()->SetColor(255, 0, 0);
	ball->SetPosition(0, 0, 0.11);
	renderer->AddActor(ball);

}

vtkActor* FieldWidget3D::createText(QString text)
{
	vtkActor* actor = vtkActor::New();
	vtkSmartPointer<vtkVectorText> txt = vtkSmartPointer<vtkVectorText>::New();
	txt->SetText(text.toStdString().c_str());
	vtkSmartPointer<vtkPolyDataMapper> txtRobotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	txtRobotMapper->SetInput(txt->GetOutput());
	actor->SetMapper(txtRobotMapper);
	actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
	actor->GetProperty()->SetAmbient(1.0);
	actor->SetOrientation(0, 0, 90);
	return actor;
}

vtkActor* FieldWidget3D::createObstacle()
{
	// Obstacle actors
	vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
	cylinder->SetRadius(0.25);
	cylinder->SetHeight(OBSTACLE_HEIGHT);
	cylinder->SetResolution(12);
	vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	cylinderMapper->SetInput(cylinder->GetOutput());

	vtkActor* obstacleActor = vtkActor::New();
	obstacleActor->SetMapper(cylinderMapper);
	obstacleActor->GetProperty()->SetColor(0, 0, 0);
	//obstacleActor->GetProperty()->SetRepresentationToWireframe();
	obstacleActor->RotateX(90); // Rotate 90 degrees in XX axis

	return obstacleActor;
}

vtkActor* FieldWidget3D::createDebugPt()
{
	// Setup four points
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	float zz = 0.00;

	points->InsertNextPoint(0.05, 0.05, zz);
	points->InsertNextPoint(0.15, 0.05, zz);
	points->InsertNextPoint(0.15, -0.05, zz);
	points->InsertNextPoint(0.05, -0.05, zz);
	points->InsertNextPoint(0.05, -0.15, zz);
	points->InsertNextPoint(-0.05, -0.15, zz);
	points->InsertNextPoint(-0.05, -0.05, zz);
	points->InsertNextPoint(-0.15, -0.05, zz);
	points->InsertNextPoint(-0.15, 0.05, zz);
	points->InsertNextPoint(-0.05, 0.05, zz);
	points->InsertNextPoint(-0.05, 0.15, zz);
	points->InsertNextPoint(0.05, 0.15, zz);

	int nPoints = 12;

	// Create the polygon
	vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
	polygon->GetPointIds()->SetNumberOfIds(nPoints);
	for (int i = 0; i < nPoints; i++)
		polygon->GetPointIds()->SetId(i, i);

	// Add the polygon to a list of polygons
	vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
	polygons->InsertNextCell(polygon);

	// Create a PolyData
	vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
	polygonPolyData->SetPoints(points);
	polygonPolyData->SetPolys(polygons);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInput(polygonPolyData);

	vtkActor* actor = vtkActor::New();
	actor->SetMapper(mapper);
	actor->RotateZ(45);

	return actor;
}

void FieldWidget3D::updateGridView()
{
	if (!heightVisible)
	{
		if (heightActor != NULL)
			deleteGridView();

		return;
	}
	else if (heightActor == NULL)
	{
		initGridView();
	}

	heightActor->SetVisibility(1);
	heightActor->GetProperty()->SetOpacity(0.8);

	// GRID SIZE: 81x57

	//int sizeL = 81;
	//int sizeW = 57;
	//double resolution = 0.25;
	double xx, yy, zz;
	double minz = 2010.0;
	double maxz = -2010.0;
	int i = 0;

	//addtions
	//GridView grid;
	//DB_get(0,GRIDVIEW, &grid);
	/*if(grid.count > 0)
	 {

	 for(int i=0;i<grid.count;i++)
	 {
	 xx=grid.grid[i].pos.x;
	 yy=grid.grid[i].pos.y;
	 float realVal = grid.grid[i].val;

	 if(!height3D)
	 zz = -0.01;
	 else
	 zz = realVal;
	 heightPoints->SetPoint(i, xx,yy,zz);

	 if(i==0)
	 {
	 minz = realVal;
	 maxz = realVal;
	 }else{
	 if(realVal < minz)
	 minz = realVal;
	 if(realVal > maxz)
	 maxz = realVal;
	 }
	 //cout<<"XX="<<xx<<" YY="<<yy<<" ZZ="<<zz<<" I="<<i<<endl;

	 }

	 }else{
	 minz = 0.0;
	 maxz = 0.0;
	 }
	 */
	if (height3D)
	{
		for (int j = 0; j < i; j++)
		{
			heightPoints->SetPoint(j, heightPoints->GetPoint(j)[0], heightPoints->GetPoint(j)[1],
									heightPoints->GetPoint(j)[2] - minz);
		}
	}

	heightPolyData->SetPoints(heightPoints);
	heightDelaunay->SetInput(heightPolyData);
	heightDelaunay->Update();
	heightPolyDataAfterInterp = heightDelaunay->GetOutput();

	// Create the color map
	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(minz, maxz);

	if (heightColor)
	{
		colorLookupTable->SetValueRange(1, 1);
		colorLookupTable->SetSaturationRange(1, 1);
		//colorLookupTable->SetHueRange(0, 1);
	}
	else
	{
		colorLookupTable->SetValueRange(0, 1);
		colorLookupTable->SetSaturationRange(0, 0);
		//colorLookupTable->SetHueRange(0, 0);
	}
	colorLookupTable->Build();

	// Generate the colors for each point based on the color map
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);

	for (int i = 0; i < heightPolyDataAfterInterp->GetNumberOfPoints(); i++)
	{
		double p[3];
		heightPolyDataAfterInterp->GetPoint(i, p);

		double dcolor[3];
		//colorLookupTable->GetColor(grid.grid[i].val, dcolor);
		unsigned char color[3];
		for (unsigned int j = 0; j < 3; j++)
		{
			color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
		}
		colors->InsertNextTupleValue(color);
	}

	heightPolyDataAfterInterp->GetPointData()->SetScalars(colors);
}

void FieldWidget3D::initGridView()
{

	heightPoints = vtkPoints::New(); // Create a grid of points (height/terrian map)
	heightPolyData = vtkPolyData::New();
	heightDelaunay = vtkDelaunay2D::New();

	int sizeL = 81;
	int sizeW = 57;
	float resolution = 0.25;
	for (int x = -sizeW / 2; x <= sizeW / 2; x++)
	{
		for (int y = -sizeL / 2; y <= sizeL / 2; y++)
		{
			heightPoints->InsertNextPoint(x * resolution, y * resolution, x * resolution * y * resolution);
		}
	}

	heightPolyData->SetPoints(heightPoints); // Add the grid points to a polydata object
	heightDelaunay->SetInput(heightPolyData);

	// Triangulate the grid points
	heightDelaunay->Update();
	heightPolyDataAfterInterp = heightDelaunay->GetOutput();

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(heightPolyDataAfterInterp->GetProducerPort());
	heightActor = vtkActor::New();
	heightActor->SetMapper(mapper);
	heightActor->SetVisibility(0);
	heightActor->GetProperty()->SetAmbient(1);
	heightActor->GetProperty()->SetSpecular(0);
	heightActor->GetProperty()->SetDiffuse(0);

	// Add the actor to the scene
	renderer->AddActor(heightActor);
}

void FieldWidget3D::deleteGridView()
{
	if (heightActor == NULL)
		return;

	renderer->RemoveActor(heightActor);

	heightDelaunay->Delete();
	heightPolyDataAfterInterp->Delete();
	heightPolyData->Delete();
	heightPoints->Delete();

	heightActor = NULL;
	heightDelaunay = NULL;
	heightPolyDataAfterInterp = NULL;
	heightPolyData = NULL;
	heightPoints = NULL;

}

void FieldWidget3D::setTop(bool top)
{
	this->top = top;

	fprintf(stderr, "TOP\n");

	if (top)
	{
		camera->SetPosition((camera->GetPosition()[0] < 0) ? -0.001 : 0.001, 0, 25);
		camera->SetFocalPoint(0, 0, 0);
		camera->SetViewUp((camera->GetPosition()[0] < 0) ? 1 : -1, 0, 0);
	}
	else
	{
		if (camera->GetPosition()[0] < 0)
			camera->SetPosition(-_FIELD_WIDTH, 0, 22);
		else
			camera->SetPosition(_FIELD_WIDTH, 0, 22);

		camera->SetFocalPoint(0, 0, 0);
		camera->SetViewUp(0, 0, 1);
	}
}

void FieldWidget3D::lock(bool lock)
{
	this->lockCam = lock;
}

//TODO marker to find implemented methods

void FieldWidget3D::onSharedWorldInfo(boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> info)
{
	lock_guard<mutex> lock(swmMutex);
	int i = 0;
	if (i > ringBufferLength)
	{
		savedSharedWorldInfo.pop_back();
		i--;
	}
	i++;
	savedSharedWorldInfo.push_front(info);
	bool alreadyIn = false;
	for (auto element : latestInfo)
	{
		if (element->getId() == info->senderID)
		{
			element->setMsg(info);
			auto tmp = ros::Time::now();
			unsigned long now = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;
			element->setTimeStamp(now);
			alreadyIn = true;
		}
	}
	if (!alreadyIn)
	{
		shared_ptr<RobotInfo> robotInfo = make_shared<RobotInfo>();
		robotInfo->setId(info->senderID);
		robotInfo->setMsg(info);
		auto tmp = ros::Time::now();
		unsigned long now = (unsigned long)tmp.sec * (unsigned long)1000000000 + (unsigned long)tmp.nsec;
		robotInfo->setTimeStamp(now);
		latestInfo.push_back(robotInfo);
	}
}

list<boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> > FieldWidget3D::getSavedSharedWorldInfo()
{
	return savedSharedWorldInfo;
}

void FieldWidget3D::moveBall(double x, double y, double z)
{
	ball->SetPosition(x, y, z);
}

void FieldWidget3D::drawOpponent(double x, double y, double z)
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
	shared_ptr<RobotVisualization> robot = make_shared<RobotVisualization>();
	robot->setTop(obstacleTop);
	robot->setBottom(obstacleBottom);
	obstacles.push_front(robot);
}

void FieldWidget3D::removeObstacles(vtkRenderer* renderer)
{
	for (shared_ptr<RobotVisualization> actor : obstacles)
	{
		renderer->RemoveActor(actor->getTop());
		renderer->RemoveActor(actor->getBottom());
	}
	obstacles.clear();
}

void FieldWidget3D::drawTeamRobot(shared_ptr<RobotVisualization> robot, double x, double y, double z)
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

	vtkSmartPointer<vtkPolyDataMapper> teamBottomMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	teamBottomMapper->SetInputConnection(cubeSrc->GetOutputPort());

	vtkSmartPointer<vtkDataSetMapper> teamTopMapper = vtkSmartPointer<vtkDataSetMapper>::New();
	teamTopMapper->SetInput(ug);

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
	teamTop->GetProperty()->SetColor(1, 1, 1);
	teamTop->GetProperty()->SetDiffuse(0.4);
	teamTop->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(teamTop);

	robot->setTop(teamTop);
	robot->setBottom(teamBottom);
	team.push_front(robot);
}

void FieldWidget3D::moveRobot(shared_ptr<RobotVisualization> robot, double x, double y, double z)
{
	robot->getTop()->SetPosition(x, y, z + 0.4);
	robot->getBottom()->SetPosition(x, y, z + 0.2);
}

