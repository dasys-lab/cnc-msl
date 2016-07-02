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
#include "MainWindow.h"

//#include "ConfigXML.h"

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


//################################################################################################
//########################################## Static ##############################################
//################################################################################################

vtkSmartPointer<vtkActor> FieldWidget3D::createLine(float x1, float y1, float z1, float x2, float y2, float z2, float width, std::array<double,3> color)
{
        vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
        vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
        line->SetPoint1(x1, y1, z1);
        line->SetPoint2(x2, y2, z2);
        lineMapper->SetInputConnection(line->GetOutputPort());
        lineActor->SetMapper(lineMapper);
        lineActor->GetProperty()->SetLineWidth(width);
        lineActor->GetProperty()->SetColor(color[0], color[1], color[2]);
        lineActor->GetProperty()->SetPointSize(1);
        lineActor->GetProperty()->SetLineWidth(3);

        return lineActor;
}

void FieldWidget3D::updateLine(vtkSmartPointer<vtkActor> actor, float x1, float y1, float z1, float x2, float y2, float z2)
{
        vtkSmartPointer<vtkMapper> lineMapper = actor->GetMapper();

        vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
        line->SetPoint1(x1, y1, z1);
        line->SetPoint2(x2, y2, z2);

        lineMapper->SetInputConnection(line->GetOutputPort());
}

std::shared_ptr<Line> FieldWidget3D::createDashedLine(float x1, float y1, float z1, float x2, float y2,float z2, float width, int pattern, std::array<double,3> color)
{
        vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
        vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
        line->SetPoint1(x1, y1, z1);
        line->SetPoint2(x2, y2, z2);
        lineMapper->SetInputConnection(line->GetOutputPort());
        lineActor->SetMapper(lineMapper);
        lineActor->GetProperty()->SetLineWidth(width);
        lineActor->GetProperty()->SetLineStipplePattern(pattern);
        lineActor->GetProperty()->SetLineStippleRepeatFactor(1);
        lineActor->GetProperty()->SetColor(color[0], color[1], color[2]);
        lineActor->GetProperty()->SetPointSize(1);
        lineActor->GetProperty()->SetLineWidth(3);

        return std::make_shared<Line>(lineActor, line);
}

vtkSmartPointer<vtkActor> FieldWidget3D::createDot(float x, float y, float radius, std::array<double,3> color)
{
        vtkSmartPointer<vtkCylinderSource> dot = vtkSmartPointer<vtkCylinderSource>::New();
        dot->SetRadius(radius);
        dot->SetHeight(0.001);
        dot->SetResolution(32);
        vtkSmartPointer<vtkPolyDataMapper> dotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        dotMapper->SetInput(dot->GetOutput());

        vtkSmartPointer<vtkActor> coloredDot = vtkSmartPointer<vtkActor>::New();
        coloredDot->SetMapper(dotMapper);
        coloredDot->GetProperty()->SetColor(color[0], color[1], color[2]);
        coloredDot->SetPosition(x, y, 0.01);
        coloredDot->SetOrientation(90, 0, 0);
        coloredDot->GetProperty()->SetAmbient(1.0);
        return coloredDot;
}

vtkSmartPointer<vtkActor> FieldWidget3D::addCircle(float x, float y, float outerRadius, float innerRadius)
{
        vtkSmartPointer<vtkDiskSource> diskSource = vtkSmartPointer<vtkDiskSource>::New();
        diskSource->SetCircumferentialResolution(100);
        diskSource->SetOuterRadius(outerRadius);
        diskSource->SetInnerRadius(innerRadius);

        // Create a mapper and actor.
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(diskSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->SetPosition(x, y, 0);
        actor->GetProperty()->SetColor(1, 1, 1);
        actor->GetProperty()->SetAmbient(1);
        actor->GetProperty()->SetDiffuse(0);
        actor->GetProperty()->SetSpecular(0);

        return actor;
}

vtkSmartPointer<vtkActor> FieldWidget3D::addArc(float x, float y, float radius, float startDeg, float endDeg)
{
        vtkSmartPointer<vtkArcSource> arcSource = vtkSmartPointer<vtkArcSource>::New();
        arcSource->SetResolution(100);
        arcSource->NegativeOff();
        arcSource->SetCenter(x, y, 0);
        arcSource->SetPoint1(x + radius * cos(startDeg * M_PI / 180), y + radius * sin(startDeg * M_PI / 180), 0);
        arcSource->SetPoint2(x + radius * cos(endDeg * M_PI / 180), y + radius * sin(endDeg * M_PI / 180), 0);

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(arcSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->GetProperty()->SetColor(1, 1, 1);
        actor->GetProperty()->SetAmbient(1);
        actor->GetProperty()->SetDiffuse(0);
        actor->GetProperty()->SetSpecular(0);
        actor->SetMapper(mapper);

        return actor;
}

vtkSmartPointer<vtkActor> FieldWidget3D::createText(QString text)
{
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
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



//################################################################################################
//########################################## Stuff ###############################################
//################################################################################################

bool robotVisActive[7] = {false};
bool robotPpActive[7] = {false};
bool robotPassingActive[7] = {false};
bool robotCorrActive[7] = {false};
bool robotVoronoiActive[7] = {false};
bool robotSidesActive[7] = {false};
int robotIndex[101] = {0};
string robotNames[101] = {};
int selectedRobot = 0;

FieldWidget3D::FieldWidget3D(QWidget *parent) :
		QVTKWidget(parent)
{
	// showPath = true;
	// showVoronoiNet = false;
	// showCorridorCheck = false;
	// showSitePoints = false;
	// showPathPlannerAll = false;
	showDebugPoints = false;

	this->parent = parent;
	rosNode = new ros::NodeHandle();
//	savedSharedWorldInfo = list<boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo>>(ringBufferLength);
	sharedWorldInfoSubscriber = rosNode->subscribe("/WorldModel/SharedWorldInfo", 10, &FieldWidget3D::onSharedWorldInfo,
													(FieldWidget3D*)this);
	pathPlannerSubscriber = rosNode->subscribe("/PathPlanner/PathPlanner", 10, &FieldWidget3D::onPathPlannerMsg,
												(FieldWidget3D*)this);
	voronoiSidesSubscriber = rosNode->subscribe("/PathPlanner/VoronoiNet", 10, &FieldWidget3D::onVoronoiNetMsg,
												(FieldWidget3D*)this);
	corridorCheckSubscriber = rosNode->subscribe("/PathPlanner/CorridorCheck", 10, &FieldWidget3D::onCorridorCheckMsg,
													(FieldWidget3D*)this);
        debugMsgSubscriber = rosNode->subscribe("/DebugMsg", 10, &FieldWidget3D::onDebugMsg,  (FieldWidget3D*)this);
        passMsgSubscriber = rosNode->subscribe("/WorldModel/PassMsg", 10, &FieldWidget3D::onPassMsg,  (FieldWidget3D*)this);

        spinner = new ros::AsyncSpinner(1);
	spinner->start();
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	Update_timer = new QTimer();
	Update_timer->setInterval(1);
	connect(Update_timer, SIGNAL(timeout()), this, SLOT(update_robot_info()));
	currentField = (*sc)["FootballField"]->get<string>("FootballField", "CurrentField", NULL);
	_FIELD_LENGTH = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "FieldLength", NULL) / 1000;
	_FIELD_WIDTH = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "FieldWidth", NULL) / 1000;
	_LINE_THICKNESS = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "LineWidth", NULL) / 1000;
	_GOAL_AREA_LENGTH = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "GoalAreaXSize", NULL) / 1000;
	_GOAL_AREA_WIDTH = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "GoalAreaYSize", NULL) / 1000;
	_PENALTY_AREA_LENGTH = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "PenaltyAreaXSize", NULL) / 1000;
	_PENALTY_AREA_WIDTH = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "PenaltyAreaYSize", NULL) / 1000;
	_CENTER_CIRCLE_RADIUS = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "MiddleCircleRadius", NULL) / 1000;
	_BALL_DIAMETER = (*sc)["Rules"]->get<double>("Rules.BallRadius", NULL) * 2.0 / 1000;
	_CORNER_CIRCLE_RADIUS = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "CornerCircleRadius", NULL) / 1000;
	_PENALTY_MARK_DISTANCE = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "PenaltySpot", NULL) / 1000;
	_BLACK_POINT_WIDTH = _FIELD_WIDTH / 4.0;
	_BLACK_POINT_LENGTH = (*sc)["FootballField"]->get<double>("FootballField", currentField.c_str(), "PenaltySpot", NULL) / 1000;
	_ROBOT_RADIUS = (*sc)["Rules"]->get<double>("Rules.RobotRadius", NULL) * 2 / 1000; // this was diameter before, although the variable's name is _ROBOT_RADIUS

	renderWindow = vtkRenderWindow::New();
	renderer = vtkRenderer::New();
	renderer->SetBackground(72.0 / 255.0, 72.0 / 255.0, 72.0 / 255.0);

	renderWindow->AddRenderer(renderer);
	this->SetRenderWindow(renderWindow);

	drawField(renderer);
	drawGoals(renderer);

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
	iren->SetInteractorStyle(intStyle);
	renderWindow->SetInteractor(iren);
	// WIPFIX renderer->Render();

	/* Read CAMBADA model */
	vtkSmartPointer<vtkOBJReader> readerCbd = vtkSmartPointer<vtkOBJReader>::New();
	readerCbd->SetFileName("../config/3DModels/cambada_base.obj");

	vtkSmartPointer<vtkPolyDataMapper> actorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actorMapper->SetInput(readerCbd->GetOutput());

	// WIPFIX renderer->Render();

	// View heightmap
	heightVisible = false;
	heightColor = true;
	height3D = false;
	heightActor = NULL;

	lockCam = false;
	top = false;

	Update_timer->start(33);

}

pair<double, double> FieldWidget3D::transformToGuiCoords(double x, double y)
{
        pair<double, double> ret;
        ret.first = y / 1000;
        ret.second = -x / 1000;
        return ret;
}

void FieldWidget3D::update_robot_info(void)
{
	lock_guard<mutex> lock(swmMutex);

	for (auto robot : robots)
	{
		int myId = robot->getId();
		int selectedIndex = mainWindow->robotSelector->currentIndex();

		// detect change on robot selector
		if (selectedRobot != selectedIndex)
		{
			selectedRobot = selectedIndex;
			if (selectedIndex == 0) // if selector is changed to ALL
			{
				robotVisActive[0] = true;
				robotPpActive[0] = true;
				robotPassingActive[0] = true;
				robotCorrActive[0] = true;
				robotVoronoiActive[0] = true;
				robotSidesActive[0] = true;
				for (int i=1;i<mainWindow->robotSelector->count();i++)
				{
					if (!robotVisActive[i]) robotVisActive[0] = false;
					if (!robotPpActive[i]) robotPpActive[0] = false;
					if (!robotPassingActive[i]) robotPassingActive[0] = false;
					if (!robotCorrActive[i]) robotCorrActive[0] = false;
					if (!robotVoronoiActive[i]) robotVoronoiActive[0] = false;
					if (!robotSidesActive[i]) robotSidesActive[0] = false;
				}
			}

			// adjust the checkboxes accordingly
			mainWindow->checkVis->setChecked(robotVisActive[selectedIndex]);
			mainWindow->checkPp->setChecked(robotPpActive[selectedIndex]);
			mainWindow->checkPassing->setChecked(robotPassingActive[selectedIndex]);
			mainWindow->checkCorr->setChecked(robotCorrActive[selectedIndex]);
			mainWindow->checkVoronoi->setChecked(robotVoronoiActive[selectedIndex]);
			mainWindow->checkSides->setChecked(robotSidesActive[selectedIndex]);
		}

		// detect change on visible checkbox
		bool visCheckBoxState = mainWindow->checkVis->checkState();
		if (robotVisActive[selectedIndex] != visCheckBoxState)
		{
			if (selectedIndex == 0) // all robots' visualization checkboxes are changed
			{
				for (int i=0;i<7;i++) robotVisActive[i] = visCheckBoxState;
			} else // only one robot's visualization checkbox is changed
			robotVisActive[selectedIndex] = visCheckBoxState;

			//deactivating visible caused pathplanner deactivated
			if (!robotVisActive[selectedIndex])
			{
				robotPpActive[selectedIndex] = false;
				robotPassingActive[selectedIndex] = false;
				robotCorrActive[selectedIndex] = false;
				robotVoronoiActive[selectedIndex] = false;
				robotSidesActive[selectedIndex] = false;

				// turn off all checkboxes
				mainWindow->checkVis->setChecked(false);
				mainWindow->checkPp->setChecked(false);
				mainWindow->checkPassing->setChecked(false);
				mainWindow->checkCorr->setChecked(false);
				mainWindow->checkVoronoi->setChecked(false);
				mainWindow->checkSides->setChecked(false);
			}
		}

		// detect change on pathplanner checkbox
		bool ppCheckBoxState = mainWindow->checkPp->checkState();
		if (robotPpActive[selectedIndex] != ppCheckBoxState)
		{
			if (selectedIndex == 0) // all robots' pathplanner checkboxes are changed
			{
				for (int i=0;i<7;i++) robotPpActive[i] = ppCheckBoxState;
			} else // only one robot's pathplanner checkbox is changed
				robotPpActive[selectedIndex] = ppCheckBoxState;
		}

		// detect change on passing checkbox
		bool passingCheckBoxState = mainWindow->checkPassing->checkState();
		if (robotPassingActive[selectedIndex] != passingCheckBoxState)
		{
			if (selectedIndex == 0) // all robots' passing checkboxes are changed
			{
				for (int i=0;i<7;i++) robotPassingActive[i] = passingCheckBoxState;
			} else // only one robot's passing checkbox is changed
				robotPassingActive[selectedIndex] = passingCheckBoxState;
		}

		// detect change on corridor checkbox
		bool corrCheckBoxState = mainWindow->checkCorr->checkState();
		if (robotCorrActive[selectedIndex] != corrCheckBoxState)
		{
			if (selectedIndex == 0) // all robots' corridor checkboxes are changed
			{
				for (int i=0;i<7;i++) robotCorrActive[i] = corrCheckBoxState;
			} else // only one robot's corridor checkbox is changed
				robotCorrActive[selectedIndex] = corrCheckBoxState;
		}

		// detect change on voronoi checkbox
		bool voronoiCheckBoxState = mainWindow->checkVoronoi->checkState();
		if (robotVoronoiActive[selectedIndex] != voronoiCheckBoxState)
		{
			if (selectedIndex == 0) // all robots' voronoi checkboxes are changed
			{
				for (int i=0;i<7;i++) robotVoronoiActive[i] = voronoiCheckBoxState;
			} else // only one robot's voronoi checkbox is changed
				robotVoronoiActive[selectedIndex] = voronoiCheckBoxState;
		}

		// detect change on sides checkbox
		bool sidesCheckBoxState = mainWindow->checkSides->checkState();
		if (robotSidesActive[selectedIndex] != sidesCheckBoxState)
		{
			if (selectedIndex == 0) // all robots' sides checkboxes are changed
			{
				for (int i=0;i<7;i++) robotSidesActive[i] = sidesCheckBoxState;
			} else // only one robot's sides checkbox is changed
				robotSidesActive[selectedIndex] = sidesCheckBoxState;
		}

		if (!robotVisActive[robotIndex[myId]]) robot->setVisStatus(false);
			else robot->setVisStatus(true);

		// test for inactivated robot
        if (robot->isTimeout()) // || !robotVisActive[robotIndex[myId]])
		{
		        robot->getVisualization()->remove(this->renderer);
                        continue;
		}

        robot->getVisualization()->updatePathPlannerDebug(this->renderer, robotPpActive[robotIndex[myId]]);
       	robot->getVisualization()->updateCorridorDebug(this->renderer, robotCorrActive[robotIndex[myId]]);
        robot->getVisualization()->updateVoronoiNetDebug(this->renderer, robotVoronoiActive[robotIndex[myId]], robotSidesActive[robotIndex[myId]]);
        robot->getVisualization()->updatePassMsg(this->renderer, robotPassingActive[robotIndex[myId]]);

        robot->getVisualization()->updatePosition(this->renderer);
        robot->getVisualization()->updateBall(this->renderer);
        robot->getVisualization()->updateSharedBall(this->renderer);
        robot->getVisualization()->updateObjects(this->renderer);
        robot->getVisualization()->updateDebugPoints(this->renderer, this->showDebugPoints);

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

//void FieldWidget3D::obstacles_point_flip(unsigned int Robot_no, bool on_off)
//{
//	option_draw_obstacles[Robot_no] = on_off;
//}
//
//void FieldWidget3D::obstacles_point_flip_all(bool on_off)
//{
//}
//
//void FieldWidget3D::debug_point_flip(unsigned int Robot_no, bool on_off)
//{
//	option_draw_debug[Robot_no] = on_off;
//}

void FieldWidget3D::showDebugPointsToggle()
{
        if (this->showDebugPoints == false)
        {
                showDebugPoints = true;
        }
        else
        {
                showDebugPoints = false;
        }

        mainWindow->robotSelector->setCurrentIndex(0);
        mainWindow->checkVis->setChecked(true);
		mainWindow->checkPp->setChecked(true);
		mainWindow->checkPassing->setChecked(true);
		mainWindow->checkCorr->setChecked(true);
		mainWindow->checkVoronoi->setChecked(true);
		mainWindow->checkSides->setChecked(true);
//        this->updatePathPlannerAll();
}

/*void FieldWidget3D::showPathToggle()
{
	if (this->showPath == false)
	{
		showPath = true;
	}
	else
	{
		showPath = false;
	}

        this->updatePathPlannerAll();
}

void FieldWidget3D::showVoronoiNetToggle(void)
{
	if (this->showVoronoiNet == false)
	{
		showVoronoiNet = true;
	}
	else
	{
		showVoronoiNet = false;
	}

        this->updatePathPlannerAll();
}

void FieldWidget3D::showCorridorCheckToggle(void)
{
	if (this->showCorridorCheck == false)
	{
		showCorridorCheck = true;
	}
	else
	{
		showCorridorCheck = false;
	}

        this->updatePathPlannerAll();
}

void FieldWidget3D::showSitePointsToggle(void)
{
	if (this->showSitePoints == false)
	{
		showSitePoints = true;
	}
	else
	{
		showSitePoints = false;
	}

        this->updatePathPlannerAll();
}

void FieldWidget3D::updatePathPlannerAll(void)
{
        if (showPathPlannerAll && showCorridorCheck && showPath && showSitePoints && showVoronoiNet)
        {
                this->mainWindow->actionShow_All_PathPlanner_Components->setChecked(true);
        }
        else
        {
                this->mainWindow->actionShow_All_PathPlanner_Components->setChecked(false);
        }
}

void FieldWidget3D::showPathPlannerAllToggle(void)
{
	if (this->showPathPlannerAll == false)
	{
		showPathPlannerAll = true;
		showCorridorCheck = true;
		showPath = true;
		showSitePoints = true;
		showVoronoiNet = true;

		this->mainWindow->actionShow_All_PathPlanner_Components->setChecked(true);
                this->mainWindow->actionShow_Corridor_Check->setChecked(true);
                this->mainWindow->actionShow_PathPlanner_Path->setChecked(true);
                this->mainWindow->actionShow_Sides->setChecked(true);
                this->mainWindow->actionShow_Voronoi_Diagram->setChecked(true);
	}
	else
	{
		showPathPlannerAll = false;
		showCorridorCheck = false;
		showPath = false;
		showSitePoints = false;
		showVoronoiNet = false;

                this->mainWindow->actionShow_All_PathPlanner_Components->setChecked(false);
                this->mainWindow->actionShow_Corridor_Check->setChecked(false);
                this->mainWindow->actionShow_PathPlanner_Path->setChecked(false);
                this->mainWindow->actionShow_Sides->setChecked(false);
                this->mainWindow->actionShow_Voronoi_Diagram->setChecked(false);
	}
}

*/

//void FieldWidget3D::debug_point_flip_all(bool on_off)
//{
//}

void FieldWidget3D::drawGoals(vtkRenderer* renderer)
{
	double post = 0.125;

	vtkSmartPointer<vtkCubeSource> cubeSrc = vtkSmartPointer<vtkCubeSource>::New();
	cubeSrc->SetXLength(post);
	cubeSrc->SetYLength(post);
	cubeSrc->SetZLength(1);

	vtkSmartPointer<vtkCubeSource> cubeSrc2 = vtkSmartPointer<vtkCubeSource>::New();
	cubeSrc2->SetXLength(2 + 2 * post);
	cubeSrc2->SetYLength(post);
	cubeSrc2->SetZLength(post);

	vtkSmartPointer<vtkPolyDataMapper> goalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	goalMapper->SetInput(cubeSrc->GetOutput());

	vtkSmartPointer<vtkPolyDataMapper> goalMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	goalMapper2->SetInput(cubeSrc2->GetOutput());

	vtkSmartPointer<vtkActor> goalBlue = vtkSmartPointer<vtkActor>::New();
	goalBlue->SetMapper(goalMapper);
	goalBlue->SetPosition(0 - 1 - post / 2, -_FIELD_LENGTH / 2 - post / 2, 0.5);
	goalBlue->GetProperty()->SetColor(0.2, 0.2, 1);
	goalBlue->GetProperty()->SetDiffuse(0.4);
	goalBlue->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalBlue);

	vtkSmartPointer<vtkActor> goalBlue2 = vtkSmartPointer<vtkActor>::New();
	goalBlue2->SetMapper(goalMapper);
	goalBlue2->SetPosition(0 + 1 + post / 2, -_FIELD_LENGTH / 2 - post / 2, 0.5);
	goalBlue2->GetProperty()->SetColor(0.2, 0.2, 1);
	goalBlue2->GetProperty()->SetDiffuse(0.4);
	goalBlue2->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalBlue2);

	vtkSmartPointer<vtkActor> goalBlue3 = vtkSmartPointer<vtkActor>::New();
	goalBlue3->SetMapper(goalMapper2);
	goalBlue3->SetPosition(0, -_FIELD_LENGTH / 2 - post / 2, 1);
	goalBlue3->GetProperty()->SetColor(0.2, 0.2, 1);
	goalBlue3->GetProperty()->SetDiffuse(0.4);
	goalBlue3->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalBlue3);

	vtkSmartPointer<vtkActor> goalYellow = vtkSmartPointer<vtkActor>::New();
	goalYellow->SetMapper(goalMapper);
	goalYellow->SetPosition(0 - 1 - post / 2, _FIELD_LENGTH / 2 + post / 2, 0.5);
	goalYellow->GetProperty()->SetColor(1, 1, 0.2);
	goalYellow->GetProperty()->SetDiffuse(0.4);
	goalYellow->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalYellow);

	vtkSmartPointer<vtkActor> goalYellow2 = vtkSmartPointer<vtkActor>::New();
	goalYellow2->SetMapper(goalMapper);
	goalYellow2->SetPosition(0 + 1 + post / 2, _FIELD_LENGTH / 2 + post / 2, 0.5);
	goalYellow2->GetProperty()->SetColor(1, 1, 0.2);
	goalYellow2->GetProperty()->SetDiffuse(0.4);
	goalYellow2->GetProperty()->SetAmbient(0.8);
	renderer->AddActor(goalYellow2);

	vtkSmartPointer<vtkActor> goalYellow3 = vtkSmartPointer<vtkActor>::New();
	goalYellow3->SetMapper(goalMapper2);
	goalYellow3->SetPosition(0, _FIELD_LENGTH / 2 + post / 2, 1);
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
	planeSrc->SetPoint1(_FIELD_WIDTH + 2.0, 0, 0);
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

	float lineOffSet = _LINE_THICKNESS / 2;

	// Draw Field
	// middle line
	drawFieldLine(renderer, -_FIELD_WIDTH / 2, -lineOffSet, 0.0, _FIELD_WIDTH / 2, +lineOffSet, 0.0);

	// left side line
	drawFieldLine(renderer, (-_FIELD_WIDTH / 2) - lineOffSet, -_FIELD_LENGTH / 2 - lineOffSet, 0.0,
				(-_FIELD_WIDTH / 2) + lineOffSet, _FIELD_LENGTH / 2 + lineOffSet, 0.0);

	// right side line
	drawFieldLine(renderer, (_FIELD_WIDTH / 2) - lineOffSet, -_FIELD_LENGTH / 2 - lineOffSet, 0.0,
				(_FIELD_WIDTH / 2) + lineOffSet, _FIELD_LENGTH / 2 + lineOffSet, 0.0);

	// enemy goal line
	drawFieldLine(renderer, -_FIELD_WIDTH / 2, (_FIELD_LENGTH / 2) - lineOffSet, 0.0, _FIELD_WIDTH / 2,
				(_FIELD_LENGTH / 2) + lineOffSet, 0.0);

	// own goal line
	drawFieldLine(renderer, -_FIELD_WIDTH / 2, (-_FIELD_LENGTH / 2) - lineOffSet, 0.0, _FIELD_WIDTH / 2,
				(-_FIELD_LENGTH / 2) + lineOffSet, 0.0);


	// Goal Areas (1. own, 2. opponent)
	// long goal area line
	drawFieldLine(renderer, -_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH - lineOffSet, 0.0,
				_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH + lineOffSet, 0.0);

	// left short goal area line
	drawFieldLine(renderer, -_GOAL_AREA_WIDTH / 2 - lineOffSet, -_FIELD_LENGTH / 2, 0.0,
				-_GOAL_AREA_WIDTH / 2 + lineOffSet, -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH + lineOffSet, 0.0);

	// right short goal area line
	drawFieldLine(renderer, _GOAL_AREA_WIDTH / 2 - lineOffSet, -_FIELD_LENGTH / 2, 0.0, _GOAL_AREA_WIDTH / 2 + lineOffSet,
				-_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH + lineOffSet, 0.0);

	// long goal area line
	drawFieldLine(renderer, -_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH - lineOffSet, 0.0,
				_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH + lineOffSet, 0.0);

	// left short goal area line
	drawFieldLine(renderer, -_GOAL_AREA_WIDTH / 2 - lineOffSet, _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH - lineOffSet, 0.0,
				-_GOAL_AREA_WIDTH / 2 + lineOffSet, _FIELD_LENGTH / 2, 0.0);

	// right short goal area line
	drawFieldLine(renderer, _GOAL_AREA_WIDTH / 2 - lineOffSet, _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH - lineOffSet, 0.0,
				_GOAL_AREA_WIDTH / 2 + lineOffSet, _FIELD_LENGTH / 2, 0.0);

	// Penalty Areas (1. opponent, 2. own)
	drawFieldLine(renderer, -_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH - lineOffSet, 0.0,
				_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH + lineOffSet, 0.0);

	drawFieldLine(renderer, -_PENALTY_AREA_WIDTH / 2 - lineOffSet, -_FIELD_LENGTH / 2, 0.0,
				-_PENALTY_AREA_WIDTH / 2 + lineOffSet, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH + lineOffSet, 0.0);

	drawFieldLine(renderer, _PENALTY_AREA_WIDTH / 2 - lineOffSet, -_FIELD_LENGTH / 2, 0.0,
				_PENALTY_AREA_WIDTH / 2 + lineOffSet, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH + lineOffSet, 0.0);

	drawFieldLine(renderer, -_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH - lineOffSet, 0.0,
				_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH + lineOffSet, 0.0);

	drawFieldLine(renderer, -_PENALTY_AREA_WIDTH / 2 - lineOffSet, _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH - lineOffSet,
				0.0, -_PENALTY_AREA_WIDTH / 2 + lineOffSet, _FIELD_LENGTH / 2, 0.0);

	drawFieldLine(renderer, _PENALTY_AREA_WIDTH / 2 - lineOffSet, _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH - lineOffSet,
				0.0, _PENALTY_AREA_WIDTH / 2 + lineOffSet, _FIELD_LENGTH / 2, 0.0);

	// Corner Arcs
	auto arc = FieldWidget3D::addArc(-_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 0, 90);
        renderer->AddActor(arc);
	arc = FieldWidget3D::addArc(-_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 0, 270);
        renderer->AddActor(arc);
	arc = FieldWidget3D::addArc(_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 180, 90);
        renderer->AddActor(arc);
	arc = FieldWidget3D::addArc(_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 180, 270);
        renderer->AddActor(arc);

	// Center Circle
        float outerRadius = _CENTER_CIRCLE_RADIUS + _LINE_THICKNESS / 2;
        float innerRadius = _CENTER_CIRCLE_RADIUS - _LINE_THICKNESS / 2;
	auto circle = FieldWidget3D::addCircle(0, 0, outerRadius, innerRadius);
	renderer->AddActor(circle);

	// Black Dots
	auto dot = FieldWidget3D::createDot(_FIELD_WIDTH / 4, 0, lineOffSet, {0.0,0.0,0.0});
	renderer->AddActor(dot);
	dot = FieldWidget3D::createDot(-_FIELD_WIDTH / 4, 0, lineOffSet, {0.0,0.0,0.0});
        renderer->AddActor(dot);

	dot = FieldWidget3D::createDot(_FIELD_WIDTH / 4, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, lineOffSet, {0.0,0.0,0.0});
        renderer->AddActor(dot);
	dot = FieldWidget3D::createDot(-_FIELD_WIDTH / 4, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, lineOffSet, {0.0,0.0,0.0});
        renderer->AddActor(dot);
	dot = FieldWidget3D::createDot(0, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, lineOffSet);
        renderer->AddActor(dot);

	dot = FieldWidget3D::createDot(_FIELD_WIDTH / 4, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, lineOffSet, {0.0,0.0,0.0});
        renderer->AddActor(dot);
	dot = FieldWidget3D::createDot(-_FIELD_WIDTH / 4, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, lineOffSet, {0.0,0.0,0.0});
        renderer->AddActor(dot);
	dot = FieldWidget3D::createDot(0, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, lineOffSet);
        renderer->AddActor(dot);

	dot = FieldWidget3D::createDot(0, 0, lineOffSet);
        renderer->AddActor(dot);
}

void FieldWidget3D::drawFieldLine(vtkRenderer* renderer, float x1, float y1, float z1, float x2, float y2, float z2)
{
          vtkSmartPointer<vtkPlaneSource> planeSrc = vtkSmartPointer<vtkPlaneSource>::New();
          planeSrc->SetOrigin(0, 0, 0);
          planeSrc->SetPoint1(abs(x2 - x1), 0, 0);
          planeSrc->SetPoint2(0, abs(y2 - y1), 0);
          vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
          planeMapper->SetInput(planeSrc->GetOutput());
          vtkSmartPointer<vtkActor> lineActor = vtkActor::New();
          lineActor->SetMapper(planeMapper);
          lineActor->GetProperty()->SetColor(1, 1, 1);
          lineActor->SetPosition(x1, y1, 0);
          lineActor->GetProperty()->SetAmbient(1);
          lineActor->GetProperty()->SetDiffuse(0);
          lineActor->GetProperty()->SetSpecular(0);

          renderer->AddActor(lineActor);
}

vtkSmartPointer<vtkActor> FieldWidget3D::createObstacle()
{
	// Obstacle actors
	vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
	cylinder->SetRadius(0.25);
	cylinder->SetHeight(OBSTACLE_HEIGHT);
	cylinder->SetResolution(12);
	vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	cylinderMapper->SetInput(cylinder->GetOutput());

	vtkSmartPointer<vtkActor> obstacleActor = vtkSmartPointer<vtkActor>::New();
	obstacleActor->SetMapper(cylinderMapper);
	obstacleActor->GetProperty()->SetColor(0, 0, 0);
	//obstacleActor->GetProperty()->SetRepresentationToWireframe();
	obstacleActor->RotateX(90); // Rotate 90 degrees in XX axis

	return obstacleActor;
}

vtkSmartPointer<vtkActor> FieldWidget3D::createDebugPt()
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

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
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
	double xx, yy, zz;
	double minz = 2010.0;
	double maxz = -2010.0;
	int i = 0;
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

list<shared_ptr<RobotInfo>>* FieldWidget3D::getRobots()
{
        return &this->robots;
}

void FieldWidget3D::lock(bool lock)
{
	this->lockCam = lock;
}

std::shared_ptr<RobotInfo> FieldWidget3D::getRobotById(int id)
{
        for (auto element : robots)
        {
                if (element->getId() == id)
                {
                        return element;
                }

        }

        shared_ptr<RobotInfo> robot = make_shared<RobotInfo>(this);
        robot->setId(id);
        robots.push_back(robot);

        robot->getVisualization()->init(this->renderer, id);

        int robotCount = mainWindow->robotSelector->count();
        if (robotCount == 0)
        {
        	robotNames[0]= "ALL";
        	robotNames[1]= "Mops";
        	robotNames[8]= "Hairy";
        	robotNames[9]= "Nase";
        	robotNames[10]= "Savvy";
        	robotNames[11]= "Myo";
        	robotNames[100]= "Brain";

        	mainWindow->robotSelector->addItem(QString::fromStdString(robotNames[0]), 0);
        	mainWindow->robotSelector->setCurrentIndex(0);
        	robotVisActive[0] = true;
			mainWindow->checkVis->setChecked(true);
        	robotIndex[0] = 0;
            robotCount++;
        }

        robotIndex[id] = robotCount;
        string robotName = robotNames[id];
        QString robotStr = QString::fromStdString(robotName+" ("+boost::lexical_cast<std::string>(id)+")");
        mainWindow->robotSelector->addItem(robotStr, id);
        robot->setVisStatus(true);
		robotVisActive[robotCount] = true;

        return robot;
}

//################################################################################################
//########################################## OnMsg ###############################################
//################################################################################################

void FieldWidget3D::onPathPlannerMsg(boost::shared_ptr<msl_msgs::PathPlanner> info)
{
        lock_guard<mutex> lock(pathMutex);

        auto robot = this->getRobotById(info->senderId);
        robot->setPathPlannerInfo(info);
//        robot->updateTimeStamp();
}

void FieldWidget3D::onSharedWorldInfo(boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> info)
{
        lock_guard<mutex> lock(swmMutex);

        auto robot = this->getRobotById(info->senderID);

        robot->setSharedWorldInfo(info);
        robot->updateTimeStamp();
}

void FieldWidget3D::onVoronoiNetMsg(boost::shared_ptr<msl_msgs::VoronoiNetInfo> info)
{
        lock_guard<mutex> lock(voronoiMutex);

        auto robot = this->getRobotById(info->senderId);
        robot->setVoronoiNetInfo(info);
//        robot->updateTimeStamp();
}

void FieldWidget3D::onCorridorCheckMsg(boost::shared_ptr<msl_msgs::CorridorCheck> info)
{
        lock_guard<mutex> lock(corridorMutex);

        auto robot = this->getRobotById(info->senderId);
        robot->setCorridorCheckInfo(info);
//        robot->updateTimeStamp();
}

void FieldWidget3D::onDebugMsg(boost::shared_ptr<msl_helper_msgs::DebugMsg> info)
{
        lock_guard<mutex> lock(debugMutex);

        auto robot = this->getRobotById(info->senderID);
        robot->addDebugMsg(info);
//        robot->updateTimeStamp();
}

void FieldWidget3D::onPassMsg(boost::shared_ptr<msl_helper_msgs::PassMsg> info)
{
        lock_guard<mutex> lock(debugMutex);

        auto robot = this->getRobotById(info->senderID);
        robot->setPassMsg(info);
//        robot->updateTimeStamp();
}
