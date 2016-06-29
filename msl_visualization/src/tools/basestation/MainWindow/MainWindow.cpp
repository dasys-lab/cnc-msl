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

#include "MainWindow.h"

MWind *wind;

MWind::MWind(QMainWindow *parent)
{
	/* Criação da janela */
	setupUi(parent);

	//FieldW->updateField();
	FieldW->mainWindow = this;

	/* Pintar as letras dos group em branco */
	QColor color = QColor::fromRgb(255, 255, 255, 255);
	QPalette plt;

	plt.setColor(QPalette::Foreground, color);

	Group_RobotFilter->setPalette(plt);
	Group_RB->setPalette(plt);

	fullinfowindow = new QMainWindow;

	/* inicialização das variáveis */
	mwind = parent;
	fullscreenflag = 0;

	QString str;
	QString rm = "Role";

	/* Inicializar o Timer de update do game time*/
	UpdateTimer = new QTimer();

	/* Conectar as funções da barra de menus */
	connect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
	connect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
	connect(actionConnect, SIGNAL(triggered()), RefBoxWG, SLOT(detailsBotPressed()));
	connect(actionLock, SIGNAL(toggled(bool)), FieldW, SLOT(lock(bool)));
	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateGameTime()));
	connect(RefBoxWG, SIGNAL(changeGoalColor (int)), this, SLOT(GoalColorChanged(int)));
	connect(RefBoxWG, SIGNAL(UpdateGameParameter_signal()), this, SLOT(UpdateGameParameters()));

	// Debug
	connect(actionDebug_All_On_Off, SIGNAL(triggered()), FieldW, SLOT(showDebugPointsToggle()));

	// Path planner
//	connect(actionShow_PathPlanner_Path, SIGNAL(triggered()), FieldW, SLOT(showPathToggle()));
//	connect(actionShow_Corridor_Check, SIGNAL(triggered()), FieldW, SLOT(showCorridorCheckToggle()));
//	connect(actionShow_Voronoi_Diagram, SIGNAL(triggered()), FieldW, SLOT(showVoronoiNetToggle()));
//	connect(actionShow_Sites, SIGNAL(triggered()), FieldW, SLOT(showSitePointsToggle()));
//	connect(actionShow_All_PathPlanner_Components, SIGNAL(triggered()), FieldW, SLOT(showPathPlannerAllToggle()));

	// Robot filtering


	/* instalar o filtro de eventos */
	parent->installEventFilter(this);

	TeamColorChanged(0);
	RobotVisChanged(0, 1);
	GoalColorChanged(1);

	UpdateTimer->start(10);

	QTime GTime;
	GTime.setHMS(0, 0, 0);
	Game_time_clock->setText(GTime.toString("mm:ss"));

	/* Formation */
	// no fomration file
//	QFile file("../config/formation.conf");
//	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
//	{
//		int nFormations = 0;
//		QTextStream in(&file);
//
//		while (!in.atEnd())
//		{
//			QString line = in.readLine();
//			if (line.contains("FORMATIONDT"))
//			{
//				line.remove("FORMATIONDT");
//				line = line.trimmed();
//			}
//			else if (line.contains("FORMATION"))
//			{
//				line.remove("FORMATION");
//			}
//		}
//
//	}
//	else
//	{
//		printf("Error Opening the Formation File\n");
//	}

	connect(actionVisible, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMapVisible(bool)));
	connect(action3D, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMap3D(bool)));
	connect(actionColor, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMapColor(bool)));
}

MWind::~MWind()
{
	// Disconnect
	disconnect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
	disconnect(actionConnect, SIGNAL(triggered()), RefBoxWG, SLOT(detailsBotPressed()));
//	disconnect(actionShow_PathPlanner_Path, SIGNAL(triggered()), FieldW, SLOT(showPathToggle()));
//	disconnect(actionShow_Corridor_Check, SIGNAL(triggered()), FieldW, SLOT(showCorridorCheckToggle()));
//	disconnect(actionShow_Voronoi_Diagram, SIGNAL(triggered()), FieldW, SLOT(showVoronoiNetToggle()));
//	disconnect(actionShow_Sites, SIGNAL(triggered()), FieldW, SLOT(showSitePointsToggle()));
//	disconnect(actionShow_All_PathPlanner_Components, SIGNAL(triggered()), FieldW, SLOT(showPathPlannerAllToggle()));

	//Delete
	if (UpdateTimer != NULL)
		delete UpdateTimer;
	UpdateTimer = NULL;
	if (fullinfowindow != NULL)
		delete fullinfowindow;
	fullinfowindow = NULL;

	if (this->RefBoxWG != NULL)
		delete this->RefBoxWG;
	this->RefBoxWG = NULL;
}

void MWind::TeamColorChanged(int team)
{
	QColor Mag = QColor::fromRgb(222, 111, 161, 255); //Qt::magenta;//
	QColor Cy = QColor::fromRgb(128, 160, 191, 255);
	QPalette plt;

	if (team == 0)
	{
		plt.setColor(QPalette::Button, Mag);
	}
	else
	{
		plt.setColor(QPalette::Button, Cy);
	}


}

void MWind::RobotVisChanged(int team, int robot_id)
{
	QColor Mag = QColor::fromRgb(222, 111, 161, 255); //Qt::magenta;//
	QColor Cy = QColor::fromRgb(128, 160, 191, 255);
	QPalette plt;

/*	for (int robots = 0; robots < robotCount; robots++)
	{
		plt.setColor(QPalette::Button, robots);
	}

	RobotVisCombo->setPalette(plt);
*/
}

void MWind::GoalColorChanged(int goal)
{

	QColor Yell = QColor::fromRgb(255, 191, 105, 255);
	QColor Bl = QColor::fromRgb(0, 180, 247, 255); //Qt::blue;//
	QPalette plt;

	if (goal == 0)
	{
		plt.setColor(QPalette::Button, Yell);
	}
	else
	{
		plt.setColor(QPalette::Button, Bl);
	}

}

bool MWind::eventFilter(QObject *obj, QEvent *event)
{
	if (event->type() == QEvent::KeyPress)
	{

		/* Shortcuts da basestation */
		QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

		if (keyEvent->key() == Qt::Key_F1)
                {
                }

                if (keyEvent->key() == Qt::Key_F2)
                {
                }

                if (keyEvent->key() == Qt::Key_F3)
                {
                }

                if (keyEvent->key() == Qt::Key_F4)
                {
                }

                if (keyEvent->key() == Qt::Key_F5)
                {
                }

                if (keyEvent->key() == Qt::Key_F6)
                {
                }

                if (keyEvent->key() == Qt::Key_Escape)
                {
                }

		if (keyEvent->key() == Qt::Key_F12)
		{

		}

		if (keyEvent->key() == Qt::Key_U)
		{
		}

		if (keyEvent->key() == Qt::Key_C)
		{
		}

		if (keyEvent->key() == Qt::Key_F)
		{
		}

		if (keyEvent->key() == Qt::Key_O)
		{
		}

	}

	if ((obj == mwind) && (event->type() == QEvent::Resize))
	{
		QSize s = mwind->size();

		/* Janela Info */
		char size_inc = 0;
		if (s.rwidth() > 1263)
			size_inc = 5;
		else if (s.rwidth() > 1248)
			size_inc = 4;
		else if (s.rwidth() > 1194)
			size_inc = 3;
		else if (s.rwidth() > 1059)
			size_inc = 2;
		else if (s.rwidth() > 993)
			size_inc = 1;

		size_inc = 3;

		/* Janela principal */
		size_inc = 0;
		if (s.rwidth() > 1278)
			size_inc = 3;
		else if (s.rwidth() > 1158)
			size_inc = 2;
		else if (s.rwidth() > 1086)
			size_inc = 1;

		size_inc = 3;

		// Deal with Logo
		//printf("cenas %d\n",s.rheight());

		return true;
	}

	return false;
}

void MWind::UpdateGameTime(void)
{
	QTime GTime;
	int min = 0, sec = 0;
}

void MWind::UpdateGameParameters(void)
{

}

