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
	rosNode = new ros::NodeHandle();
	sharedWorldInfoSubscriber = rosNode->subscribe("/SharedWorldInfo", 10, &MWind::handleSharedWorldInfo, (MWind*)this);
	/* Criação da janela */
	setupUi( parent );
	
	//FieldW->updateField();

	/* Pintar as letras dos group em branco */
	QColor color = QColor::fromRgb(255,255,255,255);
	QPalette plt;

	plt.setColor(QPalette::Foreground, color);

	Group_T->setPalette(plt); 
	Group_RB->setPalette(plt); 

	fullinfowindow = new QMainWindow;
	FIW = new FInfoWind(fullinfowindow);

	/* inicialização das variáveis */
	mwind = parent;
	fullscreenflag = 0;


	GoalColorCombo->setCurrentIndex(1);

	QString str;
	QString rm = "Role";
	for (int i=1; i<num_roles;i++)
	{
		str=role_names[i];
		
		if(i!=0) str.remove(rm);
	}

	/* Inicializar o Timer de update do game time*/
	UpdateTimer = new QTimer();

	/* Conectar as funções da barra de menus */
    connect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
	connect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
	connect(actionConnect, SIGNAL(triggered()), RefBoxWG, SLOT(detailsBotPressed()));
	connect(actionFull_Screen, SIGNAL(triggered()), this, SLOT(changeWindowFullScreenMode()));
    connect(actionView_Full_Screen_info, SIGNAL(triggered()), this, SLOT(showFullScreenInfoWindow()));
	connect(TeamColorCombo, SIGNAL(activated ( int)), this, SLOT(TeamColorChanged(int)));
	connect(GoalColorCombo, SIGNAL(activated ( int)), this, SLOT(GoalColorChanged(int)));

	//Obstacles
	
	
    connect(actionTop_View, SIGNAL(toggled(bool)), this, SLOT(on_actionTop_View_toggled(bool)));
    connect(actionLock, SIGNAL(toggled(bool)), FieldW, SLOT(lock(bool)));


	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateGameTime()));
	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(transmitCoach()));

	connect(RefBoxWG, SIGNAL(changeGoalColor (int)), this, SLOT(GoalColorChanged(int)));

	connect(RefBoxWG, SIGNAL(UpdateGameParameter_signal()), this, SLOT(UpdateGameParameters()));

    // Arranque em auto-formation

	/* instalar o filtro de eventos */
	parent->installEventFilter(this);

	TeamColorChanged( 0 );
	GoalColorChanged( 1 );
	
    UpdateTimer->start(10);

	QTime GTime;
	GTime.setHMS(0,0,0);
	Game_time_clock->setText(GTime.toString("mm:ss"));

	/* Inicializar o Logo */


	/* Formation */
	QFile file("../config/formation.conf");
	if( file.open(QIODevice::ReadOnly | QIODevice::Text) )
	{
		int nFormations = 0;
		QTextStream in(&file);
		
		while( !in.atEnd() ) 
		{
			QString line = in.readLine();
			if( line.contains("FORMATIONDT") )
			{
				line.remove("FORMATIONDT");
                                line = line.trimmed();
			}
			else if( line.contains("FORMATION") )
			{
				line.remove("FORMATION");
			}
		}

    }
	else
	{
		printf("Error Opening the Formation File\n");
	}

    connect(actionVisible, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMapVisible(bool)));
    connect(action3D, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMap3D(bool)));
    connect(actionColor, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMapColor(bool)));

}



MWind::~MWind()
{
	// Disconnect
    disconnect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
    disconnect(actionDebug_Points, SIGNAL(triggered()), FieldW, SLOT(debug_point_flip()));
	disconnect(actionConnect, SIGNAL(triggered()), RefBoxWG, SLOT(detailsBotPressed()));
	disconnect(actionFull_Screen,  SIGNAL(triggered()), this, SLOT(changeWindowFullScreenMode()));
    disconnect(actionView_Full_Screen_info, SIGNAL(triggered()), this, SLOT(showFullScreenInfoWindow()));
	disconnect(TeamColorCombo, SIGNAL(activated ( int)), this, SLOT(TeamColorChanged(int)));
	disconnect(GoalColorCombo, SIGNAL(activated ( int)), this, SLOT(GoalColorChanged(int)));

	

	// Destroy "Gustavo" Threads

	
	

	//Delete
	if(UpdateTimer!=NULL)	delete UpdateTimer; UpdateTimer=NULL;
    if(fullinfowindow!=NULL) delete fullinfowindow; fullinfowindow=NULL;
    if(FIW!=NULL)			delete FIW; FIW=NULL;

	if( this->RefBoxWG != NULL ) delete this->RefBoxWG; this->RefBoxWG = NULL;
}


void MWind::TeamColorChanged(int team)
{
QColor Mag = QColor::fromRgb(222,111,161,255);//Qt::magenta;//
QColor Cy = QColor::fromRgb(128,160,191,255);
QPalette plt;




	if (team == 0) 
	{
		plt.setColor(QPalette::Button, Mag);
	}
	else 
	{
		plt.setColor(QPalette::Button, Cy);
	}
	
	TeamColorCombo->setPalette(plt);

}

void MWind::GoalColorChanged(int goal)
{

QColor Yell = QColor::fromRgb(255,191,105,255);
QColor Bl =  QColor::fromRgb(0,180,247,255);//Qt::blue;//
QPalette plt;



	if (goal == 0) 
	{
		plt.setColor(QPalette::Button, Yell);
	}
	else 
	{
		plt.setColor(QPalette::Button, Bl);
	}

	GoalColorCombo->setPalette(plt);
	GoalColorCombo->setCurrentIndex(goal);
}



bool MWind::eventFilter(QObject *obj, QEvent *event)
{
	if (event->type() == QEvent::KeyPress)
	{

		/* Shortcuts da basestation */
		QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

		if (keyEvent->key() == Qt::Key_F1)


		if (keyEvent->key() == Qt::Key_F2)


		if (keyEvent->key() == Qt::Key_F3)

		if (keyEvent->key() == Qt::Key_F4)

		if (keyEvent->key() == Qt::Key_F5)

        if (keyEvent->key() == Qt::Key_F6)

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
	
	if ( (obj == mwind) && (event->type() == QEvent::Resize) )
	{
		QSize s = mwind->size();

		/* Janela Info */
		char size_inc = 0;
		if 	(s.rwidth() > 1263) size_inc=5;
		else if	(s.rwidth() > 1248) size_inc=4;
		else if	(s.rwidth() > 1194) size_inc=3;
		else if	(s.rwidth() > 1059) size_inc=2;
		else if	(s.rwidth() > 993 ) size_inc=1;

		size_inc=3;

		/* Janela principal */
		size_inc = 0;
		if 	(s.rwidth() > 1278) size_inc=3;
		else if	(s.rwidth() > 1158) size_inc=2;
		else if	(s.rwidth() > 1086) size_inc=1;

		size_inc=3;

		


		// Deal with Logo
		//printf("cenas %d\n",s.rheight());

		return true;
	}


	return false;
}

void MWind::UpdateGameTime(void)
{
	QTime GTime;
	int min=0, sec=0;
}

void MWind::handleSharedWorldInfo(boost::shared_ptr<msl_sensor_msgs::SharedWorldInfo> info)
{
}

void MWind::UpdateGameParameters(void)
{

}


