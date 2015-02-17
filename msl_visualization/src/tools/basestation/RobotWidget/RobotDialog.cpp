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

#include "RobotDialog.h"


RobotDialog::RobotDialog(QDialog *parent)
{
	setupUi(parent);
	RobotNumberVal->setValue(-1);

	my_number = -1;

	connect(RobotNumberVal, SIGNAL(valueChanged(int)), this, SLOT(robotNumberChanged(int)));
	connect(TeamColorCombo, SIGNAL(activated(int )), this, SLOT(teamColorChanged(int)));
	connect(GoalColorCombo, SIGNAL(activated(int )), this, SLOT(goalColorChanged(int)));
	connect(StartBot, SIGNAL(clicked()), this, SLOT(startBotPressed()));

}


RobotDialog::~RobotDialog()
{

}

void RobotDialog::get_robot_number(int num)
{
	my_number = num;

	
}


void RobotDialog::robotNumberChanged(int num)
{


}

void RobotDialog::teamColorChanged(int color_id)
{

}

void RobotDialog::goalColorChanged(int color_id)
{


}

void RobotDialog::startBotPressed(void)
{

	if( my_number == -1) return;

	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);
	system( (cmd + " cd ~/bin;./STOP").toAscii() );
	system( (cmd + "  cd ~/bin;./START").toAscii() );
}

