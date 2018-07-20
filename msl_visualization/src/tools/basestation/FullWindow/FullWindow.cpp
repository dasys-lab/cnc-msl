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

#include "FullWindow.h"

FWind::FWind(QMainWindow *parent)
{
    /* Criação da janela */
    setupUi(parent);

    /* Inicialização das variáveis locais */
    fullscreenflag = 0;
    fullwindow = parent;

    UpdateTimer = new QTimer();

    /* Conecções */
    connect(FullScrBot, SIGNAL(clicked()), this, SLOT(WindowFullScreenMode()));
    connect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
    connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateGameInfo()));

    UpdateTimer->start(100);
}

FWind::~FWind()
{
    // disconnect(FullScrBot, SIGNAL(clicked()), this, SLOT(WindowFullScreenMode()));
    // disconnect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
    // disconnect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
    // disconnect(UpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateGameInfo()));

    delete UpdateTimer;
}

void FWind::WindowFullScreenMode(void)
{
    if (fullscreenflag == 0)
    {
        fullscreenflag = 1;
        fullwindow->showFullScreen();
        FullScrBot->setText(QString("Leave Full Screen"));
    }
    else
    {
        fullscreenflag = 0;
        fullwindow->showMaximized();
        FullScrBot->setText(QString("Full Screen"));
    }
}

void FWind::UpdateGameInfo(void)
{

    QTime GTime;
    int min = 0, sec = 0;
}
