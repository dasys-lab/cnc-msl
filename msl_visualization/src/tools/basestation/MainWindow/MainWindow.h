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

#ifndef __MAINWINDOW_H
#define __MAINWINDOW_H

#include <QtGui>

#include "ui_MainWindow.h"

#include <iostream>
#include <string>

using namespace std;

class MWind : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

  public:
    MWind(QMainWindow *parent = 0);
    ~MWind();

    // LMOTA
    void incrementOurGoals()
    {
    }
    void incrementTheirGoals()
    {
    }

    /*const WSColor ourColor() const{
      assert(db_coach_info!=NULL);
      return db_coach_info->TeamColor;}
    const WSColor theirColor() const{assert(db_coach_info!=NULL);
      return (db_coach_info->TeamColor==Magenta?Cyan:Magenta);}*/

  private:
    bool fullscreenflag;         // indica se a janela se encontra no modo fullscreen
    QMainWindow *mwind;          // ponteiro para o mainwindow pai (parent*)
    QMainWindow *fullinfowindow; // ponteiro para a fullinfowindow
    QTimer *UpdateTimer;

  protected:
    bool eventFilter(QObject *obj, QEvent *event);

  public Q_SLOTS:
    void TeamColorChanged(int team);
    void RobotVisChanged(int team, int robot_id);
    void GoalColorChanged(int goal);

    void UpdateGameTime(void);
    void UpdateGameParameters(void);
  private Q_SLOTS:
};

extern MWind *wind;

#endif
