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

#include "RefBoxWidget.h"

RefBoxWidget::RefBoxWidget(QWidget * parent)
{
	setupUi(parent);

	RBDialog = new QDialog(parent);
	RBDial = new RefBoxDialog(RBDialog);
	UpdateTimer = new QTimer();

	/* Inicialização das comboboxes (our & their)*/

	/* conecções */

	connect(Play_On_bot, SIGNAL(clicked()), this, SLOT(PlayOnPressed()));
	connect(Stop_bot, SIGNAL(clicked()), this, SLOT(StopPressed()));
	connect(Halt_bot, SIGNAL(clicked()), this, SLOT(HaltPressed()));
	connect(Dropped_bot, SIGNAL(clicked()), this, SLOT(DroppedBallPressed()));
	connect(Parking_bot, SIGNAL(clicked()), this, SLOT(ParkingPressed()));

	//our
	connect(Our_Kick_Off_bot, SIGNAL(clicked()), this, SLOT(OurKickOffPressed()));
	connect(Our_Free_Kick_bot, SIGNAL(clicked()), this, SLOT(OurFreeKickPressed()));
	connect(Our_Goal_Kick_bot, SIGNAL(clicked()), this, SLOT(OurGoalKickPressed()));
	connect(Our_Throwin_bot, SIGNAL(clicked()), this, SLOT(OurThrowinPressed()));
	connect(Our_Corner_Kick_bot, SIGNAL(clicked()), this, SLOT(OurCornerKickPressed()));
	connect(Our_Penalty_bot, SIGNAL(clicked()), this, SLOT(OurPenaltyPressed()));

	//their
	connect(Their_Kick_Off_bot, SIGNAL(clicked()), this, SLOT(TheirKickOffPressed()));
	connect(Their_Free_Kick_bot, SIGNAL(clicked()), this, SLOT(TheirFreeKickPressed()));
	connect(Their_Goal_Kick_bot, SIGNAL(clicked()), this, SLOT(TheirGoalKickPressed()));
	connect(Their_Throwin_bot, SIGNAL(clicked()), this, SLOT(TheirThrowinPressed()));
	connect(Their_Corner_Kick_bot, SIGNAL(clicked()), this, SLOT(TheirCornerKickPressed()));
	connect(Their_Penalty_bot, SIGNAL(clicked()), this, SLOT(TheirPenaltyPressed()));

	connect(RBDial, SIGNAL(transmitCoach()), this, SLOT(updateCoachInfo()));

	connect(RBDial, SIGNAL(changeGoalColor(int)), this, SLOT(changeGoalColor_sl(int)));

	connect(RBDial, SIGNAL(updateGameParam()), this, SLOT(UpdateGameParameter_slot()));

	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(updateStateInfo()));

	UpdateTimer->start(50);

	RefLog->clear();

}
RefBoxWidget::~RefBoxWidget()
{
	disconnect(Play_On_bot, SIGNAL(clicked()), this, SLOT(PlayOnPressed()));
	disconnect(Stop_bot, SIGNAL(clicked()), this, SLOT(StopPressed()));
	disconnect(Halt_bot, SIGNAL(clicked()), this, SLOT(HaltPressed()));
	disconnect(Dropped_bot, SIGNAL(clicked()), this, SLOT(DroppedBallPressed()));
	disconnect(Parking_bot, SIGNAL(clicked()), this, SLOT(ParkingPressed()));
	disconnect(Our_Kick_Off_bot, SIGNAL(clicked()), this, SLOT(OurKickOffPressed()));
	disconnect(Our_Free_Kick_bot, SIGNAL(clicked()), this, SLOT(OurFreeKickPressed()));
	disconnect(Our_Goal_Kick_bot, SIGNAL(clicked()), this, SLOT(OurGoalKickPressed()));
	disconnect(Our_Throwin_bot, SIGNAL(clicked()), this, SLOT(OurThrowinPressed()));
	disconnect(Our_Corner_Kick_bot, SIGNAL(clicked()), this, SLOT(OurCornerKickPressed()));
	disconnect(Our_Penalty_bot, SIGNAL(clicked()), this, SLOT(OurPenaltyPressed()));
	disconnect(Their_Kick_Off_bot, SIGNAL(clicked()), this, SLOT(TheirKickOffPressed()));
	disconnect(Their_Free_Kick_bot, SIGNAL(clicked()), this, SLOT(TheirFreeKickPressed()));
	disconnect(Their_Goal_Kick_bot, SIGNAL(clicked()), this, SLOT(TheirGoalKickPressed()));
	disconnect(Their_Throwin_bot, SIGNAL(clicked()), this, SLOT(TheirThrowinPressed()));
	disconnect(Their_Corner_Kick_bot, SIGNAL(clicked()), this, SLOT(TheirCornerKickPressed()));
	disconnect(Their_Penalty_bot, SIGNAL(clicked()), this, SLOT(TheirPenaltyPressed()));
	disconnect(RBDial, SIGNAL(transmitCoach()), this, SLOT(updateCoachInfo()));
	disconnect(RBDial, SIGNAL(changeGoalColor(int)), this, SLOT(changeGoalColor_sl(int)));
	disconnect(RBDial, SIGNAL(updateGameParam()), this, SLOT(UpdateGameParameter_slot()));
	disconnect(UpdateTimer, SIGNAL(timeout()), this, SLOT(updateStateInfo()));

	delete RBDialog;
	delete RBDial;
	delete UpdateTimer;
}

void RefBoxWidget::detailsBotPressed(void)
{
	RBDial->update_manual_config();
	RBDialog->show();
}

/*================================================== Game States =====================================*/
void RefBoxWidget::PlayOnPressed(void)
{
	printf("PlayOnPressed SIGstart\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::StopPressed(void)
{
	printf("StopPressed SIGstop\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::HaltPressed(void)
{
	printf("HaltPressed SIGhalt");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::DroppedBallPressed(void)
{
	printf("DroppedBallPressed SIGdropBall\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::ParkingPressed(void)
{
	printf("ParkingPressed SIGparking\n");

	Q_EMIT transmitCoach();
}

//================================================ Our States =======================================

void RefBoxWidget::OurKickOffPressed(void)
{
	printf("OurKickOffPressed SIGourKickOff\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::OurFreeKickPressed(void)
{
	printf("OurFreeKickPressed IGourFreeKick\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::OurGoalKickPressed(void)
{
	printf("OurGoalKickPressed SIGourGoalKick\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::OurThrowinPressed(void)
{
	printf("OurThrowinPressed SIGourThrowIn\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::OurCornerKickPressed(void)
{
	printf("OurCornerKickPressed SIGourCornerKick\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::OurPenaltyPressed(void)
{
	printf("OurPenaltyPressed SIGourPenalty\n");

	Q_EMIT transmitCoach();
}

//================================================ Their States =======================================

void RefBoxWidget::TheirKickOffPressed(void)
{
	printf("TheirKickOffPressed SIGtheirKickOff\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::TheirFreeKickPressed(void)
{
	printf("TheirFreeKickPressed SIGtheirFreeKick\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::TheirGoalKickPressed(void)
{
	printf("TheirGoalKickPressed SIGtheirGoalKick\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::TheirThrowinPressed(void)
{
	printf("TheirThrowinPressed SIGtheirThrowIn\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::TheirCornerKickPressed(void)
{
	printf("TheirCornerKickPressed SIGtheirCornerKick\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::TheirPenaltyPressed(void)
{
	printf("TheirPenaltyPressed SIGtheirPenalty\n");

	Q_EMIT transmitCoach();
}

void RefBoxWidget::updateStateInfo(void)
{

	QColor color;
	QColor White = Qt::white; //QColor::fromRgb(0,0,0,255);
	QColor Green = Qt::green; //QColor::fromRgb(0,0,0,255);
	QColor Red = Qt::red; //QColor::fromRgb(0,0,0,255);
	QColor back_color = QColor::fromRgb(72, 72, 72, 255);
	QPalette plt;

	// TODO 1 = enum for GameState
	QString gmstate = QString(refbox_signal_names[1]);

	if (gmstate.contains("SIGstop"))
	{
		color = Red;
	}
	else if (gmstate.contains("SIGstart"))
	{
		color = Green;
	}
	else
		color = White;

	// TODO 1 = enum for GameState
	State_val->setText(QString(" ") + refbox_signal_names[1]);

	plt.setColor(QPalette::Background, back_color);
	plt.setColor(QPalette::Foreground, color);
	State_val->setPalette(plt);

}

void RefBoxWidget::updateCoachInfo(void)
{

	//Update Ref Log
	updateRefLog();

	Q_EMIT transmitCoach();
}

void RefBoxWidget::updateRefLog(void)
{

	QTime GTime;
	int min = 0, sec = 0;

	// TODO set times right
	min = 1; //(int)((db_coach_info->GameTime.elapsed() / 1000) / 60);
	sec = 1; //(int)((db_coach_info->GameTime.elapsed() / 1000) % 60);

	GTime.setHMS(0, min, sec);

	QString Msg = GTime.toString("mm:ss");
	Msg.append(" - ");

	// TODO 1 = enum for GameState
	Msg.append(QString(refbox_signal_names[1]));
	//QString* Msg = new QString(refbox_signal_names[db_coach_info->Coach_Info.gameState]);
	RefLog->append(Msg);

}

void RefBoxWidget::changeGoalColor_sl(int goal)
{

	printf("emit change color RefBWidget\n");
	Q_EMIT changeGoalColor(goal);
}

void RefBoxWidget::UpdateGameParameter_slot(void)
{
	Q_EMIT UpdateGameParameter_signal();
}

void RefBoxWidget::SetLogViewMode_slot(bool on_off)
{
	Q_EMIT SetLogViewMode_signal(on_off);
}
