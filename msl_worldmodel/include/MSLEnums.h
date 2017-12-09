/*
 * MSLEnums.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_MSLENUMS_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_MSLENUMS_H_

namespace msl
{
enum Situation
{
    Undefined = 0,
    Stop = 1,
    Start = 2,
    Halt = 3,
    Ready = 4,

    FirstHalf = 5,
    HalfTime = 6,
    SecondHalf = 7,
    EndGame = 8,
    Cancel = 9,

    Restart = 10,

    OwnKickoff = 11,
    OppKickoff = 12,

    OwnFreekick = 13,
    OppFreekick = 14,

    OwnGoalkick = 15,
    OppGoalkick = 16,

    OwnThrowin = 17,
    OppThrowin = 18,

    OwnCorner = 19,
    OppCorner = 20,

    OwnPenalty = 21,
    OppPenalty = 22,

    DropBall = 23,
    Parking = 76,

    Joystick = 254,

    //              OwnGoal=24,
    //              OppGoal=25,

    RemoteControl = 28,
    RemoteControlEnd = 29,

};

enum Color
{
    UnknownColor = 0,
    White = 1,
    Black = 2,
    Green = 3,
    Cyan = 4,
    Magenta = 5,
    Yellow = 6,
    Blue = 7
};

enum EntityType
{
    /**
     * ArtificialObstacles are:
     * - the artificial field surrounding
     * - the artificial blocking obstacles (like around the 3m radius)
     *
     * Obstacles are:
     * - real obstacles, which are not robots, e.g., the ball
     */
    Opponent = -1,
    ArtificialObstacle = -2,
    Obstacle = -3,
    UndefinedEntity = -4
};

enum BallPossessionStatus
{
	HaveBall = 0,
	LightBarrierUnblocked = 1,
	NotInKickerDistance = 2,
	AsideOfKicker = 3,
	NoBallSeen = 4
};
}

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_MSLENUMS_H_ */
