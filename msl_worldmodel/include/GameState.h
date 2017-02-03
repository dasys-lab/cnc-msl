/*
 * GameState.h
 *
 *  Created on: Jul 8, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAMESTATE_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAMESTATE_H_

namespace msl
{
enum GameState
{
    Duel,
    OwnBallPossession,
    OppBallPossession,
    NobodyInBallPossession
    // former:  Melee, Attack, Defend, Conflict
};
}

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAMESTATE_H_ */
