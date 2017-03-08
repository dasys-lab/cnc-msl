/*
 * GameState.h
 *
 *  Created on: Jul 8, 2015
 *      Author: Stefan Jakob
 */

#pragma once

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

} /* namespace msl */
