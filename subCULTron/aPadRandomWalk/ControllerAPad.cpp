/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2017 Alexandre Campo                                 */
/*                                                                            */
/*    This file is part of FaMouS  (a fast, modular and simple simulator).    */
/*                                                                            */
/*    FaMouS is free software: you can redistribute it and/or modify          */
/*    it under the terms of the GNU General Public License as published by    */
/*    the Free Software Foundation, either version 3 of the License, or       */
/*    (at your option) any later version.                                     */
/*                                                                            */
/*    FaMouS is distributed in the hope that it will be useful,               */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/*    GNU General Public License for more details.                            */
/*                                                                            */
/*    You should have received a copy of the GNU General Public License       */
/*    along with FaMouS.  If not, see <http://www.gnu.org/licenses/>.         */
/*----------------------------------------------------------------------------*/

#include "ControllerAPad.h"

#include "Simulator.h"

#include <cmath>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
extern gsl_rng* rng;
extern long int rngSeed;

#define EXPLORE 0
#define TURN 1

ControllerAPad::ControllerAPad (aPad* pad)
{
    this->pad = pad;

    reset();
}


void ControllerAPad::step ()
{    
    time = object->simulator->time;
    
    switch (state)
    {
    case EXPLORE : stateExplore(); break;
    case TURN : stateTurn(); break;
    }
}

void ControllerAPad::stateExploreInit ()
{
    float rnd = 1.0 - gsl_ran_flat(rng, 0.0, 1.0);
    exploreDuration = - log (rnd) * exploreMeanDuration;

    exploreStartTime = time;
    state = EXPLORE;

    // set robot's colour
    pad->setColor(1, 1, 55.0/254.0);
}


void ControllerAPad::stateExplore ()
{
    // if time to change direction -> turn
    if (time - exploreStartTime > exploreDuration)
    {
    	// jump to turn state
    	float angle = gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI;    
    	stateTurnInit(EXPLORE, angle);

    	return;
    }

    pad->propellerLeft->setSpeed(exploreSpeed);
    pad->propellerRight->setSpeed(exploreSpeed);
}


void ControllerAPad::stateTurnInit(int previousState, float angle)
{
    turnPreviousState = previousState;

    if (angle < 0.0)
	turnSign = 1.0;
    else
	turnSign = -1.0;

    turnDuration = (fabs(angle) / M_PI) * 7.3 / turnSpeed;
    turnStartTime = time;
    state = TURN;
}

void ControllerAPad::stateTurn()
{
    // transitions to other states
    if (time - turnStartTime > turnDuration)
    {
	switch (turnPreviousState)
	{
	case EXPLORE : stateExploreInit(); return;
	}
    }

    // inside the state
    pad->propellerLeft->setSpeed(turnSpeed * turnSign);
    pad->propellerRight->setSpeed(-turnSpeed * turnSign);
}

void ControllerAPad::reset ()
{
    // reset time
    time = 0.0;

    // state working variables
    exploreDuration = 0.0;
    exploreStartTime = 0.0;

    turnPreviousState = 0;
    turnDuration = 0.0;
    turnStartTime = 0.0;
    turnSign = 1.0;
    
    // start in explore state
    state = EXPLORE;
    stateExploreInit();

    state = TURN;
    stateTurnInit(EXPLORE, M_PI);
}


