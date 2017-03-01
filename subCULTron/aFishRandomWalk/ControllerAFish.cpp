/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2012 Alexandre Campo                                 */
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

#include "ControllerAFish.h"

#include "Simulator.h"

#include <cmath>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
extern gsl_rng* rng;
extern long int rngSeed;

#define EXPLORE 0
#define TURN 1

ControllerAFish::ControllerAFish (aFish* fish)
    : Controller (fish)
{
    this->fish = fish;

    Reset();
}


void ControllerAFish::Step ()
{
    time = object->simulator->time;
    timestep = object->simulator->timestep;
    
    switch (state)
    {
    case EXPLORE : StateExplore(); break;
    case TURN : StateTurn(); break;
    }
}

void ControllerAFish::StateExploreInit ()
{
    float rnd = 1.0 - gsl_ran_flat(rng, 0.0, 1.0);
    exploreDuration = - log (rnd) * exploreMeanDuration;

    exploreStartTime = time;
    state = EXPLORE;

    // set robot's colour
    fish->SetColor(1, 1, 55.0/254.0);
}


void ControllerAFish::StateExplore ()
{
    // if time to change direction -> turn
    if (time - exploreStartTime > exploreDuration)
    {
	// jump to turn state
	float angle = gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI;    
	StateTurnInit(EXPLORE, angle);

//	cout << "turning" << endl;
	return;
    }

    // inside the state    
    if (ObstacleAvoidance ())
	return;

    fish->propellerLeft->SetSpeed(exploreSpeed);
    fish->propellerRight->SetSpeed(exploreSpeed);
}


void ControllerAFish::StateTurnInit(int previousState, float angle)
{
    turnPreviousState = previousState;

    if (angle < 0.0)
	turnSign = 1.0;
    else
	turnSign = -1.0;

    turnDuration = (fabs(angle) / M_PI) / 3.0 / turnSpeed;
    turnStartTime = time;
    state = TURN;
}

void ControllerAFish::StateTurn()
{
    // transitions to other states
    if (time - turnStartTime > turnDuration)
    {
	switch (turnPreviousState)
	{
	case EXPLORE : StateExploreInit(); return;
	}
    }

    // inside the state
    fish->propellerLeft->SetSpeed(turnSpeed * turnSign);
    fish->propellerRight->SetSpeed(-turnSpeed * turnSign);
}


bool ControllerAFish::ObstacleAvoidance()
{    
    // check if an obstacle is perceived 
    int obstaclePerceived = 0;

    float pl = fish->rayFrontLU->GetValue() + fish->rayFrontLD->GetValue() + fish->rayLeft->GetValue();
    float pr = fish->rayFrontRU->GetValue() + fish->rayFrontRD->GetValue() + fish->rayRight->GetValue();
    pl /= 3.0;
    pr /= 3.0;   

    // don't take into account down obstacles, as it can be the ground...
    if (fish->rayFrontLU->hasHit()
//	|| fish->rayFrontLD->hasHit()
	|| fish->rayFrontRU->hasHit()
//	|| fish->rayFrontRD->hasHit()
	|| fish->rayLeft->hasHit()
	|| fish->rayRight->hasHit()
	)
	obstaclePerceived = 1;	    
    
    // no obstacles to avoid, return immediately
    if (obstaclePerceived == 0)
	return false;
    
    float leftSpeed = 0.0;
    float rightSpeed = 0.0;

    // proportional break
    if (pr > obstacleAvoidanceThreshold && pl > obstacleAvoidanceThreshold)
    {
//	cout << "OA1 " << pr << " " << pl << endl;
//	cout << fish->rayFrontLU->GetValue() << " " << fish->rayFrontRU->GetValue() << " "  << fish->rayFrontLD->GetValue() << " "  << fish->rayFrontRD->GetValue() << " "  << fish->rayLeft->GetValue() << " "  << fish->rayRight->GetValue() << " " << endl;
	// break symmetry
	leftSpeed = -breakSpeed * pr / (pr + pl);
	rightSpeed = -breakSpeed * pl / (pr + pl);	
    }
    // turn left
    if (pr >= pl)
    {
//	cout << "OA2 " << pr << " " << pl << endl;
	leftSpeed = -obstacleAvoidanceSpeed;
	rightSpeed = obstacleAvoidanceSpeed;
    }
    // turn right
    else
    {
//	cout << "OA3 " << pr << " " << pl << endl;
	leftSpeed = obstacleAvoidanceSpeed;
	rightSpeed = -obstacleAvoidanceSpeed;
    }
    
    // change movement direction
    fish->propellerLeft->SetSpeed(leftSpeed);
    fish->propellerRight->SetSpeed(rightSpeed);

    // advertise obstacle avoidance in progress
    return true;
}


void ControllerAFish::Reset ()
{
    // reset time
    time = 0.0;
    timestep = 0.0;

    // state working variables
    exploreDuration = 0.0;
    exploreStartTime = 0.0;

    turnPreviousState = 0;
    turnDuration = 0.0;
    turnStartTime = 0.0;
    turnSign = 1.0;
    
    // start in explore state
    state = EXPLORE;
    StateExploreInit();
}


