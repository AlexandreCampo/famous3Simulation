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
#define BRAKE 2
#define REST 3

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

    // send a message in all directions
    fish->optical->Send(1);
    
    // receive messages
    messagesReceived = 0;
    msgx = 0;
    msgy = 0;
    int n = 0;
    DeviceOpticalTransceiver::Message msg;    
    while (fish->optical->Receive(msg))
    {
	messagesReceived++;
	msgx += msg.direction.x();
	msgy += msg.direction.y();
	if (msg.distance < 0.4) n++;
    }

    // by default skip attraction
    attraction = false;
    
    if (messagesReceived > 0)
    {
	msgx /= messagesReceived;
	msgy /= messagesReceived;
	
	// less than 2 neighbours in close range
	if (n <= 2) attraction = true;
    }
    
    switch (state)
    {
    case EXPLORE : StateExplore(); break;
    case TURN : StateTurn(); break;
    case REST : StateRest(); break;
    case BRAKE : StateBrake(); break;
    }
}

void ControllerAFish::StateExploreInit ()
{
    float rnd = 1.0 - gsl_ran_flat(rng, 0.0, 1.0);
    stateDuration = - log (rnd) * exploreMeanDuration;

    stateStartTime = time;
    state = EXPLORE;

    // set robot's colour
    fish->SetColor(1, 1, 55.0/254.0);
}


void ControllerAFish::StateExplore ()
{
    // if several messages received -> stop
    if (messagesReceived >= 3)
    {
	StateBrakeInit();
	return;
    }
    
    // if time to change direction -> turn
    if (time - stateStartTime > stateDuration)
    {
	// jump to turn state
	float angle = gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI;    
	StateTurnInit(EXPLORE, angle);

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

    stateDuration = (fabs(angle) / M_PI) / 3.0 / turnSpeed;
    stateStartTime = time;
    state = TURN;

    fish->propellerLeft->SetSpeed(turnSpeed * turnSign);
    fish->propellerRight->SetSpeed(-turnSpeed * turnSign);
}

void ControllerAFish::StateTurn()
{
    // transitions to other states
    if (time - stateStartTime > stateDuration)
    {
	switch (turnPreviousState)
	{
	case EXPLORE : StateExploreInit(); return;
	}
    }
}


void ControllerAFish::StateBrakeInit ()
{
    stateStartTime = time;
    state = BRAKE;

    fish->SetColor(1, 0, 0);

    fish->propellerLeft->SetSpeed(-brakeSpeed);
    fish->propellerRight->SetSpeed(-brakeSpeed);
}


void ControllerAFish::StateBrake ()
{
    if (time - stateStartTime > brakeDuration)
    {
	StateRestInit();
	return;
    }
}


void ControllerAFish::StateRestInit ()
{
    stateDuration = restDuration;

    stateStartTime = time;
    state = REST;

    fish->SetColor(55.0/255.0, 1, 55.0/255.0);
    
    fish->propellerLeft->SetSpeed(0);
    fish->propellerRight->SetSpeed(0);

//    cout << this << " init rest" << endl;
}


void ControllerAFish::StateRest ()
{
    // if time to change direction -> turn
    if (time - stateStartTime > stateDuration)
    {
	// jump to explore state
	StateExploreInit();

	return;
    }

    // if messages received -> reset counters
    if (messagesReceived >= 3)
    {
	// cout << this << " keep rest" << endl;
	stateStartTime = time;
    }

    // if attraction, slowly drive towards neighbours
    float ls = 0.0;
    float rs = 0.0;
    if (attraction)
    {
	float angle = atan2(msgy, msgx);
	if (fabs(angle) < 30.0 * M_PI / 180.0)
	{
	    ls = attractionSpeed * 0.1;
	    rs = attractionSpeed * 0.1;
	}
	else if (fabs(angle) > 150.0 * M_PI / 180.0)
	{
	    ls = -attractionSpeed * 0.1;
	    rs = -attractionSpeed * 0.1;
	}
	else if (angle < 0)
	{
	    ls = -attractionSpeed;
	    rs = attractionSpeed;
	}
	else
	{
	    ls = attractionSpeed;
	    rs = -attractionSpeed;
	}
    }

    fish->propellerLeft->SetSpeed(ls);
    fish->propellerRight->SetSpeed(rs);
    
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

    // proportional brake
    if (pr > obstacleAvoidanceThreshold && pl > obstacleAvoidanceThreshold)
    {
	// break symmetry
	leftSpeed = -brakeSpeed * pr / (pr + pl);
	rightSpeed = -brakeSpeed * pl / (pr + pl);	
    }
    // turn left
    if (pr >= pl)
    {
	leftSpeed = -obstacleAvoidanceSpeed + 0.01;
	rightSpeed = obstacleAvoidanceSpeed + 0.01;
    }
    // turn right
    else
    {
	leftSpeed = obstacleAvoidanceSpeed + 0.01;
	rightSpeed = -obstacleAvoidanceSpeed + 0.01;
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
    stateDuration = 0.0;
    stateStartTime = 0.0;

    turnPreviousState = 0;
    turnSign = 1.0;
    
    // start in explore state
    state = EXPLORE;
    StateExploreInit();
}


