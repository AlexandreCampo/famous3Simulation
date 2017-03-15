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
{
    this->fish = fish;

    reset();
}


void ControllerAFish::step ()
{
    time = object->simulator->time;

    // send a message in all directions
    fish->optical->send(1);
    
    // receive messages
    messagesReceived = 0;
    msgx = 0;
    msgy = 0;
    int n = 0;
    DeviceOpticalTransceiver::Message msg;    
    while (fish->optical->receive(msg))
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
    case EXPLORE : stateExplore(); break;
    case TURN : stateTurn(); break;
    case REST : stateRest(); break;
    case BRAKE : stateBrake(); break;
    }
}

void ControllerAFish::stateExploreInit ()
{
    float rnd = 1.0 - gsl_ran_flat(rng, 0.0, 1.0);
    stateDuration = - log (rnd) * exploreMeanDuration;

    stateStartTime = time;
    state = EXPLORE;

    // set robot's colour
    fish->setColor(1, 1, 55.0/254.0);
}


void ControllerAFish::stateExplore ()
{
    // if several messages received -> stop
    if (messagesReceived >= 3)
    {
	stateBrakeInit();
	return;
    }
    
    // if time to change direction -> turn
    if (time - stateStartTime > stateDuration)
    {
	// jump to turn state
	float angle = gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI;    
	stateTurnInit(EXPLORE, angle);

	return;
    }

    // inside the state    
    if (obstacleAvoidance ())
	return;

    fish->propellerLeft->setSpeed(exploreSpeed);
    fish->propellerRight->setSpeed(exploreSpeed);
}


void ControllerAFish::stateTurnInit(int previousState, float angle)
{
    turnPreviousState = previousState;

    if (angle < 0.0)
	turnSign = 1.0;
    else
	turnSign = -1.0;

    stateDuration = (fabs(angle) / M_PI) / 3.0 / turnSpeed;
    stateStartTime = time;
    state = TURN;

    fish->propellerLeft->setSpeed(turnSpeed * turnSign);
    fish->propellerRight->setSpeed(-turnSpeed * turnSign);
}

void ControllerAFish::stateTurn()
{
    // transitions to other states
    if (time - stateStartTime > stateDuration)
    {
	switch (turnPreviousState)
	{
	case EXPLORE : stateExploreInit(); return;
	}
    }
}


void ControllerAFish::stateBrakeInit ()
{
    stateStartTime = time;
    state = BRAKE;

    fish->setColor(1, 0, 0);

    fish->propellerLeft->setSpeed(-brakeSpeed);
    fish->propellerRight->setSpeed(-brakeSpeed);
}


void ControllerAFish::stateBrake ()
{
    if (time - stateStartTime > brakeDuration)
    {
	stateRestInit();
	return;
    }
}


void ControllerAFish::stateRestInit ()
{
    stateDuration = restDuration;

    stateStartTime = time;
    state = REST;

    fish->setColor(55.0/255.0, 1, 55.0/255.0);
    
    fish->propellerLeft->setSpeed(0);
    fish->propellerRight->setSpeed(0);

//    cout << this << " init rest" << endl;
}


void ControllerAFish::stateRest ()
{
    // if time to change direction -> turn
    if (time - stateStartTime > stateDuration)
    {
	// jump to explore state
	stateExploreInit();

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

    fish->propellerLeft->setSpeed(ls);
    fish->propellerRight->setSpeed(rs);
    
}

bool ControllerAFish::obstacleAvoidance()
{    
    // check if an obstacle is perceived 
    int obstaclePerceived = 0;

    float pl = fish->rayFrontLU->getValue() + fish->rayFrontLD->getValue() + fish->rayLeft->getValue();
    float pr = fish->rayFrontRU->getValue() + fish->rayFrontRD->getValue() + fish->rayRight->getValue();
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
    fish->propellerLeft->setSpeed(leftSpeed);
    fish->propellerRight->setSpeed(rightSpeed);

    // advertise obstacle avoidance in progress
    return true;
}


void ControllerAFish::reset ()
{
    // reset time
    time = 0.0;

    // state working variables
    stateDuration = 0.0;
    stateStartTime = 0.0;

    turnPreviousState = 0;
    turnSign = 1.0;
    
    // start in explore state
    state = EXPLORE;
    stateExploreInit();
}


