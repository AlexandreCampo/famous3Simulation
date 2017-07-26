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

#include <Eigen/Eigen>

extern gsl_rng* rng;
extern long int rngSeed;

#define EXPLORE 0
#define TURN 1

using namespace std;

ControllerAFish::ControllerAFish (aFish* fish)
{
    this->fish = fish;

    reset();
}


void ControllerAFish::step ()
{
    time = object->simulator->time;

    Eigen::VectorXf pola(5);
    
// update ffcounter
    counter += 1.0 * getTimestep();
cout << "counter : ";    
  	cout << counter << " ";
	cout << actif_passif << " ";
    	cout << endl;

    if (counter <= counter_threshold) 
    {
        cout << "PASSIF " << endl;
        fish->setColor(1, 1, 55.0/254.0);
        // passif
        pola << 0, 0, 0, 0, 0;
        fish->esense->setPolarization (pola);
        // read e-sense
        fish->esense->getCurrents();
	cout << "E-sense current measured : ";    
    	for (int i = 0; i < fish->esense->numElectrodes; i++)
    	{
	     cout << fish->esense->I(i) << " ";
    	}
        cout << endl;
        if ((fish->esense->I(0)==0))// && (actif_passif == 1))
        {
            actif_passif = 0;
        }
        if ((fish->esense->I(0)!=0) )//&& (actif_passif == 0))
        {
           // counter = counter -  1*counter_threshold;
            counter -= 1.0* getTimestep();
            cout << "counter delayed ";
	    cout << counter << " ";
    	    cout << endl;
	    actif_passif = 1;
            fish->setColor(0, 1, 0);
        }
    }
    else
    {
        cout << "actIF " << endl;
        // actif
        pola << 10, 0, 0, 0, 0;
        fish->esense->setPolarization (pola);
        // read e-sense
        fish->esense->getCurrents();
        fish->setColor(1, 0, 0);
    }
    if (counter>=counter_max)
    {
        cout << "max " << endl;
        fish->setColor(1, 1, 55.0/254.0);
        // passif
        pola << 0, 0, 0, 0, 0;
        fish->esense->setPolarization (pola);
        counter = 0.0;
        actif_passif = 0;
    }

    // read e-sense
    fish->esense->getCurrents();

    
    
    
    // send a message
    fish->optical->send(1);
    
    // receive messages
    DeviceOpticalTransceiver::Message msg;
    while (fish->optical->receive(msg))
    {
//	if (dbg) cout << this << " fish received msg " << msg.content << " at time " << time << endl;
    }

    
    switch (state)
    {
    case EXPLORE : stateExplore(); break;
    case TURN : stateTurn(); break;
    }
}

void ControllerAFish::stateExploreInit ()
{
    float rnd = 1.0 - gsl_ran_flat(rng, 0.0, 1.0);
    exploreDuration = - log (rnd) * exploreMeanDuration;

    exploreStartTime = time;
    state = EXPLORE;

    // set robot's colour
    fish->setColor(1, 1, 55.0/254.0);
}


void ControllerAFish::stateExplore ()
{
    // if time to change direction -> turn
    if (time - exploreStartTime > exploreDuration)
    {
	// jump to turn state
	float angle = gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI;    
	stateTurnInit(EXPLORE, angle);

//	cout << "turning" << endl;
	return;
    }

    // inside the state    
    if (obstacleAvoidance ())
	return;
    /*float ac_pas;
    float ac_pas_rot;
 
    ac_pas = -0.05;
    ac_pas_rot = -400000.0;

    if (fish->esense->polarization(0) ==0)
    {
    ac_pas = 3.0;
    ac_pas_rot = 900.0;
    }*/
    fish->propellerLeft->setSpeed(exploreSpeed*0);
    fish->propellerRight->setSpeed(exploreSpeed*0);
    /*fish->propellerLeft->setSpeed(ac_pas*(fish->esense->I(1)+fish->esense->I(3))+ac_pas_rot*(fish->esense->I(1)-fish->esense->I(3)));
    fish->propellerRight->setSpeed(ac_pas*(fish->esense->I(1)+fish->esense->I(3))-ac_pas_rot*(fish->esense->I(1)-fish->esense->I(3)));*/
}


void ControllerAFish::stateTurnInit(int previousState, float angle)
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

void ControllerAFish::stateTurn()
{
    // transitions to other states
    if (time - turnStartTime > turnDuration)
    {
	switch (turnPreviousState)
	{
	case EXPLORE : stateExploreInit(); return;
	}
    }
/*    float ac_pas;
    float ac_pas_rot;
    ac_pas = -0.05;
    ac_pas_rot = -400000.0;

    if (fish->esense->polarization(0) ==0)
    {
    ac_pas = 3.0;
    ac_pas_rot = 900.0;
    }*/
    fish->propellerLeft->setSpeed(exploreSpeed*0);
    fish->propellerRight->setSpeed(exploreSpeed*0);
    /*fish->propellerLeft->setSpeed(ac_pas*(fish->esense->I(1)+fish->esense->I(3))+ac_pas_rot*(fish->esense->I(1)-fish->esense->I(3)));
    fish->propellerRight->setSpeed(ac_pas*(fish->esense->I(1)+fish->esense->I(3))-ac_pas_rot*(fish->esense->I(1)-fish->esense->I(3)));*/
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

    // proportional break
    if (pr > obstacleAvoidanceThreshold && pl > obstacleAvoidanceThreshold)
    {
	// break symmetry
	leftSpeed = -breakSpeed * pr / (pr + pl);
	rightSpeed = -breakSpeed * pl / (pr + pl);	
    }
    // turn left
    if (pr >= pl)
    {
	leftSpeed = -obstacleAvoidanceSpeed;
	rightSpeed = obstacleAvoidanceSpeed;
    }
    // turn right
    else
    {
	leftSpeed = obstacleAvoidanceSpeed;
	rightSpeed = -obstacleAvoidanceSpeed;
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
    exploreDuration = 0.0;
    exploreStartTime = 0.0;

    turnPreviousState = 0;
    turnDuration = 0.0;
    turnStartTime = 0.0;
    turnSign = 1.0;
    
    // start in explore state
    state = EXPLORE;
    stateExploreInit();
}



