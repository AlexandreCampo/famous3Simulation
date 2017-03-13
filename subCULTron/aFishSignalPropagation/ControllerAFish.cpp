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


ControllerAFish::ControllerAFish (aFish* fish)
    : Controller (fish)
{
    this->fish = fish;

    Reset();
}


void ControllerAFish::Step ()
{
    float time = object->simulator->time;
    float timestep = object->simulator->timestep;

    // if a message is received, record data
    bool messageReceived = false;
    float x = 0;
    float y = 0;
    int n = 0;
    DeviceOpticalTransceiver::Message msg;
    while (fish->optical->Receive(msg))
    {
	messageReceived = true;
	x += msg.direction.x();
	y += msg.direction.y();
	n++;
    }
    if (messageReceived)
    {
	x /= n;
	y /= n;
    }

    // not in refractory ?
    if (time > lastBlinkTime + refractoryPeriod)
    {
	// with some proba, send a blink
	float rnd = gsl_ran_flat(rng, 0.0, 1.0);
	if (rnd < blinkProba * timestep)
	{
	    fish->optical->Send(1);
	    lastBlinkTime = time;
	    fish->SetColor(1, 0, 0);
	}
	// relay blink ?
	else if (messageReceived)
	{
	    fish->optical->Send(1);
	    lastBlinkTime = time;
	    fish->SetColor(1, 0, 0);
	}
	else
	{
	    fish->SetColor(1, 1, 55.0/254.0);
	}

	// get attracted towards message emitters
	if (messageReceived > 0)
	{
	    float angle = atan2(y, x);
	    if (fabs(angle) < 30.0 * M_PI / 180.0)
	    {
		leftSpeed = speed * forwardCoeff;
		rightSpeed = speed * forwardCoeff;
	    }
	    else if (fabs(angle) > 150.0 * M_PI / 180.0)
	    {
		leftSpeed = -speed * forwardCoeff;
		rightSpeed = -speed * forwardCoeff;
	    }
	    else if (y < 0)
	    {
		leftSpeed = -speed;
		rightSpeed = speed;
	    }
	    else
	    {
		leftSpeed = speed;
		rightSpeed = -speed;
	    }
	}
	// no msg received, stay still
	else
	{
	    leftSpeed = 0;
	    rightSpeed = 0;
	}
    }

    fish->propellerLeft->SetSpeed(leftSpeed);
    fish->propellerRight->SetSpeed(rightSpeed);	    

    return;
}

void ControllerAFish::Reset()
{
    lastBlinkTime = 0;
    fish->SetColor(1, 1, 55.0/254.0);
    leftSpeed = 0;
    rightSpeed = 0;
}
