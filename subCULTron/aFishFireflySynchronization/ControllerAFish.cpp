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


ControllerAFish::ControllerAFish (aFish* fish)
{
    this->fish = fish;

    reset();
}

void ControllerAFish::step ()
{
    float time = object->simulator->time;

    // update ffcounter
    counter -= counter * gamma * getTimestep();

    // not in refractory ?
    DeviceOpticalTransceiver::Message msg;
    if (time > lastBlinkTime + refractoryPeriod)
    {	
	// check for messages
	if (fish->optical->receive(msg))
	{
	    counter -= epsilon;
	}
    }

    // empty msg buffer
    while (fish->optical->receive(msg));

    // blink if counter has reached timeout
    if (counter <= 0.1)
    {
	fish->optical->send(1);
	lastBlinkTime = time;
	fish->setColor(1, 0, 0);
	counter = 1;
    }
    else
    {
	fish->setColor(1, 1, 55.0/254.0);
    }
    

    // fish->setTextDrawable(true);
    // fish->setText(to_string(counter));

    return;
    
}

void ControllerAFish::reset()
{
    lastBlinkTime = 0;
    fish->setColor(1, 1, 55.0/254.0);

    counter = gsl_ran_flat(rng, 0, 1);
    lastBlinkTime = -refractoryPeriod;
}
