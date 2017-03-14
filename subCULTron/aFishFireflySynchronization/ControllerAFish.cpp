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

    // update ffcounter
    counter -= counter * gamma * GetTimeStep();

    // not in refractory ?
    DeviceOpticalTransceiver::Message msg;
    if (time > lastBlinkTime + refractoryPeriod)
    {	
	// check for messages
	if (fish->optical->Receive(msg))
	{
	    counter -= epsilon;
	}
    }

    // empty msg buffer
    while (fish->optical->Receive(msg));

    // blink if counter has reached timeout
    if (counter <= 0.1)
    {
	fish->optical->Send(1);
	lastBlinkTime = time;
	fish->SetColor(1, 0, 0);
	counter = 1;
    }
    else
    {
	fish->SetColor(1, 1, 55.0/254.0);
    }
    

    // fish->SetTextDrawable(true);
    // fish->SetText(to_string(counter));

    return;
    
}

void ControllerAFish::Reset()
{
    lastBlinkTime = 0;
    fish->SetColor(1, 1, 55.0/254.0);

    counter = gsl_ran_flat(rng, 0, 1);
    lastBlinkTime = -refractoryPeriod;
}
