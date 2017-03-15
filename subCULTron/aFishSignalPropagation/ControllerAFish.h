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

#ifndef CONTROLLER_A_FISH_H
#define CONTROLLER_A_FISH_H

#include "Controller.h"
#include "aFish.h"

class ControllerAFish : public Controller
{
public : 
    aFish* fish;

    float refractoryPeriod = 2.0;
    float lastBlinkTime = 0.0;
    float blinkProba = 0.05;
    float speed = 0.6;
    float forwardCoeff = 0.02;

    float leftSpeed;
    float rightSpeed;
    
    // methods
    ControllerAFish (aFish* fish);
    ~ControllerAFish ();

    void step ();

    void reset ();
};


#endif
