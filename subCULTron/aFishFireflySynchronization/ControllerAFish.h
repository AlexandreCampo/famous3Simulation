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


    float counter = 1.0;
    float gamma = 0.2;
    float epsilon = 0.05;
    float refractoryPeriod = 0.2;
    float lastBlinkTime = 0.0;

    
    float blinkProba = 0.05;
    float speed = 0.6;
    float forwardCoeff = 0.02;

    float leftSpeed;
    float rightSpeed;
    
    // methods
    ControllerAFish (aFish* fish);
    ~ControllerAFish ();

    void Step ();

    void DoFirefly();
    
    void Reset ();
};


#endif
