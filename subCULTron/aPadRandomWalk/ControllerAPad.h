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

#ifndef CONTROLLER_APAD_H
#define CONTROLLER_APAD_H

#include "Controller.h"
#include "aPad.h"

class ControllerAPad : public Controller
{
public : 
    aPad* pad;
    
    // parameters
    float exploreMeanDuration = 5.0;
    float exploreSpeed = 0.5;
    float turnSpeed = 1;

    // state handling
    int state;

    // time
    float time;

    // state working variables
    float exploreDuration;
    float exploreStartTime;

    int turnPreviousState;
    float turnDuration;
    float turnStartTime;
    float turnSign;
    
    float collisionsDecisionLastTime;
    
    // methods
    ControllerAPad (aPad* pad);
    ~ControllerAPad ();

    void Step ();

    void StateExploreInit ();
    void StateExplore ();
    void StateTurnInit (int previousState, float angle);
    void StateTurn ();
    void Reset ();
};


#endif
