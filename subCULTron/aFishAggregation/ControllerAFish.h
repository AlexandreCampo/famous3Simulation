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

    // parameters
    float obstacleAvoidanceThreshold = 0.1;
//    float maxProximitySensing = 0.12;
    float obstacleAvoidanceSpeed = 0.5; // 1
    float exploreMeanDuration = 5.0;
    float exploreSpeed = 0.01; // 0.05
    float restDuration = 30.00;
    float brakeDuration = 0.5;
    float turnSpeed = 0.3;
    float brakeSpeed = 0.05;
    float attractionSpeed = 0.2;

    // state handling
    int state;

    // time
    float time;
    float timestep;

    // state working variables
    float stateDuration;
    float stateStartTime;
    
    int turnPreviousState;
    float turnSign;

    int messagesReceived;
    float msgx;
    float msgy;
    bool attraction = false;
    
    // methods
    ControllerAFish (aFish* fish);
    ~ControllerAFish ();

    void Step ();

    void StateExploreInit ();
    void StateExplore ();
    void StateBrakeInit ();
    void StateBrake ();
    void StateRestInit ();
    void StateRest ();
    void StateTurnInit (int previousState, float angle);
    void StateTurn ();
    void Reset ();
    bool ObstacleAvoidance ();
};


#endif
