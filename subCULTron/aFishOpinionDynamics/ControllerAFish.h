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
    float obstacleAvoidanceThreshold = 0.05;
//    float maxProximitySensing = 0.12;
    float obstacleAvoidanceSpeed = 0.99;
    float exploreMeanDuration = 5.0;
    float exploreSpeed = 0.05;
    float turnSpeed = 0.3;
    float breakSpeed = 1;

    int opinion;
    float confidence;
    float fermiCoeff = 7;
    float blinkProba = 0.25;
    
    
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
    ControllerAFish (aFish* fish);
    ~ControllerAFish ();

    void Step ();

    void DiffuseAndUpdateOpinion();
    void StateExploreInit ();
    void StateExplore ();
    void StateTurnInit (int previousState, float angle);
    void StateTurn ();
    void Reset ();
    bool ObstacleAvoidance ();
};


#endif
