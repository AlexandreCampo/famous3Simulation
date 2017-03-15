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

#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "Simulator.h"
#include "Service.h"
#include "Gsl.h"

#include <vector>
#include <fstream>


class PhysicsBullet;
class WaterVolume;
class RenderOSG;
class aFish;

class Experiment : public Service
{
public:

    // services
    PhysicsBullet* physics;
    WaterVolume* waterVolume;
    RenderOSG* render;

    // objects
    std::vector<aFish*> aFishes;
	
    // parameters
    int aFishCount = 100;
    float maxTime = 3600;
    float aquariumRadius = 3.0;    
   
    // methods
    Experiment (Simulator* s, bool graphics);
    ~Experiment ();

    void reset ();
    void step ();
    void run();
};


#endif
