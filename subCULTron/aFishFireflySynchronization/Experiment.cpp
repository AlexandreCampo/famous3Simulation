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

// Services
#include "Experiment.h"

#include "RenderOSG.h"
#include "PhysicsBullet.h"
#include "WaterVolume.h"

// Objects
#include "AquariumCircular.h"
#include "aFish.h"

// Controllers
#include "ControllerAFish.h"
#include "Experiment.h"

// Utilities
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <tinyxml.h>

extern gsl_rng* rng;
extern long int rngSeed;

float calculateWaterVolumeHeight(btVector3 pos, float time)
{
    float phase =  10.1 / (2.0 * M_PI);
    float amplitude = 0.15;
    float shift = time / M_PI * 2.0;
    float height = 2.0 + amplitude * (sin(pos.getX() * phase + shift) * sin(pos.getY() * phase + shift));    
    
    return height;    
}

btVector3 calculateWaterVolumeCurrent(btVector3 pos, float time)
{
    return btVector3(0,0,0);
}


Experiment::Experiment (Simulator* simulator, bool graphics)
{
    // random number generation
    init_rng(&rng);        

    // add services
    simulator->setTimestep (0.05);
    physics = new PhysicsBullet();
    physics->setTimestep(0.05);
    simulator->add (physics);
    float waterDensity = 1000.0;
    waterVolume = new WaterVolume(waterDensity, calculateWaterVolumeHeight, calculateWaterVolumeCurrent);
    simulator->add (waterVolume);
    
    render = NULL;
    if (graphics)
    {	
	render = new RenderOSG(simulator);
	simulator->add (render);

	// setup camera of the render service
	render->viewer->getCamera()->setProjectionMatrixAsPerspective(45.0, 1.0, 0.1, 1000); 

	osgGA::SphericalManipulator* manip = dynamic_cast<osgGA::SphericalManipulator*>(render->viewer->getCameraManipulator());
	manip->setDistance(7);
	manip->setCenter(osg::Vec3(0,0,1.0));
	manip->setElevation(90.0 * M_PI / 180.0);
	manip->setHeading(0.0 * M_PI / 180.0);
    }

    // add the experiment so that we can step regularly
    simulator->add (this);
    
    // add aFish
    for (int i = 0; i < aFishCount; i++)
    {
	aFish* r = new aFish ();
	r->registerService(physics);
	r->registerService(waterVolume);
	if (render) 
	{	    
	    r->setMeshFilename ("../3dmodels/aFish.3ds.(0.015,0.015,0.015).scale");
	    r->registerService(render); 
	}
	r->addDevices();
//	r->optical->setDrawable(true);
	r->optical->setReceiveOmnidirectional(true);
	r->optical->setRange(0.5);
	r->setProximitySensorsRange(0.25);	
	r->setDragCoefficients(btVector3( 0.1, 0.25, 0.1), btVector3( 0.05, 0.05, 0.2));

	ControllerAFish* c = new ControllerAFish (r);
	r->add(c);
	c->setTimestep(0.1);

	aFishes.push_back(r);
	simulator->add(r);   	
	
	// position is set in reset
    }

    
    AquariumCircular* aquarium = new AquariumCircular(aquariumRadius,  3.0, 1.0, 40.0);
    aquarium->registerService(physics);
    aquarium->registerService(waterVolume);
    if (render) aquarium->registerService(render);
    simulator->add(aquarium);

    // set last stuff, position of robots mainly
    reset();
}

Experiment::~Experiment()
{
}

void Experiment::reset ()
{
    // reset aFish
    for (unsigned int i = 0; i < aFishes.size(); i++)
    {
	aFish* r = aFishes[i];

	// reset aFish position
	float distance = gsl_rng_uniform(rng) * 0.8 * aquariumRadius;
	float angle = (gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI);
	float x = cos(angle) * distance;
	float y = sin(angle) * distance;
	float z = 0.7 + r->dimensions[2] / 2.0;	

	r->setPosition (btVector3(x, y, z));
	r->setRotation (btQuaternion(btVector3(0, 0, 1), gsl_rng_uniform(rng) * M_PI * 2.0 - M_PI));
	r->ballast->setBuoyancyFactor(0.0);
	r->optical->setRange(0.75);
    }
}

void Experiment::step ()
{       
}

void Experiment::run()
{
    // with render, give up control and stepping is done by render
    if (render)
    {
	render->setPaused (true);
	render->run();
    }
    else
    {
	while(simulator->time < maxTime)
	{
	    simulator->step();
	}
    }
}
