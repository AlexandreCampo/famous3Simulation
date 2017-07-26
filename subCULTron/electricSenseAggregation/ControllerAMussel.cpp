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

#include "ControllerAMussel.h"

#include "Simulator.h"

#include <cmath>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
extern gsl_rng* rng;
extern long int rngSeed;


ControllerAMussel::ControllerAMussel (aMussel* mussel)
{
    this->mussel = mussel;
    
    reset ();
}


ControllerAMussel::~ControllerAMussel ()
{
}

void ControllerAMussel::reset ()
{
    lastTime = 0.0;
    factor = -1;
}

void ControllerAMussel::step ()
{
    
    Eigen::VectorXf pola(5);
    // passif
 /*   pola << 0, 0, 0, 0, 0;
    mussel->esense->setPolarization (pola);
    // read e-sense
    mussel->esense->getCurrents();
    
    if (mussel->esense->I(0)==0)
    {*/
    // actif
    pola << 10, 0, 0, 0, 0;
    mussel->esense->setPolarization (pola);
    // read e-sense
    mussel->esense->getCurrents();
    mussel->setColor(1, 0, 0);
 /*   }
    else 
    {
    mussel->setColor(1, 1, 55.0/254.0);
    }*/

 /*   cout << "E-sense current measured : ";    
    for (int i = 0; i < mussel->esense->numElectrodes; i++)
    {
	cout << mussel->esense->I(i) << " ";
    }
    cout << endl;*/

/*if (mussel->simulator->time - lastTime >= 10.0) 
    {
	lastTime = mussel->simulator->time;

	if (factor < -0.5) factor = 1.0;
	else factor = -1.0;

	mussel->ballast->setBuoyancyFactor(factor);
    }*/

}
