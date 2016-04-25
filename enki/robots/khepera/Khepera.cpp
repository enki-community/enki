/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "Khepera.h"

/*! \file Khepera.cpp
	\brief Implementation of the Khepera robot
*/
namespace Enki
{
	Khepera::Khepera(unsigned capabilities) :
		DifferentialWheeled(5.2, 100, 0.05),
		infraredSensor0(this, Vector(1.0, 1.5),  1.8, M_PI/2, 10, 1200, -0.9, 7, 20),
		infraredSensor1(this, Vector(1.3, 1.3),  1.8, M_PI/4, 10, 1200, -0.9, 7, 20),
		infraredSensor2(this, Vector(1.6, 0.6),  1.8, 0,      10, 1200, -0.9, 7, 20),
		infraredSensor3(this, Vector(1.6, -0.6), 1.8, 0,      10, 1200, -0.9, 7, 20),
		infraredSensor4(this, Vector(1.3, -1.3), 1.8, -M_PI/4,10, 1200, -0.9, 7, 20),
		infraredSensor5(this, Vector(1.0, -1.5), 1.8, -M_PI/2,10, 1200, -0.9, 7, 20),
		infraredSensor6(this, Vector(-1.5, -1.0),1.8, -M_PI,  10, 1200, -0.9, 7, 20),
		infraredSensor7(this, Vector(-1.5, 1.0), 1.8, -M_PI,  10, 1200, -0.9, 7, 20),
		camera(this, Vector(0, 0), 0, 0.0, M_PI/4, 50)
	{
		if (capabilities & CAPABILITIY_BASIC_SENSORS)
		{
			addLocalInteraction(&infraredSensor0);
			addLocalInteraction(&infraredSensor1);
			addLocalInteraction(&infraredSensor2);
			addLocalInteraction(&infraredSensor3);
			addLocalInteraction(&infraredSensor4);
			addLocalInteraction(&infraredSensor5);
			addLocalInteraction(&infraredSensor6);
			addLocalInteraction(&infraredSensor7);
		}
		
		if (capabilities & CAPABILITY_CAMERA)
		{
			addLocalInteraction(&camera);
		}
		
		setCylindric(2.6, 5, 80);
	}
}

