/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2013 Stephane Magnenat <stephane at magnenat dot net>
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

#include "enki/robots/marxbot/Marxbot.h"
#include <cassert>

/*!	\file Marxbot.cpp
	\brief Implementation of the marXbot robot
*/

namespace Enki
{
	// TODO: use similar function as for distance sensors
	// if we were to use IRSensors, the parameters would be
	// around m=3000, x0=0.2, c=1
	double marxbotVirtualBumperResponseFunction(double dist)
	{
		if (dist<0.5)
			dist = -440*dist+3000;
		else if (dist>=0.5 && dist<=9)
			dist = 4526*exp(-0.9994*dist);
		else
			dist = random.getRange(20.0);
		
		dist *= (0.97+random.getRange(0.06));
		
		return dist;
	}
	
	Marxbot::Marxbot() :
		DifferentialWheeled(15, 30, 0.02),
		rotatingDistanceSensor(this, 11, 90)
	{
		addLocalInteraction(&rotatingDistanceSensor);
		
		setCylindric(8.5, 12, 1000);
		setColor(Color(0.7, 0.7, 0.7));
	}
	
	double Marxbot::getVirtualBumper(unsigned number)
	{
		assert(number < 24);
		unsigned physicalNumber = (24 + 12 - number) % 24;
		return marxbotVirtualBumperResponseFunction(sqrt(rotatingDistanceSensor.zbuffer[(physicalNumber * 180) / 24]) - getRadius());
	}
}

