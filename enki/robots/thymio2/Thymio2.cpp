/*
	Enki - a fast 2D robot simulator
	Copyright (C) 1999-2008 Stephane Magnenat <stephane at magnenat dot net>
	Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
	Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
	Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
	Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
	See AUTHORS for details

	This program is free software; the authors of any publication 
	arising from research using this software are asked to add the 
	following reference:
	Enki - a fast 2D robot simulator
	http://lis.epfl.ch/enki
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

#include "Thymio2.h"
#include <algorithm>
#include <functional>
#include <climits>
#include <math.h>

/*! \file Thymio2.cpp
	\brief Implementation of the Thymio II robot
*/
namespace Enki
{
	using namespace std;
	
	Thymio2::Thymio2() :
		DifferentialWheeled(9.4, 16.6, 0.027),
		infraredSensor0(this, Vector(6.2, 4.85),   3.4, 0.69813,  14, 4505, 0.03, 73, 2.87),
		infraredSensor1(this, Vector(7.5, 2.55),   3.4, 0.34906,  14, 4505, 0.03, 73, 2.87),
		infraredSensor2(this, Vector(7.95, 0.0),   3.4, 0.0,      14, 4505, 0.03, 73, 2.87),
		infraredSensor3(this, Vector(7.5, -2.55),  3.4, -0.34906, 14, 4505, 0.03, 73, 2.87),
		infraredSensor4(this, Vector(6.2, -4.85),  3.4, -0.69813, 14, 4505, 0.03, 73, 2.87),
		infraredSensor5(this, Vector(-2.95, 2.95), 3.4, -M_PI,    14, 4505, 0.03, 73, 2.87),
		infraredSensor6(this, Vector(-2.95, -2.95),3.4, -M_PI,    14, 4505, 0.03, 73, 2.87),
		groundSensor0(this, Vector(7.2, 1.2),  806.93, 45, 0.3, 10),
		groundSensor1(this, Vector(7.2, -1.2), 806.93, 45, 0.3, 10)
	{
		// add interactions
		addLocalInteraction(&infraredSensor0);
		addLocalInteraction(&infraredSensor1);
		addLocalInteraction(&infraredSensor2);
		addLocalInteraction(&infraredSensor3);
		addLocalInteraction(&infraredSensor4);
		addLocalInteraction(&infraredSensor5);
		addLocalInteraction(&infraredSensor6);
		addLocalInteraction(&groundSensor0);
		addLocalInteraction(&groundSensor1);
		
		//staticFrictionThreshold = 0.5;
		dryFrictionCoefficient = 0.25;
		dryFrictionCoefficient = 2.5;
		
		// define the physical shape of the Thymio
		Enki::Polygone thymio2Shape;
		const double amount = 10.0;
		const double radius = 8.0;
		const double height = 5.1;
		const double angle1 = asin(5.5/8.0);
		const double angle2 = atan(5.5/3.0);
		const double distance = sqrt(3.0*3.0+5.5*5.5);
		for (double a = -angle1; a < angle1+0.01; a += 2*angle1/amount)
			thymio2Shape.push_back(Enki::Point(radius * cos(a), radius * sin(a)));        
		thymio2Shape.push_back(Enki::Point(distance * cos(M_PI - angle2), distance * sin(M_PI - angle2)));
		thymio2Shape.push_back(Enki::Point(distance * cos(M_PI - angle2), distance * sin(M_PI + angle2)));
		Enki::PhysicalObject::Hull hull(Enki::PhysicalObject::Part(thymio2Shape, height));
		setCustomHull(hull, 200);
		setColor(Color(0.98, 0.98, 0.98));
	}
	
	Thymio2::~Thymio2()
	{
	}
}

