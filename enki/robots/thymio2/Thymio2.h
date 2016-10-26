/*
	Enki - a fast 2D robot simulator
	Copyright (C) 1999-2006 Stephane Magnenat <stephane at magnenat dot net>
	Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
	Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
	Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
	Copyright (C) 2006 Laboratory of Robotics Systems, EPFL, Lausanne
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

#ifndef __ENKI_THYMIO2_H
#define __ENKI_THYMIO2_H

#include <enki/robots/DifferentialWheeled.h>
#include <enki/interactions/IRSensor.h>
#include <enki/interactions/GroundSensor.h>

/*!	\file Thymio2.h
	\brief Header of the Thymio robot
*/
	
namespace Enki
{
	//! A simple model of the Thymio robot.
	/*! \ingroup robot */
	class Thymio2 : public DifferentialWheeled
	{
	public:
		//! The infrared sensor 0 (front-left-left)
		IRSensor infraredSensor0;
		//! The infrared sensor 1 (front-left)
		IRSensor infraredSensor1;
		//! The infrared sensor 2 (front-front)
		IRSensor infraredSensor2;
		//! The infrared sensor 3 (front-right)
		IRSensor infraredSensor3;
		//! The infrared sensor 4 (front-right-right)
		IRSensor infraredSensor4;
		//! The infrared sensor 5 (back-left)
		IRSensor infraredSensor5;
		//! The infrared sensor 6 (back-right)
		IRSensor infraredSensor6;
		
		//! The ground sensor 0 (left)
		GroundSensor groundSensor0;
		//! The ground sensor 1 (right)
		GroundSensor groundSensor1;
		
	public:
		//! Create a Thymio II
		Thymio2();
		//! Destructor
		~Thymio2();
	};
}

#endif

