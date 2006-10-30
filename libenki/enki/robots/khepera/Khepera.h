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

#ifndef __ENKI_KHEPERA_H
#define __ENKI_KHEPERA_H

#include <enki/PhysicalEngine.h>
#include <enki/interactions/IRSensor.h>
#include <enki/interactions/CircularCam.h>

/*!	\file Khepera.h
	\brief Header of the Khepera robot
*/
	
namespace Enki
{
	//! A simple model of the Khepera robot. 
	/*! \ingroup robot */
	class Khepera : public Robot
	{
	public:
		//! The infrared sensor 0 (left)
		IRSensor infraredSensor0;
		//! The infrared sensor 1 (front-left)
		IRSensor infraredSensor1;
		//! The infrared sensor 2 (front)
		IRSensor infraredSensor2;
		//! The infrared sensor 3 (front)
		IRSensor infraredSensor3;
		//! The infrared sensor 4 (front-right)
		IRSensor infraredSensor4;
		//! The infrared sensor 5 (right)
		IRSensor infraredSensor5;
		//! The infrared sensor 6 (back)
		IRSensor infraredSensor6;
		//! The infrared sensor 7 (back)
		IRSensor infraredSensor7;
		//! Linear camera
		CircularCam camera;
		
	public:
		//! The bot's left and right wheel speed [1:100] cm/sec
		double leftSpeed, rightSpeed;
		//! The encoder for left and right wheel speed
		double leftEncoder, rightEncoder;
	
	protected:
		//! Save of last position to compute encoders.
		Vector oldPos;
		//! Save of last angle to compute encoders.
		double oldAngle;
		
	public:
		enum Capabilities
		{
			//! The bot's capabilities. You can simply select a predefined set of sensors. These correspond to the different extension modules that exist for the Khepera.
			CAPABILITIY_NONE = 0,
			//! Basic_Sensors: Just the 8 IRSensors of the base module
			CAPABILITIY_BASIC_SENSORS = 0x1,
			//! Camera: add a linear camera
			CAPABILITY_CAMERA = 0x2
		};

	public:
		//! Create a Khepera with certain modules aka capabilities (basic)
		Khepera(unsigned capabilities = CAPABILITIY_BASIC_SENSORS);
		//! Call Khepera::controlStep and do all the calculations
		void step(double dt);
		
		//! Reset the encoder. Should be called when robot is moved manually.
		void resetEncoders();
	
	};
}

#endif

