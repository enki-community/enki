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

#ifndef __ENKI_EPUCK_H
#define __ENKI_EPUCK_H

#include <enki/robots/DifferentialWheeled.h>
#include <enki/interactions/IRSensor.h>
#include <enki/interactions/CircularCam.h>
#include <enki/interactions/Bluetooth.h>

/*!	\file EPuck.h
	\brief Header of the E-puck robot
*/
	
namespace Enki
{
	//! The rotating, long range distance sensor turret of the E-puck robot.
	/*! \ingroup interaction 
		The measured physical sensors response function is applied so zbuffer contains the simulated physical values
		*/
	class EPuckScannerTurret : public OmniCam
	{
	public:
		//! Constructor
		/*!
			\param owner robot this camera is attached to
			\param height height of this camera with respect to ground
			\param halfPixelCount half the number of pixel to cover the full 2*PI field of view
			\param minDist minimum scanning distance
			\param maxDist maximum scanning distance
		*/
		EPuckScannerTurret(Robot *owner, double height, unsigned halfPixelCount);
		
		virtual void finalize(double dt, World* w);
	
	public:
		std::valarray<double> scan;
	};
	
	//! A simple model of the E-puck robot.
	/*! \ingroup robot */
	class EPuck : public DifferentialWheeled
	{
	public:
		//! The infrared sensor 0 (front-front-right)
		IRSensor infraredSensor0;
		//! The infrared sensor 1 (front-right)
		IRSensor infraredSensor1;
		//! The infrared sensor 2 (right)
		IRSensor infraredSensor2;
		//! The infrared sensor 3 (back-right)
		IRSensor infraredSensor3;
		//! The infrared sensor 4 (back-left)
		IRSensor infraredSensor4;
		//! The infrared sensor 5 (left)
		IRSensor infraredSensor5;
		//! The infrared sensor 6 (front-left)
		IRSensor infraredSensor6;
		//! The infrared sensor 7 (front-front-left)
		IRSensor infraredSensor7;
		//! Linear camera
		CircularCam camera;
		//! The rotating, long range distance sensor turret
		EPuckScannerTurret scannerTurret;
		//! Bluetooth module
		Bluetooth* bluetooth;
		
	public:
		//! The bot's capabilities. You can simply select a predefined set of sensors. These correspond to the different extension modules that exist for the E-Puck.
		enum Capabilities
		{
			//! No sensor: not very useful
			CAPABILITY_NONE = 0,
			//! Basic_Sensors: Just the 8 IRSensors of the base module
			CAPABILITY_BASIC_SENSORS = 0x1,
			//! Camera: add a linear camera
			CAPABILITY_CAMERA = 0x2,
			//! The rotating, long range distance sensor turret
			CAPABILITY_SCANNER_TURRET = 0x3,
			//! Bluetooth: activate the bluetooth module (Requires the use of Bluetooth master)
			CAPABILITY_BLUETOOTH = 0x4
		};

	public:
		//! Create a E-Puck with certain modules aka capabilities (basic)
		EPuck(unsigned capabilities = CAPABILITY_BASIC_SENSORS);
		//! Destructor
		~EPuck();
		
		//! Set ring color (true = red, false = black) 
		void setLedRing(bool status);
	};
}

#endif

