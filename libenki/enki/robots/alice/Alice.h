/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2006 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <antoine dot beyeler at epfl dot ch>
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

#ifndef __ALICE_H
#define __ALICE_H

#include <enki/PhysicalEngine.h>
#include <enki/interactions/IRSensor.h>
#include <enki/interactions/CircularCam.h>
#include <enki/robots/alice/AliceCommunication.h>

/*!	\file Alice.h
	\brief Header of the Alice robot
*/
	
namespace Enki
{
	//! A simple model of the Alice robot. 
	/*! \ingroup robot */
	class Alice : public Robot
	{
	protected:
		//! Container for sensor models for Alice
		struct SensorModels
		{
			SensorResponseFunctor *frontleft[3];
			SensorResponseFunctor *frontright[3];
			SensorResponseFunctor *normal[3];
			SensorResponseFunctor *high[3];
			
			SensorModels();
			~SensorModels();
		} sensorModels;
	
	public:
		//! Left InfraRed Sensor
		IRSensor left;
		//! Front InfraRed Sensor
		IRSensor front;
		//! Right InfraRed Sensor
		IRSensor right;
		//! Back InfraRed Sensor
		IRSensor back;
		//! Front High InfraRed Sensor of the ANTS project extension module, a little outside the Alice
		IRSensor frontHighExt;
		//! Back Left InfraRed Sensor
		IRSensor backleft;
		//! Back Right InfraRed Sensor
		IRSensor backright;
		//! The circular camera adapted for our humble needs
		CircularCam circcam;
		//! Communication module
		AliceCommunication comm;
		//!Polygone for alice
		Polygone AP;


	public:
		//! The bot's left and right wheel speed
		double leftSpeed, rightSpeed;
		
	public:
		enum Capabilities
		{
			//! The bot's capabilities. You can simply select a predefined set of sensors. These correspond to the different extension modules that exist for the Alice.
			CAPABILITIY_NONE = 0,
			//! Basic_Sensors: Just the 4 IRSensors of the base module
			CAPABILITIY_BASIC_SENSORS = 0x1,
			//! Camera: Adds the camera
			CAPABILITIY_CAMERA = 0x2,
			//! Communication: The Alice can use the IR-Sensors to communicate
			CAPABILITIY_COMMUNICATION = 0x4,
			//! External Module High Sensor: The Alice is equipped with the ANTS extension module and uses the high sensor
			CAPABILITIY_EXT_HIGH_SENSOR = 0x8,
			//! External Module Back Left and Right Sensor: The Alice is equipped with the ANTS extension module and uses the back left and right sensors
			CAPABILITIY_EXT_BACK_LR_SENSORS = 0x10,
		};

	public:
		//! Create an Alice with certain modules aka capabilities (basic, advanced, camera, communication, sound)
		Alice(unsigned capabilities = CAPABILITIY_BASIC_SENSORS);
		//! Call Alice::controlStep and do all the calculations
		void step(double dt);
	};
}

#endif

