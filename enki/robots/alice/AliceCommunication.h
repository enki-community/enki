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

#ifndef __ENKI_ALICECOMMUNICATION_H
#define __ENKI_ALICECOMMUNICATION_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>

/*!
	\file AliceCommunication.h
	\brief Header of the Alice communication interaction
*/

namespace Enki
{
	//! Communication module for the Alice
	/*! In the real alice, the communication is done via the infrared sensors. Here it is a different interaction
		\ingroup interaction
	*/
	class AliceCommunication : public LocalInteraction
	{
	protected:
		//! The position of sensors through which the Alices communicate
		std::vector<std::pair<Vector, double> > sensors;
		//! The actual position of sensors, transformed by robot position and orientation
		std::vector<Vector> realSensors;
		//! True if the realSensors reflect the actual real position of the robot
		bool updatedRealSensors;
		
		//! Compute the real position of sensors, update realSensors
		void computeRealSensors(void);
	
		//! Is communication active
		bool enabled;
		//! Value to transmit through communication
		unsigned transmitValue;
		
		//! Was communication active last step
		bool wasReceived;
		//! Sensor through which we received value
		unsigned receivedSensor;
		//! Value received through communication
		unsigned receivedValue;
		//! the smallest squared distance between two robots
		double smallestDistance2;
		 
	public:
		//! Constructor; r is the robot's bounding circle radius
		AliceCommunication(double r, Robot *owner);
		//! Init routine run before the interaction
		void init();
		//! The actual interaction
		void objectStep(double dt, PhysicalObject *po, World *w);
		
		//! Add a new sensor at position pos of radius ray
		void addSensor(Vector pos, double ray);
		//! The value to transmit
		void setTransmitValue(unsigned val);
		//! Turn transmission on or off
		void setTransmit(bool enabled);
		//! Returns true if a communication signal was received
		bool wasCommunication(void);
		//! Returns true if a communication signal was sent
		bool wasCommunicationSent(void);
		//! Returns the Id 
		unsigned getSensorId(void);
		//! Returns the received SensorValue
		unsigned char getSensorValue(void);
	};
}

#endif

