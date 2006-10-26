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

#include "AliceCommunication.h"
#include "Alice.h"

#include <iostream>
#include <sstream>
#include <limits>

#include <assert.h>

/*!
	\file AliceCommunication.cpp
	\brief Implementation of the Alice communication interaction
*/

namespace Enki
{
	AliceCommunication::AliceCommunication(double r, Robot *owner)
	{
		this->r = r;
		this->owner = owner;
		enabled = false;
		transmitValue = 0;
	}
	
	void AliceCommunication::init()
	{
		wasReceived = false;
		receivedSensor = 0;
		receivedValue = 0;
		updatedRealSensors = false;
		smallestDistance2 = std::numeric_limits<double>::max();
	}
	
	void AliceCommunication::computeRealSensors(void)
	{
		if (!updatedRealSensors)
		{
			realSensors.resize(sensors.size());
			for (size_t i=0; i<sensors.size(); i++)
			{
				Matrix22 rot(owner->angle);
				realSensors[i] = owner->pos + rot * sensors[i].first;
			}
			updatedRealSensors = true;
		}
	}
	
	void AliceCommunication::objectStep(double dt, PhysicalObject *po, World *w)
	{
		Alice *other = dynamic_cast<Alice *>(po);
		
		if ((other) && (other->comm.enabled))
		{
			// compute real
			computeRealSensors();
			other->comm.computeRealSensors();
			
			assert(realSensors.size() == sensors.size());
			assert(other->comm.realSensors.size() == other->comm.sensors.size());
			
			// For each sensor in each robot, check communication
			for (size_t i=0; i<sensors.size(); i++)
				for (size_t j=0; j<other->comm.sensors.size(); j++)
				{
					double dist2 = (realSensors[i] - other->comm.realSensors[j]).norm2();
					if ( (dist2 < smallestDistance2) && 
						(dist2 < (sensors[i].second * sensors[i].second) + (other->comm.sensors[j].second * other->comm.sensors[j].second) ) )
					{
						// We have communication
						receivedSensor = i;
						receivedValue = other->comm.transmitValue;
						wasReceived = true;
						smallestDistance2 = dist2;
					}
				}
		}
	}

	void AliceCommunication::addSensor(Vector pos, double ray)
	{
		sensors.push_back(std::pair<Vector, double>(pos, ray));
	}
	
	void AliceCommunication::setTransmitValue(unsigned val)
	{
		transmitValue = val;
	}
	
	void AliceCommunication::setTransmit(bool enabled)
	{
		this->enabled = enabled;
	}
	
	bool AliceCommunication::wasCommunication(void)
	{
		return wasReceived;
	}
		
	bool AliceCommunication::wasCommunicationSent(void)
	{
		return enabled;
	}
	
	unsigned AliceCommunication::getSensorId(void)
	{
		return receivedSensor;
	}
	
	unsigned char AliceCommunication::getSensorValue(void)
	{
		return receivedValue;
	}
}
