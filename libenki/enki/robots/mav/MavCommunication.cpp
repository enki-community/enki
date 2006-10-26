#include "MavCommunication.h"
#include "Mav.h"

#include <iostream>
#include <sstream>
#include <limits>

#include <assert.h>

/*! \file MavCommunication.cpp
	\brief Implementation of the mav's communication interaction 
*/

namespace Enki
{
	using namespace An;
	
	MavCommunication::MavCommunication(double r, Robot *owner)
	{
		this->r = r;
		this->owner =static_cast<Mav *>(owner);
		enabled = false;
		MavMessage *com = new MavMessage(-1,-1,0.0, false,"",-1,1000,20);
		receivedValue.resize(0);
		transmitValue = com;
		wasReceived = false;
	}
 
	void MavCommunication::init()
	{
		wasReceived = false;
		receivedValue.clear();
	}
	
	
	void MavCommunication::objectStep(double dt, PhysicalObject *po, World *w)
	{
		Mav *other = static_cast<Mav *>(po);
	
		if ((other) && (other->comm.enabled))
		{
			//test if message already received
				double dist = (owner->pos - other->pos).norm();
				if (dist < r*(0.9 + random.getRange(0.1)))
				{
					receivedValue.push_back(other->comm.transmitValue);
					wasReceived = true;
					other->comm.receivedValue.push_back(transmitValue);
					other->comm.wasReceived=true;
				}
				else
				{
					wasReceived = false;
					other->comm.wasReceived=false;
				}
		}
	}

	void MavCommunication::setTransmitValue(MavMessage *val)
	{
		transmitValue = val;
	}

	MavMessage * MavCommunication::getTransmitValue(void)
	{
		return transmitValue;
	}
	void MavCommunication::setTransmit(bool enabled)
	{
		this->enabled = enabled;
	}
	
	bool MavCommunication::wasCommunication(void)
	{
		return wasReceived;
	}
		
	bool MavCommunication::wasCommunicationSent(void)
	{
		return enabled;
	}
	
	std::vector<MavMessage *> MavCommunication::getSensorValue(void)
	{
		return receivedValue;
	}
}
