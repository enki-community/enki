#ifndef __MAV_COMMUNICATION_H
#define __MAV_COMMUNICATION_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>
#include <enki/robots/mav/MavMessage.h>
/*!
	\file MavCommunication.h
	\brief Header of the mav communication interaction
*/

namespace Enki
{
	//! Communication module for the Mav
	/*! In an mav, the communication is done via messages sent through a wireless RF link.
		\ingroup interaction
	*/
class MavCommunication : public LocalInteraction
	{
	protected:
		//! Is communication active
		bool enabled;
		//! Value to transmit through communication
		MavMessage *transmitValue;
		
		//! Was communication active last step
		bool wasReceived;
		//! Value received through communication
		std::vector<MavMessage *> receivedValue;
	public:
		//! Constructor; r is the mav's maximum communication range
		MavCommunication(double r, Robot *owner);
		//! Init routine run before the interaction
		void init();
		//! The actual interaction
		void objectStep(double dt, PhysicalObject *po, World *w);
		//! The value to transmit
		void setTransmitValue(MavMessage *val);
		//! The value to transmit
		MavMessage *getTransmitValue(void);
		//! Turn transmission on or off
		void setTransmit(bool enabled);
		//! Returns true if a communication signal was received
		bool wasCommunication(void);
		//! Returns true if a communication signal was sent
		bool wasCommunicationSent(void);
		//! Returns the received messages
		std::vector<MavMessage *> getSensorValue(void);
	};
}

#endif

