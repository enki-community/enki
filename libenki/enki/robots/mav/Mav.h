#ifndef __MAV_H
#define __MAV_H

#include <enki/PhysicalEngine.h>
#include <enki/interactions/IRSensor.h>
#include <enki/interactions/CircularCam.h>
#include <enki/robots/mav/MavCommunication.h>

/*!	\file Mav.h
	\brief Header of the Mav robot
*/
	
namespace Enki
{
	//! A simple model of the Mav robot. 
	/*! \ingroup robot */
	class Mav : public Robot
	{
	public:
		MavCommunication comm;
		//! The planes desired roll, current roll and speed m/s
		double roll, forwardSpeed, currentRoll;
		bool currentRollSet;
		enum Capabilities
		{
			//! The bot's capabilities. You can simply select a predefined set of sensors. These correspond to the different extension modules that exist for the Mav.
			CAPABILITIY_NONE = 0,
			//! Basic_Sensors: Just the n communication signals of the base module
			CAPABILITIY_BASIC_SENSORS = 0x1,
		};
		virtual ~Mav();
		//! Create a Mav with certain modules aka capabilities (basic)
		Mav(unsigned capabilities = CAPABILITIY_BASIC_SENSORS);
		//! Call Mav::controlStep and do all the calculations
		void step(double dt);
	};
}

#endif

