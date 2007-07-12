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

#ifndef __ENKI_ASEBA_MARXBOT_H
#define __ENKI_ASEBA_MARXBOT_H

#include <enki/robots/marxbot/Marxbot.h>
#define ASEBA_ASSERT
#include <vm/vm.h>
#include <common/consts.h>
#include <utils/network.h>
#include <deque>

/*!	\file AsebaMarxbot.h
	\brief Header of the aseba-enabled marXbot robot
*/

namespace Enki
{
	
	//! The aseba-enabled version of the marXbot robot.
	/*! This robot provides a full simulation of event-based architecture
		as present on the real marXbot, using aseba.
		This robot provides a tcp server connection on port ASEBA_DEFAULT_PORT
		(currently 33333), or higher if other AsebaMarxbot already use it.
		This allows the connection of Aseba Studio to this robot.
		The feature is provided by inheriting from a Network server
		\ingroup robot
	*/
	class AsebaMarxbot : public Marxbot, public Aseba::NetworkClient
	{
	protected:
		struct Event
		{
			unsigned short id;
			std::vector<signed short> data;
		};
		
		struct Module
		{
			Module(unsigned short id);
			
			AsebaVMState vm;
			std::valarray<unsigned short> bytecode;
			std::valarray<signed short> stack;
			std::deque<Event> eventsQueue;
			unsigned amountOfTimerEventInQueue;
		};
		
		struct BaseVariables
		{
			sint16 args[32];
		};
		
		struct MotorVariables
		{
			sint16 args[32];
			sint16 speed;
			sint16 odo[2];
			sint16 user[220];
		};
		
		struct ProximitySensorVariables
		{
			sint16 args[32];
			sint16 bumpers[24];
			sint16 ground[12];
			sint16 user[188];
		};
		
		struct DistanceSensorVariables
		{
			sint16 args[32];
			sint16 distances[180];
			sint16 user[44];
		};
		
		MotorVariables leftMotorVariables;
		MotorVariables rightMotorVariables;
		ProximitySensorVariables proximitySensorVariables;
		DistanceSensorVariables distanceSensorVariables;
		
		Module leftMotor;
		Module rightMotor;
		Module proximitySensors;
		Module distanceSensors;
		
		std::vector<Module *> modules;
		
	public:
		//! Constructor, connect to a host and register VMs
		AsebaMarxbot(const std::string &host = "localhost", unsigned short port = ASEBA_DEFAULT_PORT);
		//! Destructor, unregister VMs
		virtual ~AsebaMarxbot();
		//! In addition to DifferentialWheeled::step(), update aseba variables and initiate periodic events.
		virtual void step(double dt);
		
		virtual void connectionEstablished();
		virtual void incomingData();
		virtual void connectionClosed();
	};

}
#endif

