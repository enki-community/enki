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

#include "AsebaMarxbot.h"
#include <common/consts.h>
#include <set>
#include <cassert>
#include <algorithm>

/*!	\file Marxbot.cpp
	\brief Implementation of the aseba-enabled marXbot robot
*/

namespace Enki
{
	AsebaMarxbot::Module::Module(unsigned short id)
	{
		bytecode.resize(512);
		vm.bytecode = &bytecode[0];
		vm.bytecodeSize = bytecode.size();
		
		stack.resize(64);
		vm.stack = &stack[0];
		vm.stackSize = stack.size();
		
		amountOfTimerEventInQueue = 0;
		AsebaVMInit(&vm, id);
	}
	
	// TODO: implement aseba callbacks
	
	AsebaMarxbot::AsebaMarxbot() :
		leftMotor(1),
		rightMotor(2),
		proximitySensors(3),
		distanceSensors(4)
	{
		// try to open a free port at or after ASEBA_DEFAULT_PORT
		const unsigned amoutOfPortsToTry = 100;
		for (unsigned port = ASEBA_DEFAULT_PORT; port < ASEBA_DEFAULT_PORT + amoutOfPortsToTry; port++)
			if (listen(port))
				break;
		
		// setup modules specific data
		leftMotor.vm.variables = reinterpret_cast<sint16 *>(&leftMotorVariables);
		leftMotor.vm.variablesSize = sizeof(leftMotorVariables);
		modules.push_back(&leftMotor);
		
		rightMotor.vm.variables = reinterpret_cast<sint16 *>(&rightMotorVariables);
		rightMotor.vm.variablesSize = sizeof(rightMotorVariables);
		modules.push_back(&rightMotor);
		
		proximitySensors.vm.variables = reinterpret_cast<sint16 *>(&proximitySensorVariables);
		proximitySensors.vm.variablesSize = sizeof(proximitySensorVariables);
		modules.push_back(&proximitySensors);
		
		distanceSensors.vm.variables = reinterpret_cast<sint16 *>(&distanceSensorVariables);
		distanceSensors.vm.variablesSize = sizeof(distanceSensorVariables);
		modules.push_back(&distanceSensors);
	}
	
	void AsebaMarxbot::step(double dt)
	{
		/*
			Values mapping
			
			motor:
				estimated 3000 == 30 cm/s
			
			encoders:
				16 tick per motor turn
				134 reduction
				6 cm wheel diameter
		*/
		
		// set physical variables
		leftSpeed = static_cast<double>(leftMotorVariables.speed) / 100;
		rightSpeed = static_cast<double>(rightMotorVariables.speed) / 100;
		
		// do motion
		DifferentialWheeled::step(dt);
		
		// get physical variables
		int odoLeft = static_cast<int>((leftEncoder * 16  * 134) / (2 * M_PI));
		leftMotorVariables.odoLow = odoLeft & 0xffff;
		leftMotorVariables.odoHigh = odoLeft >> 16;
		
		int odoRight = static_cast<int>((rightEncoder * 16  * 134) / (2 * M_PI));
		leftMotorVariables.odoLow = odoRight & 0xffff;
		leftMotorVariables.odoHigh = odoRight >> 16;
		
		for (size_t i = 0; i < 24; i++)
			proximitySensorVariables.bumpers[i] = static_cast<sint16>(getVirtualBumper(i));
		std::fill(proximitySensorVariables.ground, proximitySensorVariables.ground + 12, 0);
		
		for (size_t i = 0; i < 180; i++)
			distanceSensorVariables.distances[i] = static_cast<sint16>(rotatingDistanceSensor.zbuffer[i]);
		
		// push on timer events
		Event onTimer;
		onTimer.id = ASEBA_EVENT_PERIODIC;
		for (size_t i = 0; i < modules.size(); i++)
		{
			if (modules[i]->amountOfTimerEventInQueue == 0)
			{
				modules[i]->eventsQueue.push_back(onTimer);
				modules[i]->amountOfTimerEventInQueue++;
			}
		}
		
		// process all events
		while (true)
		{
			bool wasActivity = false;
			
			NetworkServer::step();
			
			for (size_t i = 0; i < modules.size(); i++)
			{
				// if events in queue and not blocked in a thread, execute new thread
				if (!modules[i]->eventsQueue.empty() && !AsebaVMIsExecutingThread(&modules[i]->vm))
				{
					// copy event to vm
					BaseVariables *vars = reinterpret_cast<BaseVariables*>(modules[i]->vm.variables);
					const Event &event = modules[i]->eventsQueue.front();
					size_t amount = std::min(event.data.size(), sizeof(vars->args) / sizeof(sint16));
					std::copy(event.data.begin(), event.data.begin() + amount, vars->args);
					AsebaVMSetupEvent(&modules[i]->vm, event.id);
					
					// pop event
					if (event.id == ASEBA_EVENT_PERIODIC)
						modules[i]->amountOfTimerEventInQueue--;
					modules[i]->eventsQueue.pop_front();
				}
				
				// try to run, notice if anything was run
				if (AsebaVMRun(&modules[i]->vm))
					wasActivity = true;
			}
			
			if (!wasActivity)
				break;
		}
	}
	
	void AsebaMarxbot::incomingData(Socket *socket)
	{
		unsigned short len;
		unsigned short type;
		unsigned short source;
		socket->read(&len, 2);
		socket->read(&source, 2);
		socket->read(&type, 2);
		std::valarray<unsigned char> buffer(static_cast<size_t>(len));
		socket->read(&buffer[0], buffer.size());
		
		signed short *dataPtr = reinterpret_cast<signed short *>(&buffer[0]);
		
		if (type < 0x8000)
		{
			// user type queue
			assert(buffer.size() % 2 == 0);
			
			// create event
			Event event;
			event.id = type;
			
			for (size_t i = 0; i < buffer.size(); i++)
				event.data.push_back(*dataPtr++);
			
			// push event in queus
			for (size_t i = 0; i < modules.size(); i++)
				modules[i]->eventsQueue.push_back(event);
		}
		else
		{
			// debug message
			if (type >= 0x9000)
			{
				// not bootloader
				assert(buffer.size() % 2 == 0);
				for (size_t i = 0; i < modules.size(); i++)
					AsebaVMDebugMessage(&modules[i]->vm, type, reinterpret_cast<uint16 *>(dataPtr), buffer.size() / 2);
			}
		}
	}
	
	void AsebaMarxbot::incomingConnection(Socket *socket)
	{
		// do nothing in addition to what is done by NetworkServer
	}
	
	void AsebaMarxbot::connectionClosed(Socket *socket)
	{
		// do nothing in addition to what is done by NetworkServer
	}
}

