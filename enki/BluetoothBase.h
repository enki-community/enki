/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
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

#ifndef __ENKI_BLUETOOTHBASE_H
#define __ENKI_BLUETOOTHBASE_H

#include "PhysicalEngine.h"

#include <valarray>
#include <list>
#include <queue>


/*!	\file BluetoothBase.h
	\brief Header of the bluetooth base
*/

namespace Enki
{
	class Bluetooth;

	//! Implementation of a Bluetooth base coordinating the Bluetooth modules
	/*! \ingroup interaction */
	class BluetoothBase
	{
	protected:
		//! Structure associating a pointer to a bluetooth module to its address
		struct BtClients
		{
			//! Pointer to a bluetooth module
			Bluetooth* owner;
			//! Address of the associated bluetooth module
			unsigned address;
		};
		
		//! Information needed to establish a connection or a disconnection
		struct Connections
		{
			//! Pointer to the Bluetooth module acting as the source
			Bluetooth* source;
			//! Destination address of the connection
			unsigned destaddress;
		};
		
		//! Information needed to send data along an established connection
		struct Transmissions
		{
			//! Pointer to the module sending the data
			Bluetooth* source;
			//! Address of the module receiving the data
			unsigned address;
			//! Pointer to a buffer containing the data to be sent
			char* data;
			//! Size in byte of the data to be sent
			unsigned size;
		};
		
		//! List of registered Bluetooth module
		std::list<BtClients> clients;
		//! Queue of the connection to be established
		std::queue<Connections> connectbuffer;
		//! Queue of the connection to be closed
		std::queue<Connections> disconnectbuffer;
		//! Queue of the data to be transmitted
		std::queue<Transmissions> transmissions;
		
		//! Execute the previously scheduled transfer of data
		bool bbSendDataTo(Bluetooth* source, unsigned address, char* data, unsigned size);
		//! Execute the previously scheduled connections
		bool bbConnectTo(Bluetooth* source,unsigned address);
		//! Execute the previously scheduled disconnections
		bool bbCloseConnection(Bluetooth* source,unsigned address);
		
		//! Return the pointer of the Bluetooth module associated with "address"
		Bluetooth* getAddress(unsigned address);
		//! Check if the distance between the two modules is small enough for communication
		bool checkDistance(Bluetooth* source, Bluetooth* destination);
		
	public:
		//! Bluetooth transmission errors
		enum Errors
		{
			//! No error occured during the last step
			BT_NO_ERROR = 0,
			//! The address used was unknown
			ADDRESS_UNKNOWN = 1,
			//! The distance between the 2 robots is too great
			DISTANCE_EXCEEDED = 2,
			//! No additional connection can be made as the robot is already at maximum
			TOO_MANY_CONNECTIONS = 3,
			//! The reception buffer is full and no additional data can be written
			RECEPTION_BUFFER_FULL = 4
		};
		
		//! Constructor
		BluetoothBase();
		//! Destructor
		virtual ~BluetoothBase();
		
		//! Register a module Bluetooth with its associated address
		bool registerClient(Bluetooth* owner, unsigned address);
		//! Remove a previously registered Bluetooth module
		bool removeClient(Bluetooth* owner);
		
		//! Schedule a transmission of data to be sent during the next step
		void sendDataTo(Bluetooth* source, unsigned address, char* data, unsigned size);
		//! Schedule a connection to another Bluetooth module
		void connectTo(Bluetooth* source,unsigned address);
		//! Schedule a disconnection between two Bluetooth Module
		void closeConnection(Bluetooth* source,unsigned address);
		
		//! Execute the previously scheduled operations.
		virtual void step(double dt, World *w);
	};

}

#endif
