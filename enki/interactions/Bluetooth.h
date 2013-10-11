/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2013 Stephane Magnenat <stephane at magnenat dot net>
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

#ifndef __ENKI_BLUETOOTH_H
#define __ENKI_BLUETOOTH_H

#include "../PhysicalEngine.h"
#include "../Interaction.h"


#include <queue>

/*!	\file Bluetooth.h
\brief Header of the bluetooth module
*/

namespace Enki
{	

	//! Implementation of an onboard Bluetooth module
	/*! \ingroup interaction */
	class Bluetooth: public GlobalInteraction
	{
protected:
		friend class BluetoothBase;
		
		//! Range of the interaction
		double range;
		
		//! Number of connections currently established
		unsigned nbConnections;
		//! Maximum number of simultaneous connections supported
		unsigned maxConnections;
		//! Address of the Bluetooth module
		unsigned address;
		
		//! Reception buffers for data coming from other connected modules
		char** rxBuffer;
		//! Transmission buffers for data sent to other connected modules
		char** txBuffer;
		//! Size of each buffer for the reception of data
		unsigned rxBufferSize;
		//! Size of each buffer for the transmission of data
		unsigned txBufferSize;
		//! Flags signalling the reception of data
		bool* receptionFlags;
		//! Addresses of the connected modules
		unsigned* destAddress;
		//! Size of the data to send
		unsigned* sizeToSend;
		//! Size of the data received
		unsigned* sizeReceived;
		
		//! Flag indicating a change in the address of the module
		bool updateAddress;
		//! Flag indicating that the current address has been assigned randomly
		bool randomAddress;
		
		//! Queue containing request for connection to other modules
		std::queue<unsigned> connectToRobot;
		//! Queue containing request for closing the connection with other modules
		std::queue<unsigned> closeConnectionToRobot;
		
		//! Flags indicating transmission errors
		unsigned* transmissionError;
		//! Flag indicating an error involving the connection toward another robot
		char connectionError;
		//! Flag indicating an error involving the disconnection from another robot
		char disconnectionError;
		
		//! Deallocate the memory dedicated for the reception buffer
		void cancelRxBuffer();
		//! Deallocate the memory dedicated for the transmission buffer
		void cancelTxBuffer();
		//! Deallocate all the memory
		void cancelAllData();
		//! Initialise all the data structure requires by the module
		void initAllData();
		
public:
		//! Error that bluetooth communication can produce
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
		//! e.g.: "bluetooth(this,10000,7,100,10,1)" for a module of address 1 with a range of 10 meters, 7 supporting simultaneous connections capable of receiving packets of 100 bytes and emitting packets of 10 bytes.
		Bluetooth(Robot* owner,double range, unsigned maxConnections, unsigned rxbuffersize, unsigned txbuffersize,unsigned address);
		//! Destructor
		virtual ~Bluetooth();
		
		//! On every timestep, send the commands recorded to the bluetooth Base to be executed
		virtual void step(double dt, World *w);
		
		//! Change the address of the module
		void setAddress(unsigned address);
		//! Return the address of the module
		unsigned getAddress();
		
		//! Initiate a connection with another module
		void connectTo(unsigned address);
		//! Close an established connection with another module
		bool closeConnection(unsigned index);
		
		//! Indicate if data was received during the last step
		bool didIReceive();
		//! Indicate if data from the module having the address "source" was received during the last step
		bool didIReceive(unsigned source);
		//! Return the reception flags indicating from which module data was received
		bool* getReceptionFlags();
		//! Return the reception buffer associated with another module of address "source"
		const char* getRxBuffer(unsigned source);
		//! Return the amount of data received from another module of address "source" during the last step
		unsigned getSizeReceived(unsigned source);
		
		//! Send data to the module of address "dest"
		bool sendDataTo(unsigned dest,char* data,unsigned size);
		//! Return the flags indicating on which connection a transmission error occured
		unsigned* getTransmissionError();
		//! Indicate if an error of transmission occured during the last step 
		bool isThereTxError();
		
		//! Return the status of the connection error flag
		unsigned getConnectionError();
		//! Return the status of the disconnection error flag
		unsigned getDisconnectionError();
		
		//! Return the size of the transmission buffers
		unsigned getTxBufferSize();
		//! Modify the size of the transmission buffers
		void changeTxBufferSize(unsigned size);
		
		//! Return the size of the reception buffers
		unsigned getRxBufferSize();
		//! Modify the size of the reception buffers
		void changeRxBufferSize(unsigned size);
		
		//! Return the maximum number of simultaneous connections supported by this module
		unsigned getMaxConnections();
		//! Change the maximum number of simultaneous connections supported by this module
		void changeMaxConnections(unsigned size);
		
		//! Return the number of established connections to other modules
		unsigned getNbConnections();
		//! Return the addresses of the other connected modules
		unsigned* getConnectedAddresses();
		
	};
	
}

#endif
