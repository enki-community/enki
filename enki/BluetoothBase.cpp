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

#include "interactions/Bluetooth.h"
#include "BluetoothBase.h"

#include <limits.h>
#include <assert.h>

/*!	\file BluetoothBase.cpp
	\brief Implementation of the bluetooth base
*/

namespace Enki
{
	BluetoothBase::BluetoothBase()
	{
		
	}
	
	BluetoothBase::~BluetoothBase()
	{
		
	}
	
	Bluetooth* BluetoothBase::getAddress(unsigned address)
	{
		std::list<BtClients>::iterator it;
		for (it=clients.begin(); it!=clients.end() && (*it).address!=address; ++it);
		
		if (it!=clients.end())
			return (*it).owner;
		else
			return NULL;
	}
	
	bool BluetoothBase::registerClient(Bluetooth* owner, unsigned address)
	{
		BtClients btc;
		btc.owner=owner;
		btc.address=address;
		
		
		// Look if this address has already been assigned
		std::list<BtClients>::iterator it;
		for (it=clients.begin(); it!=clients.end() && (*it).address!=address; ++it);
		
		if (it!=clients.end())
			return false;
		
		// Look for an address for this robot
		for (it=clients.begin(); it!=clients.end() && (*it).owner!=owner; ++it);
		
		if (it == clients.end())
			clients.push_back(btc);
		else
		{
			(*it).address = address;
		}
		return true;
	}
	
	bool BluetoothBase::removeClient(Bluetooth* owner)
	{
		std::list<BtClients>::iterator it;
		for (it=clients.begin(); it!=clients.end() && (*it).owner!=owner; ++it);
		
		if (it != clients.end())
		{
			it = clients.erase(it);
			return true;
		}
		else
			return false;
	}
	
	bool BluetoothBase::bbSendDataTo(Bluetooth* source, unsigned address, char* data, unsigned size)
	{
		Bluetooth* destination = getAddress(address);
		unsigned i=0, j=0;
		
		if (destination && checkDistance(source,destination))
		{
			for (i=0; i < source->maxConnections && source->destAddress[i] != address; ++i);
			if (i == source->maxConnections)
			{
				// The source has the connection.
				source->connectionError=ADDRESS_UNKNOWN;
				return false;
			}
			else
			{
				for (j=0; j < destination->maxConnections && destination->destAddress[j] != source->address; ++j);
				if (j == destination->maxConnections)
				{
					// The source has the connection.
					source->connectionError=ADDRESS_UNKNOWN;
					return false;
				}
				
				unsigned q;
				for (q=0; q<size && q<destination->rxBufferSize; ++q)
					destination->rxBuffer[j][q] = data[q];
				
				destination->sizeReceived[j] = q;
				source->sizeToSend[i] = q-size;
				source->transmissionError[i] = q<size ? RECEPTION_BUFFER_FULL : BT_NO_ERROR;
				destination->receptionFlags[j] = true;
				return q < size;
			}
		}
		else
		{
			for (i=0; i > source->maxConnections && source->destAddress[i] != address; ++i);
			if (i == source->maxConnections)
				source->connectionError = ADDRESS_UNKNOWN;
			else
				source->transmissionError[i] = DISTANCE_EXCEEDED;
			return false;
		}
	}
	
	bool BluetoothBase::bbConnectTo(Bluetooth* source,unsigned address)
	{
		Bluetooth* destination = getAddress(address);
		
		if (destination && checkDistance(source,destination))
		{
			
			// Check if both robots have a free connection
			if (source->nbConnections < source->maxConnections && destination->nbConnections < destination->maxConnections)
			{
				unsigned i=0, j=0;
				for (; i<source->maxConnections && source->destAddress[i] < UINT_MAX; ++i);
				for (; j<destination->maxConnections && destination->destAddress[j] < UINT_MAX; ++j);
				assert(i<source->maxConnections);
				assert(j<destination->maxConnections);

				source->destAddress[i] = address;
				destination->destAddress[j] = source->address;
				
				source->nbConnections++;
				destination->nbConnections++;

				source->connectionError = BT_NO_ERROR;
				return true;
			}
			else
			{  // the source or the destination has no slot free
				source->connectionError = TOO_MANY_CONNECTIONS;
				return false;
			}
		}
		else
		{ // Destination doesn't exist
			source->connectionError = ADDRESS_UNKNOWN;
			return false;
		}
	}
	
	bool BluetoothBase::bbCloseConnection(Bluetooth* source,unsigned address)
	{
		Bluetooth* destination = getAddress(address);
		
		if (destination && checkDistance(source,destination))
		{
			unsigned i = 0, j = 0;
			
			for (;i<source->maxConnections && source->destAddress[i] != address;++i);
			for (;j<destination->maxConnections && destination->destAddress[j] != source->address;++j);
			
			if (i==source->maxConnections || j==destination->maxConnections)
			{
				source->disconnectionError = ADDRESS_UNKNOWN;
				return false;
			}
			else
			{
				source->destAddress[i] = UINT_MAX;
				destination->destAddress[j] = UINT_MAX;
				
				source->nbConnections--;
				destination->nbConnections--;
				
				source->disconnectionError = BT_NO_ERROR;
				return true;
			}
		}
		else
		{
			source->disconnectionError = true;
			return false;
		}
	}
	
	bool BluetoothBase::checkDistance(Bluetooth* source, Bluetooth* destination)
	{
		Point a = source->owner->pos;
		Point b = destination->owner->pos;
		
		double dist2 = sqrt(pow(a.x-b.x,2.0) + pow(a.y-b.y,2.0));
		
		if (dist2 > source->range || dist2 > destination->range)
			return false;
		else
			return true;
		
	}

	
    void BluetoothBase::sendDataTo(Bluetooth* source, unsigned address, char* data, unsigned size)
    {
		Transmissions tr;
		tr.source = source;
		tr.address = address;
		tr.data = data;
		tr.size = size;
		
		transmissions.push(tr);
	}
	
    void BluetoothBase::connectTo(Bluetooth* source,unsigned address)
    {
		Connections con;
		
		con.source = source;
		con.destaddress = address;
		
		connectbuffer.push(con);
		
	}
	
    void BluetoothBase::closeConnection(Bluetooth* source,unsigned address)
    {
		Connections con;
		
		con.source = source;
		con.destaddress = address;
		
		disconnectbuffer.push(con);
		
	}
	
	
	void BluetoothBase::step(double dt, World *w)
	{
		// First the disconnections
		Connections con;
		Transmissions tx;
		
		while (!disconnectbuffer.empty())
		{
			con = disconnectbuffer.front();
			bbCloseConnection(con.source, con.destaddress);
			disconnectbuffer.pop();
		}
		
		// Second the connections
		
		while (!connectbuffer.empty())
		{
			con = connectbuffer.front();
			bbConnectTo(con.source, con.destaddress);
			connectbuffer.pop();
		}
		
		// Now we send the data
		
		while (!transmissions.empty())
		{
			tx = transmissions.front();
			bbSendDataTo(tx.source, tx.address, tx.data, tx.size);
			transmissions.pop();
		}
	}

}
