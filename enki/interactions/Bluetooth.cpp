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

#include "Bluetooth.h"
#include "../BluetoothBase.h"

#include <limits.h>
#include <assert.h>

/*!	\file Bluetooth.cpp
	\brief Implementation of the bluetooth module
*/

namespace Enki
{
	Bluetooth::Bluetooth(Robot* owner,double range, unsigned maxConnections, unsigned rxbuffersize, unsigned txbuffersize, unsigned address)
	{
		this->owner=owner;
		this->range=range;
		this->maxConnections=maxConnections;
		this->address=address;
		this->updateAddress=true;
		this->randomAddress=true;
		this->connectionError=BT_NO_ERROR;
		this->disconnectionError=BT_NO_ERROR;
		this->rxBufferSize=rxbuffersize;
		this->txBufferSize=txbuffersize;
		
		initAllData();
		
		nbConnections=0;
		
	}
	
	Bluetooth::~Bluetooth()
	{
		cancelAllData();
	}
	
	void Bluetooth::cancelRxBuffer()
	{
		for (unsigned i=0;i<maxConnections;++i)
			delete[] rxBuffer[i];
		delete[] rxBuffer;
	}
	
	void Bluetooth::cancelTxBuffer()
	{
		for (unsigned i=0;i<maxConnections;++i)
			delete[] txBuffer[i];
		delete[] txBuffer;
	}
	
	void Bluetooth::cancelAllData()
	{
		delete[] receptionFlags;
		delete[] destAddress;
		delete[] sizeToSend;
		delete[] sizeReceived;
		delete[] transmissionError;
		
		for (unsigned i=0;i<maxConnections;++i)
		{
			delete[] rxBuffer[i];
			delete[] txBuffer[i];
		}
		
		delete[] rxBuffer;
		delete[] txBuffer;
	}
	
	void Bluetooth::initAllData()
	{
		sizeReceived=new unsigned[maxConnections];
		transmissionError=new unsigned[maxConnections];
		rxBuffer=new char*[maxConnections];
		txBuffer=new char*[maxConnections];
		receptionFlags=new bool[maxConnections];
		destAddress=new unsigned[maxConnections];
		sizeToSend=new unsigned[maxConnections];
		for (unsigned i=0;i<maxConnections;++i)
		{
			rxBuffer[i]=new char[rxBufferSize];
			txBuffer[i]=new char[txBufferSize];
			receptionFlags[i]=false;
			destAddress[i]=UINT_MAX;
			sizeToSend[i]=0;
			sizeReceived[i]=0;
			transmissionError[i]=BT_NO_ERROR;
		}
	}
	
	void Bluetooth::setAddress(unsigned address)
	{
		this->address = address;
		updateAddress=true;
		randomAddress=false;
	}
	
	unsigned Bluetooth::getMaxConnections()
	{
		return maxConnections;
	}
	
	unsigned Bluetooth::getAddress()
	{
		return this->address;
	}
	
	unsigned Bluetooth::getNbConnections()
	{
		return nbConnections;
	}
	
	unsigned* Bluetooth::getConnectedAddresses()
	{
		return destAddress;
	}
	
	bool Bluetooth::didIReceive()
	{
		bool reception=false;
		for (unsigned i=0;i<maxConnections;++i)
			reception=reception || receptionFlags[i];
		return reception;
	}
	
	bool Bluetooth::didIReceive(unsigned source)
	{
		if (source==UINT_MAX)
			return false;
			
		unsigned index=0;
		while (index<maxConnections && destAddress[index]!=source)
			index++;

		if (index<maxConnections)
			return receptionFlags[index];
		else
			return false;
	}
	
	bool* Bluetooth::getReceptionFlags()
	{
		return receptionFlags;
	}
	
	const char* Bluetooth::getRxBuffer(unsigned source)
	{
		if (source==UINT_MAX)
			return NULL;
			
		unsigned index=0;
		while (index<maxConnections && destAddress[index]!=source)
			index++;
		
		if (index<maxConnections)
		{
			receptionFlags[index]=false;
			return rxBuffer[index];
		}
		else
			return NULL;
	}
	
	unsigned Bluetooth::getSizeReceived(unsigned source)
	{
		if (source==UINT_MAX)
			return 0;
			
		unsigned index=0;
		while (index<maxConnections && destAddress[index]!=source)
			index++;
		
		if (index<maxConnections)
		{
			return sizeReceived[index];
		}
		else
			return 0;
	}

	
	void Bluetooth::connectTo(unsigned address)
	{
		connectToRobot.push(address);
		// destAddress is updated by the base as it is the only one knowing if the connection succeeded
	}
	
	bool Bluetooth::closeConnection(unsigned dest)
	{
		// First check if the connection exists
		if (dest==UINT_MAX)
			return false;
			
		unsigned index=0;
		while (index<maxConnections && destAddress[index]!=dest)
			index++;
		
		if (index==maxConnections)
			return false;
		else
		{
			closeConnectionToRobot.push(dest);
			return true;
		}
	}
	
	bool Bluetooth::sendDataTo(unsigned dest,char* data,unsigned size)
	{
		// First, check if we are connected.
		if (dest==UINT_MAX)
			return false;
			
		unsigned index=0;
		while (index<maxConnections && destAddress[index]!=dest)
			index++;
		
		if (index==maxConnections)
			return false;
		else
		{
			unsigned i;
			for (i=0;i<size && i<txBufferSize;++i)
				txBuffer[index][i]=data[i];
			sizeToSend[index]=i;
			return true;
		}
	}
	
	unsigned* Bluetooth::getTransmissionError()
	{
		return transmissionError;
	}
	
	bool Bluetooth::isThereTxError()
	{
		bool error=false;
		for (unsigned i=0;i<maxConnections;++i)
			error=error || transmissionError[i];
		return error;
	}
	


	void Bluetooth::changeMaxConnections(unsigned size)
	{
		cancelAllData();
		maxConnections=size;
		initAllData();
		
	}

	unsigned Bluetooth::getRxBufferSize()
	{
		return rxBufferSize;
	}

	void Bluetooth::changeRxBufferSize(unsigned size)
	{
		cancelRxBuffer();
		rxBufferSize=size;
		rxBuffer=new char*[maxConnections];
		for (unsigned i=0;i<maxConnections;++i)
			rxBuffer[i]=new char[rxBufferSize];
	}

	unsigned Bluetooth::getTxBufferSize()
	{
		return txBufferSize;
	}

	void Bluetooth::changeTxBufferSize(unsigned size)
	{
		cancelTxBuffer();
		txBufferSize=size;
		txBuffer=new char*[maxConnections];
		for (unsigned i=0;i<maxConnections;++i)
			txBuffer[i]=new char[txBufferSize];
		
	}


	void Bluetooth::step(double dt, World *w)
	{
	
		BluetoothBase* bb=w->getBluetoothBase();
		if (updateAddress)
		{
			if (randomAddress)
				while (bb->registerClient(this,address) == false)
					address=random.get()%UINT_MAX;
			else
				assert(bb->registerClient(this,address));
			updateAddress=false;
		}
		
		// Connection to another robot
		connectionError=BT_NO_ERROR;
		while (!connectToRobot.empty())
		{
			bb->connectTo(this,connectToRobot.front());
			connectToRobot.pop();
		}
		
		// Disconnection from another robot
		disconnectionError=BT_NO_ERROR;
		while (!closeConnectionToRobot.empty())
		{
			bb->closeConnection(this,closeConnectionToRobot.front());
			closeConnectionToRobot.pop();
		}
		
		// Check if we have something to send
		for (unsigned i=0;i<maxConnections;++i)
		{
			if (destAddress[i]<UINT_MAX && sizeToSend[i]>0)
			{
				// We are connected and we want to send something
				transmissionError[i]=BT_NO_ERROR;
				bb->sendDataTo(this,destAddress[i],txBuffer[i],sizeToSend[i]);
			}
		}
	}
	
	unsigned Bluetooth::getConnectionError()
	{
		return (unsigned)connectionError;
	}

	unsigned Bluetooth::getDisconnectionError()
	{
		return (unsigned)disconnectionError;
	}

}
