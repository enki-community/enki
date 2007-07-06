/*
    Enki Network - convenient cross-platform packaging for tcp/ip networking
    Adapted from Ishtar Network, same author
    Copyright (C) 2003-2005 Stephane Magnenat <stephane at magnenat dot net>
    (http://stephane.magnenat.net)

    This program is free software; you can redistribute it and/or modify
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

#ifndef __ENKI_NETWORK_H
#define __ENKI_NETWORK_H

#ifndef WIN32
	#include <arpa/inet.h>
	#include <netinet/in.h>
#else
	#ifndef INADDR_ANY
	#define INADDR_ANY 0x00000000
	#endif
#endif
#include <string>
#include <list>

/*!	\file Network.h
	\brief Interface of network classes: clean, endian-safe and easy to use TCP/IP sockets wrapper
*/

namespace Enki
{
	//! A TCP/IP version 4 address
	class TCPIPAddress
	{
	public:
		//! IP Address
		/*! Stored in local byte order.
		*/
		unsigned address;
	
		//! TCP Port
		/*! Stored in local byte order.
		*/
		unsigned short port;
	
	public:
		//! Constructor
		TCPIPAddress(unsigned addr = INADDR_ANY, unsigned short prt = 0)
		{
			address = addr;
			port = prt;
		}
	
		//! Equality operator
		bool operator==(const TCPIPAddress& o) const
		{
			return address==o.address && port==o.port;
		}
		//! Less than operator
		bool operator<(const TCPIPAddress& o) const
		{
			return address<o.address || (address==o.address && port<o.port);
		}
		//! Return string form
		std::string format() const;
		//! Is the address valid?
		bool valid() const { return address != INADDR_ANY && port != 0; }
	};
	
	//! Useful functions for sockets
	class SocketHelper
	{
	private:
	#ifndef WIN32
		//! Handler that catchs SIGPIPE signal
		void (*oldHandler)(int);
	#endif
	
	public:
		//! Constructor, initializes networking subsystem
		SocketHelper();
		//! Destructor, deinitializes networking subsystem
		~SocketHelper();
	
		//! Resolve a name
		static TCPIPAddress resolve(const std::string &name, unsigned short defPort);
	};
	
	//! Instance of socket helper
	extern SocketHelper socketHelper;
	
	//! A socket, a TCP/IP binary stream
	class Socket
	{
	protected:
		friend class NetworkClient;
		friend class NetworkServer;
		//! The destination address and port
		TCPIPAddress remoteAddress;
		//! The TCP/IP stack socket
		int socket;
		//! The sending bufffer. Its size is increased when required. On flush, bufferPos is set to zero and buffersize_t rest unchanged. It is freed on Socket destruction.
		unsigned char *sendBuffer;
		//! The size of sendBuffer
		size_t buffersize_t;
		//! The actual position in sendBuffer
		size_t bufferPos;
		//! True if socket is connected
		bool connected;
		
	protected:
		//! Send the data on the socket, try until size is sent or an error occured
		void send(const void *data, size_t size);
	
	public:
		//! Constructor
		Socket();
		//! Destructor
		virtual ~Socket();
		//! Write size amount of data to the socket, first buffer.
		virtual void write(const void *data, const size_t size);
		//! Write send buffer to the socket
		virtual void flush(void);
		//! Read size amount of data from the socket
		virtual void read(void *data, size_t size);
		//! Return the name of the destination
		std::string getRemoteName(void) { return remoteAddress.format(); }
		//! Return true if socket is connected
		bool isConnected(void) { return connected; }
		//! Disconnect
		void disconnect(void) { connected = false; }
	};
	
	//! A client that connect to a TCP/IP host. User code subclass should inherits from NetworkClient and implement incomingData and ocnnectionClosed
	class NetworkClient
	{
	protected:
		//! The socket that connect to the server
		Socket *socket;
	
	public:
		//! Constructor. Create socket and connects to host on port
		NetworkClient(const std::string &host, unsigned short port);
		//! Destructor, close connection and destroy socket
		virtual ~NetworkClient();
	
		//! Run and return only when connection is closed
		void run(void);
		//! Read data from socket and call incomingData. If block is true, wait until any data arrives.  If block is false, wait timeout us at maximum
		bool step(bool block = false, long timeout = 0);

	protected:
		//! Called when datas are available on socket. If socket is closed in this method, connectionClosed is called. Must be implemented by subclass
		virtual void incomingData(Socket *socket) = 0;
		//! Called when connection is closed on socket. Must be implemented by subclass
		virtual void connectionClosed(Socket *socket) = 0;
	};
	
	//! A server that listen on a TCP/IP host. User code subclass should inherits from NetworkServer and implement incomingData, incomingConnection and ocnnectionClosed
	class NetworkServer
	{
	private:
		//! The server socket on which we listen
		Socket *serverSocket;
		//! The list of connected clients
		std::list<Socket *> clients;
		
	private:
		//! Close client socket s and remove client from list
		void closeSocket(Socket *s);
		
	public:
		//! Constructor. The socket is not yet created
		NetworkServer();
		//! Destructor, close connections and destroy server socket
		virtual ~NetworkServer() { }
		//! Create server socket that listen on port
		bool listen(unsigned short port);
		//! Stop listening and close server socket
		void shutdown();
		
		//! Run and return only on SIGTERM
		void run(void);
		//! Read server and client sockets from incoming connection or datas and call incomingConnection or incomingData. If block is true, wait until any data arrives. If block is false, wait timeout us at maximum
		void step(bool block = false, long timeout = 0);

	protected:
		//! Called when datas from client are available on socket. If socket is closed in this method, connectionClosed is called. Must be implemented by subclass
		virtual void incomingData(Socket *socket) = 0;
		//! Called when a new connection has been established on socket. If socket is closed in this method, connectionClosed is not called. Must be implemented by subclass
		virtual void incomingConnection(Socket *socket) = 0;
		//! Called when connection to client is closed on socket. Must be implemented by subclass
		virtual void connectionClosed(Socket *socket) = 0;
	};
}

#endif
