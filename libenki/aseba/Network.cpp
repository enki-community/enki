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

/*!	\file Network.cpp
	\brief Implementation of network classes: clean, endian-safe and easy to use TCP/IP sockets wrapper
*/

#include "Network.h"
#include <sstream>
#include <iostream>
#include <assert.h>
#include <string.h>

#ifndef WIN32
	#include <netdb.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <signal.h>
	#include <sys/select.h>
	#include <sys/time.h>
	#include <sys/types.h>
	#include <unistd.h>
#else
	#include <winsock2.h>
#endif

namespace Enki
{
	std::string TCPIPAddress::format() const
	{
		std::stringstream buf;
	
		struct hostent *he;
		unsigned a2 = htonl(address);
		he = gethostbyaddr((const char *)&a2, 4, AF_INET);
	
		if (he == NULL)
		{
			struct in_addr addr;
			addr.s_addr = a2;
			buf << inet_ntoa(addr) << ":" << port;
		}
		else
		{
			buf << he->h_name << ":" << port;
		}
	
		return buf.str();
	}

	SocketHelper socketHelper;

	SocketHelper::SocketHelper()
	{
		#ifdef WIN32
		WORD wVersionRequested;
		WSADATA wsaData;

		wVersionRequested = MAKEWORD( 2,0 );

		WSAStartup( wVersionRequested, &wsaData );
		#else
		oldHandler = signal(SIGPIPE,SIG_IGN);
		#endif
	}

	SocketHelper::~SocketHelper()
	{
		#ifdef WIN32
		WSACleanup();
		#else
		signal(SIGPIPE,oldHandler);
		#endif
	}

	TCPIPAddress SocketHelper::resolve(const std::string &name, unsigned short defPort)
	{
		TCPIPAddress address;
		hostent *he;

		std::string::size_type pos = name.find_first_of(':');
		if (pos != std::string::npos)
		{
			address.port = atoi(name.substr(pos+1, std::string::npos).c_str());
			he = gethostbyname(name.substr(0, pos).c_str());
		}
		else
		{
			address.port = defPort;
			he = gethostbyname(name.c_str());
		}

		if(he == NULL)
		{
			#ifndef WIN32
			struct in_addr addr;
			if (inet_aton(name.c_str(), &addr))
			{
				address.address = ntohl(addr.s_addr);
			}
			else
			{
				address.address = INADDR_ANY;
			}
			#else
			address.address = INADDR_ANY;
			#endif
		}
		else
		{
			address.address=ntohl(*((unsigned *)he->h_addr));
		}

		return address;
	}

	//! size_t of a newly allocated send buffer
	const size_t startSendBuffersize_t = 256;
	//! When size of send buffer is bigSendsize_t, send it, even if flush hasn't been called
	const size_t bigSendsize_t = 65536;
	//! Network server is running, set to 0 by SIGTERM
	int netRun = 1;
	
	Socket::Socket()
	{
		socket = -1;
		sendBuffer = (unsigned char*)malloc(startSendBuffersize_t);
		buffersize_t = startSendBuffersize_t;
		bufferPos = 0;
		connected = false;
	}
	
	Socket::~Socket()
	{
		if (connected)
		{
			#ifndef WIN32
			shutdown(socket, SHUT_RDWR);
			close(socket);
			#else
			shutdown(socket, SD_BOTH);
			closesocket(socket);
			#endif
		}
		free(sendBuffer);
	}
	
	void Socket::write(const void *data, const size_t size)
	{
		assert(socket>=0);
		
		if (size >= bigSendsize_t)
		{
			flush();
			send(data, size);
		}
		else
		{
			if (bufferPos + size > buffersize_t)
			{
				buffersize_t = std::max(buffersize_t * 2, bufferPos + size);
				sendBuffer = (unsigned char*)realloc(sendBuffer, buffersize_t);
			}
			memcpy(sendBuffer+bufferPos, (unsigned char *)data, size);
			bufferPos += size;
	
			if (bufferPos >= bigSendsize_t)
				flush();
		}
	}
	
	void Socket::flush(void)
	{
		assert(socket>=0);
		assert(sendBuffer);
		
		if (!connected)
			return;
	
		send(sendBuffer, bufferPos);
		bufferPos = 0;
	}
	
	void Socket::send(const void *data, size_t size)
	{
		if (!connected)
			return;
			
		unsigned char *ptr = (unsigned char *)data;
		size_t left = size;
		
		while (left)
		{
			#ifndef WIN32
			int len = ::send(socket, ptr, left, 0);
			#else
			int len = ::send(socket, (const char *)ptr, left, 0);
			#endif
			if (len < 1)
			{
				connected = false;
				break;
			}
			left -= len;
			ptr += len;
		}
	}
	
	void Socket::read(void *data, size_t size)
	{
		assert(socket>=0);
		
		if (!connected)
			return;
			
		unsigned char *ptr = (unsigned char *)data;
		size_t left = size;
	
		while (left)
		{
			#ifndef WIN32
			int len = recv(socket, ptr, left, 0);
			#else
			Int32 len = recv(socket, (char *)ptr, left, 0);
			#endif
			if (len < 1)
			{
				connected = false;
				memset(data, 0, size);
				break;
			}
			left -= len;
			ptr += len;
		}
	}
	
	NetworkClient::NetworkClient(const std::string &host, unsigned short port)
	{
		socket = new Socket();
		assert(socket);
	
		socket->remoteAddress = socketHelper.resolve(host, port);
		socket->socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (socket->socket == -1)
		{
			std::cerr << "NetworkClient::NetworkClient(" << host << ", " << port << ") : can't create socket" << std::endl;
			assert(false);
		}
	
		sockaddr_in addr;
		addr.sin_family = AF_INET;
		addr.sin_port = htons(socket->remoteAddress.port);
		addr.sin_addr.s_addr = htonl(socket->remoteAddress.address);
		if (::connect(socket->socket, (struct sockaddr *)&addr, sizeof(addr)) == -1)
		{
			std::cerr << "NetworkClient::NetworkClient(" << host << ", " << port << ") : can't connect to host" << std::endl;
		}
		else
		{
			socket->connected = true;
		}
	}
	
	NetworkClient::~NetworkClient()
	{
		delete socket;
	}
	
	#ifndef WIN32
	//! Called when SIGTERM arrives, halts NetworkServer run
	void termHandler(int t)
	{
		netRun = 0;
	}
	#endif
	
	void NetworkClient::run(void)
	{
		#ifndef WIN32
		signal(SIGTERM, termHandler);
		#endif
		
		while (netRun)
		{
			step(true);
		}
		
		#ifndef WIN32
		signal(SIGTERM, SIG_DFL);
		#endif
	}
	
	bool NetworkClient::step(bool block, long timeout)
	{
		fd_set rfds;
		int nfds;
		FD_ZERO(&rfds);
		FD_SET(socket->socket, &rfds);
		nfds = socket->socket;
		
		int ret;
		if (block)
		{
			ret = select(nfds+1, &rfds, NULL, NULL, NULL);
		}
		else
		{
			struct timeval t;
			t.tv_sec = 0;
			t.tv_usec = timeout;
			ret = select(nfds+1, &rfds, NULL, NULL, &t);
		}
		
		if (ret<0)
		{
			std::cerr << "NetworkClient::step(" << block << ") : select error" << std::endl;
			assert(false);
		}
	
		if (FD_ISSET(socket->socket, &rfds))
		{
			char c;
			if (recv(socket->socket, &c, 1, MSG_PEEK) != 1)
			{
				socket->connected = false;
				connectionClosed(socket);
				netRun = false;
			}
			else
			{
				incomingData(socket);
				if (!socket->connected)
				{
					connectionClosed(socket);
					netRun = false;
				}
			}
			return true;
		}
		else
		{
			return false;
		}
	}

	NetworkServer::NetworkServer()
	{
		serverSocket = 0;
	}
	
	bool NetworkServer::listen(unsigned short port)
	{
		shutdown();
		
		serverSocket = new Socket();
		assert(serverSocket);
	
		char localName[128];
		gethostname(localName,128);
		serverSocket->remoteAddress = socketHelper.resolve(localName, port);
	
		serverSocket->socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (serverSocket->socket == -1)
		{
			std::cerr << "NetworkServer::NetworkServer(" << port << ") : can't create socket" << std::endl;
			assert(false);
		}
	
		int yes = 1;
		if (setsockopt(serverSocket->socket, SOL_SOCKET, SO_REUSEADDR, (char *) &yes, sizeof (yes)) < 0)
		{
			std::cerr << "NetworkServer::NetworkServer(" << port << ") : can't enable SO_REUSEADDR" << std::endl;
			assert(false);
		}
	
		sockaddr_in addr;
		memset (&addr, 0, sizeof (addr));
		addr.sin_family = AF_INET;
		addr.sin_port = htons(serverSocket->remoteAddress.port);
		if (bind(serverSocket->socket, (struct sockaddr *) &addr, sizeof (addr)) < 0)
		{
			std::cerr << "NetworkServer::NetworkServer(" << port << ") : can't bind server socket" << std::endl;
			delete serverSocket;
			serverSocket = 0;
			return false;
		}
	
		serverSocket->connected = true;
		::listen(serverSocket->socket, 10);
		return true;
	}
	
	//! Stop listening and close server socket
	void NetworkServer::shutdown()
	{
		for (std::list<Socket *>::iterator it = clients.begin(); it != clients.end(); ++it)
			delete (*it);
		if (serverSocket)
			delete serverSocket;
	}
	
	void NetworkServer::step(bool block, long timeout)
	{
		fd_set rfds;
		int nfds = 0;
		FD_ZERO(&rfds);
	
		// add client sockets
		for (std::list<Socket *>::iterator it = clients.begin(); it != clients.end(); ++it)
		{
			int fd = (*it)->socket;
			FD_SET(fd, &rfds);
			nfds = std::max(fd, nfds);
		}
	
		// add server socket
		FD_SET(serverSocket->socket, &rfds);
		nfds = std::max(serverSocket->socket, nfds);
	
		// select !!
		int ret;
		if (block)
		{
			ret = select(nfds+1, &rfds, NULL, NULL, NULL);
		}
		else
		{
			struct timeval t;
			t.tv_sec = 0;
			t.tv_usec = timeout;
			ret = select(nfds+1, &rfds, NULL, NULL, &t);
		}
		
		if (ret<0)
		{
			std::cerr << "NetworkServer::step(" << block << ") : select error" << std::endl;
			assert(false);
		}
	
		// read client sockets
		for (std::list<Socket *>::iterator it = clients.begin(); it != clients.end();)
		{
			int fd = (*it)->socket;
			if (FD_ISSET(fd, &rfds))
			{
				char c;
				if (recv((*it)->socket, &c, 1, MSG_PEEK) != 1)
				{
					std::list<Socket *>::iterator ci = it;
					it++;
					(*ci)->connected = false;
					connectionClosed(*ci);
					closeSocket(*ci);
					continue;
				}
				else
				{
					incomingData(*it);
					if (!(*it)->connected)
					{
						std::list<Socket *>::iterator ci = it;
						it++;
						connectionClosed(*ci);
						closeSocket(*ci);
						continue;
					}
				}
			}
			++it;
		}
	
		// read server socket
		if (FD_ISSET(serverSocket->socket, &rfds))
		{
			Socket *socket = new Socket();
			assert(socket);
	
			struct sockaddr_in clientAddr;
			socklen_t l = sizeof (clientAddr);
			memset  (&clientAddr, 0, l);
			int newSocket = accept (serverSocket->socket, (struct sockaddr *)&clientAddr, &l);
			if (newSocket < 0)
			{
				std::cerr << "NetworkServer::step(" << block << ") : can't accept new connection" << std::endl;
				assert(false);
			}
			else
			{
				socket->socket = newSocket;
				socket->connected = true;
				socket->remoteAddress.port = ntohs(clientAddr.sin_port);
				socket->remoteAddress.address = ntohl(clientAddr.sin_addr.s_addr);
			}
	
			clients.push_back(socket);
	
			incomingConnection(socket);
			if (!socket->connected)
				closeSocket(socket);
		}
	}
	
	void NetworkServer::run(void)
	{
		#ifndef WIN32
		signal(SIGTERM, termHandler);
		#endif
		
		while (netRun)
		{
			step(true);
		}
		
		#ifndef WIN32
		signal(SIGTERM, SIG_DFL);
		#endif
	}
	
	void NetworkServer::closeSocket(Socket *s)
	{
		clients.remove(s);
		delete s;
	}
}
