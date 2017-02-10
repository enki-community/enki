/*
  Enki - a fast 2D robot simulator
  Copyright © 2017 Sébastien Pouteau <sebastien.pouteau@etu.u-bordeaux.fr>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef NETWORK_H
#define NETWORK_H

#include <iostream>
#include <cassert>
#include <dashel/dashel.h>
#include <enki/PhysicalEngine.h>
#include <enki/Serialize.h>

/*!	\file Network.h
	\brief Module 'Network' which can be used for sharing world instances.
*/

namespace Enki
{
	/*!
		\brief Convert stream to a string.
		\param stream this is where we read the data
		\return string message that is received
	*/
	static inline std::string readLine(Dashel::Stream* stream)
	{
		char c;
		std::string line;
		do
		{
			stream->read(&c, 1);
			line += c;
		}
		while (c != '\n' && c != '\r');
		return line;
	}

	/*!
		\brief Write line into stream.
		\param stream this is where we write the data
		\param string message that is sent
	*/
	static inline void sendString(Dashel::Stream* stream, const std::string& line)
	{
		stream->write(line.c_str(), line.length());
		stream->flush();
	}

	class Server: public Dashel::Hub
	{
	private:
		Enki::World* m_world;

	public:
		//! Constructor
		Server(World* w);

		//! brief Send information about the world to all users connected.
		void sendAll();

		/*!
			\brief Return the number of connections.
			\return number of connections
		*/
		inline int getConnectionNumbers()
		{
			return dataStreams.size();
		}

	protected:
		/*!
			\brief Send all the information about the world m_world to last connected client.
			\param stream this is where we write the data
		*/
		void initWorld(Dashel::Stream* stream);

		// Override Dashel::Hub
		void connectionCreated(Dashel::Stream *stream);
		void connectionClosed(Dashel::Stream *stream, bool abnormal);
	};

	class Client: public Dashel::Hub
	{
	private:
		bool init;
		World* m_world;

	public:
		//! Constructor
		Client(std::string ip);
		World* getWorld();

	protected:
		Dashel::Stream* inputStream;
		Dashel::Stream* remoteStream;

		// Override Dashel::Hub
		void connectionCreated(Dashel::Stream *stream);
		void incomingData(Dashel::Stream *stream);
		void connectionClosed(Dashel::Stream *stream, bool abnormal);
	};
}

#endif // NETWORK_H
