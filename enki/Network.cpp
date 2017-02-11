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

#include "Network.h"

/*!	\file Network.cpp
	\brief Module 'Network' which can be used for sharing world instances.
*/

namespace Enki
{
	using namespace std;
	using namespace Dashel;

	const unsigned short int PORT = 8765;

	Server::Server(World* w)
	{
		connect("tcpin:port=" + to_string(PORT));
		m_world = w;
	}

	void Server::sendAll()
	{
		string init = serialize(m_world);
		init += "\n";

		for (StreamsSet::iterator it = dataStreams.begin(); it != dataStreams.end(); ++it)
		{
			sendString((*it), init);
		}
	}

	void Server::initWorld(Stream* stream)
	{
		string init = serialize(m_world);
		init += "\n";
		sendString(stream, init);
	}

	void Server::connectionCreated(Stream *stream)
	{
		cerr << "+ Incoming connection from " << stream->getTargetName() << endl;

		string identification = readLine(stream);
		initWorld(stream);

		cerr << "+ User is connected." << endl;
	}

	void Server::connectionClosed(Stream *stream, bool abnormal)
	{
		cerr << "- Connection closed to " << stream->getTargetName() << endl;
		if (abnormal)
			cerr << " : " << stream->getFailReason();

		cerr << "- User is disconnected." << endl;
		dataStreams.erase(stream);
	}

	Client::Client(string ip) :
		init(true),
		inputStream(0)
	{
		string remoteTarget = "tcp:" + ip + ";port=" + to_string(PORT);
		inputStream = connect("stdin:");
		remoteStream = connect(remoteTarget);
		sendString(remoteStream, "I want to connect!\n");
	}

	void Client::connectionCreated(Stream *stream)
	{
		cerr << "Incoming connection " << stream->getTargetName() << endl;
	}

	void Client::incomingData(Stream *stream)
	{
		assert(inputStream);
		assert(remoteStream);

		if (stream == inputStream)
		{
			string line = readLine(inputStream);
			sendString(remoteStream, line);
		}
		else
		{
			string line = readLine(remoteStream);

			// Init World
			if (init)
			{
				m_world = deserialize(line);
				init = false;
			}
			else
			{
				deserializeUdpate(m_world, line);
			}
		}
	}

	void Client::connectionClosed(Stream *stream, bool abnormal)
	{
		cerr << "Connection closed to " << stream->getTargetName();
		if (abnormal)
			cerr << " : " << stream->getFailReason();
		cerr << endl;
		stop();
	}

	World* Client::getWorld()
	{
		return m_world;
	}
}
