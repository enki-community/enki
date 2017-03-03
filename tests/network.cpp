/*
 Enki - a fast 2D robot simulator
 Copyright © 2017 Sebastien Pouteau <sebastien.pouteau@etu.u-bordeaux.fr>
 Copyright © 2017 Mathieu Lirzin <mathieu.lirzin@etu.u-bordeaux.fr>

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

#include "catch.hpp"

#include <enki/PhysicalEngine.h>
#include <enki/Network.h>
#include <thread>
#include <mutex>
#include <unistd.h>

using namespace Enki;
using namespace std;

const int TIME_OUT = 10000;

bool check(Server * s, int connectionNumber)
{
	sleep(1);
	clock_t t = clock();
	clock_t tmp;
	while((tmp = ( clock() - t)) < TIME_OUT && s->getConnectionNumbers() != connectionNumber )
	{
		usleep(1000);
	}
	sleep(1);
	return tmp < TIME_OUT ;
}

SCENARIO( "Connection Deconnection", "[Enki::Network]" )
{
	GIVEN( "Init server" )
	{
		World world;
		Server* server;

		REQUIRE_NOTHROW( server = new Server(&world) );

		thread threadServer ([=]() { server->run(); });
		sleep(1);

		WHEN( "1 client" )
		{
			Client* client = NULL;
			REQUIRE_NOTHROW( client = new Client("127.0.0.1") );
			REQUIRE( check(server, 1) );

			delete client;

			server->stop();
			threadServer.join();
			delete server;

		}
		WHEN( "2 clients" )
		{
			int nb = 0;
			Client* client;

			REQUIRE_NOTHROW( client = new Client("127.0.0.1") );
			nb++;

			REQUIRE( check(server, nb) );
			Client* client1;
			REQUIRE_NOTHROW( client1 = new Client("127.0.0.1") );
			nb++;
			REQUIRE( check(server, nb) );

			delete client;
			nb--;

			REQUIRE( check(server, nb) );

			REQUIRE_NOTHROW( client = new Client("127.0.0.1") );
			nb++;

			REQUIRE( check(server, nb) );

			delete client;
			nb--;

			REQUIRE( check(server, nb) );

			delete client1;
			nb--;

			REQUIRE( check(server, nb) );

			server->stop();
			threadServer.join();
			delete server;
		}
		WHEN( "10 clients" )
		{
			vector<Client*> tabClient;
			int nbClient = 0;

			for (int i = 0; i < 10; i++)
			{
				Client* cl;
				REQUIRE_NOTHROW( cl = new Client("127.0.0.1") );
				nbClient++;
				REQUIRE( check(server, nbClient) );
				tabClient.push_back(cl);
			}

			for (int i = 0; i < tabClient.size(); i++)
			{
				delete tabClient.at(i);
			}

			tabClient.clear();

			server->stop();
			threadServer.join();
			delete server;
		}
	}
}

SCENARIO( "Receive Data", "[Enki::Network]" )
{
	GIVEN( "Init server" )
	{
		sleep(1);
		World world(12, 12);
		Server* server;
		REQUIRE_NOTHROW( server = new Server(&world) );

		thread threadServer ([=]() { server->run(); });
		sleep(1);

		WHEN( "1 client" )
		{
			Client* client;
			REQUIRE_NOTHROW( client = new Client("127.0.0.1") );

			thread threadClient ([=]() { client->run(); });

			sleep(1);
			World* worldClient = client->getWorld();

			REQUIRE( worldClient->w == world.w );
			REQUIRE( worldClient->h == world.h );
			REQUIRE( worldClient->r == world.r );
			REQUIRE( worldClient->objects.size() == world.objects.size() );
			client->stop();
			server->stop();
			threadClient.join();
			delete client;

			threadServer.join();
			delete server;

		}
	}

}
