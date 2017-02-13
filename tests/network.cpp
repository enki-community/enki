/*
  Enki - a fast 2D robot simulator
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

#include <enki/Network.h>
#include <unistd.h>
#include <thread>

using namespace Enki;

const std::string SERVER_IP = "127.0.0.1";

SCENARIO ( "A basic connection", "[Enki::Network]" ) {
	GIVEN( "a Server" )	{
		Server s(new World);
		std::thread threadServer([&]() { s.run(); });
		threadServer.join();

		WHEN( "a client connects" ) {
			Client c(SERVER_IP);
			std::thread threadClient([&]() { c.run(); });
			threadClient.join();

			THEN( "the server has a connection" ) {
				REQUIRE( s.getConnectionNumbers() == 1 );
			}
		}
	}
}
