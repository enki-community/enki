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
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "catch.hpp"

#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <enki/robots/s-bot/Sbot.h>
#include <enki/robots/khepera/Khepera.h>
#include <enki/robots/marxbot/Marxbot.h>

// XXX: Use BDD style.
TEST_CASE( "An empty Enki world", "[Enki::World]" ) {
	SECTION( "the default constructor" ) {
		Enki::World world;

		REQUIRE( world.w == 0 );
		REQUIRE( world.h == 0 );
		REQUIRE( world.r == 0 );
		REQUIRE( world.color == Enki::Color::gray );
		REQUIRE( world.groundTexture.width == 0 );
		REQUIRE( world.groundTexture.height == 0 );
		REQUIRE( world.bluetoothBase == NULL );
		REQUIRE( world.wallsType == Enki::World::WALLS_NONE );
		REQUIRE( world.objects.size() == 0 );
	}

	SECTION( "width and height" ) {
		Enki::World world(200, 200);

		REQUIRE( world.w == 200 );
		REQUIRE( world.h == 200 );
		REQUIRE( world.r == 0 );
		REQUIRE( world.color == Enki::Color::gray );
		REQUIRE( world.groundTexture.width == 0 );
		REQUIRE( world.groundTexture.height == 0 );
		REQUIRE( world.bluetoothBase == NULL );
		REQUIRE( world.wallsType == Enki::World::WALLS_SQUARE );
		REQUIRE( world.objects.size() == 0 );
	}

	SECTION( "width, height and color" ) {
		Enki::World world(200, 200, Enki::Color::red);

		REQUIRE( world.w == 200 );
		REQUIRE( world.h == 200 );
		REQUIRE( world.r == 0 );
		REQUIRE( world.color == Enki::Color::red );
		REQUIRE( world.groundTexture.width == 0 );
		REQUIRE( world.groundTexture.height == 0 );
		REQUIRE( world.bluetoothBase == NULL );
		REQUIRE( world.wallsType == Enki::World::WALLS_SQUARE );
		REQUIRE( world.objects.size() == 0 );
	}

	SECTION( "radius" )	{
		Enki::World world(200);

		REQUIRE( world.w == 0 );
		REQUIRE( world.h == 0 );
		REQUIRE( world.r == 200 );
		REQUIRE( world.color == Enki::Color::gray );
		REQUIRE( world.groundTexture.width == 0 );
		REQUIRE( world.groundTexture.height == 0 );
		REQUIRE( world.bluetoothBase == NULL );
		REQUIRE( world.wallsType == Enki::World::WALLS_CIRCULAR );
		REQUIRE( world.objects.size() == 0 );
	}

	SECTION( "radius and color" ) {
		Enki::World world(200, Enki::Color::red);

		REQUIRE( world.w == 0 );
		REQUIRE( world.h == 0 );
		REQUIRE( world.r == 200 );
		REQUIRE( world.color == Enki::Color::red );
		REQUIRE( world.groundTexture.width == 0 );
		REQUIRE( world.groundTexture.height == 0 );
		REQUIRE( world.bluetoothBase == NULL );
		REQUIRE( world.wallsType == Enki::World::WALLS_CIRCULAR );
		REQUIRE( world.objects.size() == 0 );
	}
}

using namespace Enki;

TEST_CASE( "Thymio2", "[Enki::Thymio2]" ) {
	SECTION( "The default constructor" ) {
		Thymio2 thymio;

		// Test color of robot
		for (unsigned int i=0; i< Thymio2::LED_COUNT; i++)
		{
			switch(i)
			{
			case Thymio2::TOP:
			case Thymio2::BOTTOM_LEFT:
			case Thymio2::BOTTOM_RIGHT:
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex)i) == Color(0.0,0.0,0.0,0.0) );
				break;

			case Thymio2::BUTTON_UP:
			case Thymio2::BUTTON_DOWN:
			case Thymio2::BUTTON_LEFT:
			case Thymio2::BUTTON_RIGHT:
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex)i)== Color(1.0,0.0,0.0,0.0) );
				break;

			case Thymio2::RING_0: case Thymio2::RING_1: case Thymio2::RING_2:
			case Thymio2::RING_3: case Thymio2::RING_4: case Thymio2::RING_5:
			case Thymio2::RING_6: case Thymio2::RING_7:
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex)i)== Color(1.0,0.5,0.0,0.0) );
				break;

			case Thymio2::IR_FRONT_0: case Thymio2::IR_FRONT_1: case Thymio2::IR_FRONT_2:
			case Thymio2::IR_FRONT_3: case Thymio2::IR_FRONT_4: case Thymio2::IR_FRONT_5:
			case Thymio2::IR_BACK_0:  case Thymio2::IR_BACK_1:
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex)i)== Color(1.0,0.0,0.0,0.0) );
				break;

			case Thymio2::LEFT_BLUE: case Thymio2::RIGHT_BLUE:
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex)i)== Color(0.0,1.0,1.0,0.0) );
				break;

			case Thymio2::LEFT_RED:	 case Thymio2::RIGHT_RED:
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex)i)== Color(1.0,0.0,0.0,0.0) );
				break;

			default: break;
			}
		}
	}

	SECTION( "position and color" ) {
		Thymio2 thymio;
		thymio.pos = Point(10, 10);

		REQUIRE( thymio.pos.x == 10 );
		REQUIRE( thymio.pos.y == 10 );

		for (unsigned int i = 0; i < Thymio2::LED_COUNT; i++)
		{
			thymio.setLedColor((Thymio2::LedIndex) i, Color(10.0, 10.0, 10.0, 1.0));
			if (i == Thymio2::TOP || i == Thymio2::BOTTOM_LEFT || i == Thymio2::BOTTOM_RIGHT)
			{
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex) i) == Color(10.0, 10.0, 10.0, 1.0) );
			}
			else
			{
				Color c = Color(thymio.getColorLed((Thymio2::LedIndex) i).r(),thymio.getColorLed((Thymio2::LedIndex) i).g(),thymio.getColorLed((Thymio2::LedIndex)i).b(),1.0);
				REQUIRE( thymio.getColorLed((Thymio2::LedIndex) i) ==  c );
			}
		}
	}
}

TEST_CASE( "EPuck", "[Enki::EPuck]" ) {
	SECTION( "The default constructor" ) {
		EPuck ePuck;

		REQUIRE( ePuck.pos.x == 0 );
		REQUIRE( ePuck.pos.y == 0 );
		REQUIRE( ePuck.getColor() == Color(0, 0.7, 0) );
		REQUIRE( ePuck.isCylindric() );
		REQUIRE( ePuck.getHeight() == 4.7 );
		REQUIRE( ePuck.getRadius() == 3.7 );
	}

	SECTION( "Position and color" ) {
		EPuck ePuck;

		ePuck.pos = Point(10,10);
		ePuck.setLedRing(true);

		REQUIRE( ePuck.pos.x == 10 );
		REQUIRE( ePuck.pos.y == 10 );
		REQUIRE( ePuck.getColor() == Color::red);
	}
}

TEST_CASE( "Marxbox", "[Enki::Marxbox]" ) {
	SECTION( "The default constructor" ) {
		Marxbot marxbot;

		REQUIRE( marxbot.pos.x == 0 );
		REQUIRE( marxbot.pos.y == 0 );
		REQUIRE( marxbot.isCylindric() );
		REQUIRE( marxbot.getHeight() == 12 );
		REQUIRE( marxbot.getRadius() == 8.5 );
	}

	SECTION( "Position" ) {
		Marxbot marxbot;

		marxbot.pos = Point(10,10);

		REQUIRE( marxbot.pos.x == 10 );
		REQUIRE( marxbot.pos.y == 10 );
	}
}

TEST_CASE( "Khepera", "[Enki::Khepera]" ) {
	SECTION( "The default constructor" ) {
		Khepera khepera;

		REQUIRE( khepera.pos.x == 0 );
		REQUIRE( khepera.pos.y == 0 );
		REQUIRE( khepera.isCylindric() );
		REQUIRE( khepera.getHeight() == 5 );
		REQUIRE( khepera.getRadius() == 2.6 );
	}

	SECTION( "Position" ) {
		Khepera khepera;

		khepera.pos = Point(10,10);

		REQUIRE( khepera.pos.x == 10 );
		REQUIRE( khepera.pos.y == 10 );
	}
}

TEST_CASE( "S-Bot", "[Enki::Sbot]" ) {
	SECTION( "The default constructor" ) {
		Sbot sbot;

		REQUIRE( sbot.pos.x == 0 );
		REQUIRE( sbot.pos.y == 0 );
		REQUIRE( sbot.isCylindric() );
		REQUIRE( sbot.getHeight() == 15 );
		REQUIRE( sbot.getRadius() == 6 );
	}

	SECTION( "Position" ) {
		Sbot sbot;

		sbot.pos = Point(10,10);

		REQUIRE( sbot.pos.x == 10 );
		REQUIRE( sbot.pos.y == 10 );
	}
}
