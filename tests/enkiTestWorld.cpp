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

#define CATCH_CONFIG_MAIN  // Tells Catch to provide a main()
#include "catch.hpp"

#include <enki/PhysicalEngine.h>

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
