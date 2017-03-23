/*
   Enki - a fast 2D robot simulator
   Copyright © 2017 Nicolas Palard <nicolas.palard@etu.u-bordeaux.fr>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "catch.hpp"

#include "WorldGenerator.h"
#include <enki/PhysicalEngine.h>

/*!	\file randomWorld.cpp
	\brief tests for 'random' module
*/

using namespace std;
using namespace Enki;

// Used to activate/deactive DEBUG informations
#define DEBUG 0

// Used to ensure the robustness of the tested code.
const int ITERATION_NUMBER = 100;

TEST_CASE( "UNIT Testing", "[Enki::RandomWorld.h]" ) {

	Randomizer* globalRandomizer = new Randomizer(1490219936065061490);

	SECTION( "Random integer generation" )
	{
		int previousInt = -1;
		int trueRandom = 1;
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			int number = globalRandomizer->randInt(0, i);
			REQUIRE( (number >= 0 && number <= i) );
			if (number == previousInt)
				trueRandom++;
			previousInt = number;
		}
		REQUIRE(trueRandom != ITERATION_NUMBER - 1);

	}

	SECTION( "Dice roll test" )
	{
		int dice[6] = {0, 0, 0, 0, 0, 0};
		for (int i = 0; i < 10000; i++)
		{
			int number = globalRandomizer->randInt(1, 6);
			dice[number-1] += 1;
		}
		for (int i = 0 ; i < 6 ; i++)
		{
			float proba = (dice[i] / (float)10000) * 100;
			if (DEBUG)
				std::cerr << "Proba of " << i+1 << ": " << proba << "%" << std::endl;
			REQUIRE( (proba >= 10 && proba <= 20) );
		}
	}

	SECTION( "Random float generation") {
		for (int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			float number = globalRandomizer->randFloat(0, 1);
			REQUIRE ( (number >= 0 && number <= 1) );
		}
	}

	SECTION( "A random color" ) {
		for (int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Color c = globalRandomizer->randColor();
			REQUIRE( (c.r() >= 0 && c.r() <= 1) );
			REQUIRE( (c.g() >= 0 && c.g() <= 1) );
			REQUIRE( (c.b() >= 0 && c.b() <= 1) );
		}
	}

	// This means that the Point should be within the world area
	// If you want to see if it correctly rands random numbers
	// You should check the first SECTION.
	SECTION ( "A random position in the world" ) {
		World* w = globalRandomizer->getWorld();
		for (int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Point p =globalRandomizer->randPoint();
			if (w->wallsType == World::WALLS_CIRCULAR)
			{
				REQUIRE ( ((p.x * p.x) + (p.y * p.y) <= w->r * w->r) );
			}
			else
			{
				REQUIRE ( (p.x <= w->w && p.y <= w->h) );
			}
		}
	}

	SECTION( "A random convex polygon" ) {
		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Polygone p = globalRandomizer->randConvexPolygone(i);
			REQUIRE( p.size() >= 3 );
		}

	}

	SECTION ( "A random part" ) {
		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			PhysicalObject::Part p = globalRandomizer->randComplexPart();
			REQUIRE( (p.getShape().size() >= 3) );
			REQUIRE( p.getHeight() == 1 );
		}
		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			PhysicalObject::Part p = globalRandomizer->randRectanglePart();
			REQUIRE( (p.getHeight() >= 1 && p.getHeight() <= 30) );
			REQUIRE( (p.getArea() <= (30 * 30)) );
		}
	}

	SECTION ( "A random texture" ) {
		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Texture t = globalRandomizer->randTexture();
			REQUIRE ( (t.size() >= 1 && t.size() <= 5) );
			Texture t1 = globalRandomizer->randTexture(i+1);
			REQUIRE( (t1.size() == i+1) );
		}

		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Textures t = globalRandomizer->randTextures();
			REQUIRE ( (t.size() >= 1 && t.size() <= 5) );
			Textures t1 = globalRandomizer->randTextures(i+1);
			REQUIRE( (t1.size() == i+1) );

		}
	}


	SECTION ( "A random Hull" ) {
		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			PhysicalObject::Hull hull = globalRandomizer->randHull(i);
			if(i > 0)
				REQUIRE( hull.size() == i);
			else
				REQUIRE( hull.size() > 0);
		}
	}

	SECTION( "A random empty world" ) {

		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			World* w = globalRandomizer->randWorld();

			if(w->wallsType == World::WALLS_SQUARE)
			{
				REQUIRE ( (w->r == 0 &&
					(w->h >= MIN_HEIGHT && w->h <= MAX_HEIGHT) &&
					(w->w >= MIN_WIDTH && w->w <= MAX_WIDTH) ) );
			}
			else
			{
				REQUIRE ( ((w->r >= MIN_RADIUS && w->r <= MAX_RADIUS) &&
				w->h == 0 &&
				w->w == 0) );
			}

			REQUIRE ( (w->objects.size() == 0) );

			delete w;
		}
	}

	SECTION ( "A random Robot" ) {
		globalRandomizer->randWorld();
		World* w =globalRandomizer->getWorld();

		for(int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Robot* robot = globalRandomizer->randRobot();
			REQUIRE ( robot != NULL );

			if(w->wallsType == World::WALLS_SQUARE)
			{
				REQUIRE ( (robot->pos.x <= w->w && robot->pos.y <= w->h) );
			}
			else if (w->wallsType == World::WALLS_CIRCULAR)
			{
				REQUIRE ( ((robot->pos.x * robot->pos.x) +
				(robot->pos.y * robot->pos.y) <= w->r * w->r) );
			}

			delete robot;
		}
	}

	SECTION( "A random physical object" ) {

		globalRandomizer->randWorld();
		World* w =globalRandomizer->getWorld();

		// TODO : Remove this debug
		//std::cerr << "[WxHxR]: " << w->w << "x" << w->h << "x" << w->r <<  std::endl;

		for(int i = 0 ; i < 100000 ; i++)
		{
			PhysicalObject* obj = globalRandomizer->randPhysicalObject();
			REQUIRE ( obj != NULL );

			if(w->wallsType == World::WALLS_SQUARE)
			{
				/*
				if( obj->pos.x > w->w )
				{
					std::cerr << "Failed x: " <<  obj->pos.x << " <= " << w->w << " ? with seed: " << globalRandomizer->getSeed() << std::endl;
				}
				if( obj->pos.y > w->h )
				{
					std::cerr << "Failed y:" <<  obj->pos.y << " <= " << w->h << " ? with seed: " << globalRandomizer->getSeed() << std::endl;
				}
				*/
				REQUIRE ( (obj->pos.x <= w->w && obj->pos.y <= w->h) );
			}
			else if (w->wallsType == World::WALLS_CIRCULAR)
			{
				REQUIRE ( (obj->pos.x <= w->r && obj->pos.y <= w->r) );
			}

			delete obj;
		}
	}

	delete globalRandomizer;
}
