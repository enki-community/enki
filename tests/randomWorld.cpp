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
const int ITERATION_NUMBER = 100000;

TEST_CASE( "UNIT Testing", "[Enki::RandomWorld.h]" ) {
	SECTION( "A random Int" )
	{
		Randomizer* randomizer = new Randomizer();
		int previousInt = -1;
		int trueRandom = 1;
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			int number = randomizer->generateInt(0, i);
			REQUIRE( (number >= 0 && number <= i) );
			if (number == previousInt)
				trueRandom++;
			previousInt = number;
		}
		REQUIRE(trueRandom != ITERATION_NUMBER);

	}

	SECTION( "A simple dice roll test" )
	{
		Randomizer* randomizer = new Randomizer();
		int dice[6] = {0, 0, 0, 0, 0, 0};
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			int number = randomizer->generateInt(1, 6);
			dice[number-1] += 1;
		}
		for (int i = 0 ; i < 6 ; i++)
		{
			float proba = (dice[i] / (float)ITERATION_NUMBER) * 100;
			std::cerr << "Proba of " << i+1 << ": " << proba << "%" << std::endl;
			REQUIRE( (proba >= 10 && proba <= 20) );
		}
	}

	SECTION( "A random color" ) {
		Randomizer* randomizer = new Randomizer();
		for (int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Color c = randomizer->generateColor();
			REQUIRE( (c.r() >= 0 && c.r() <= 1) );
			REQUIRE( (c.g() >= 0 && c.g() <= 1) );
			REQUIRE( (c.b() >= 0 && c.b() <= 1) );
		}
	}

	// This means that the Point should be within the world area
	// If you want to see if it correctly generate random numbers
	// You should check the first SECTION.
	SECTION( " A random point" ) {
		Randomizer* randomizer = new Randomizer();
		World* w = randomizer->getWorld();
		for (int i = 0 ; i < ITERATION_NUMBER ; i++)
		{
			Point p = randomizer->generatePoint();
			if (w->wallsType == World::WALLS_CIRCULAR)
			{
				REQUIRE( ((p.x * p.y) + (p.x * p.y) <= w->r * w->r) );
			}
			else
			{
				REQUIRE( (p.x <= w->w && p.y < w->h) );
			}
		}
	}

	SECTION( " A random Robot" ) {
		Randomizer* randomizer = new Randomizer();
		World* w = randomizer->getWorld();
		randomizer->generateRobot();
	}
}
/*
	SECTION( "A random Thymio" ) {
		World* w = new World();
		Thymio2* thymio = randomThymio(w);
		REQUIRE( thymio != NULL );
		if (w->wallsType == World::WALLS_SQUARE)
			REQUIRE( (thymio->pos.x <= w->w && thymio->pos.y <= w->h) );
		// TODO: else with CIRCULAR WALLS (Require to compute possible positions
		// from circle radius)
	}

	SECTION( "A random world" ) {
		World* w = randomWorld();
		addObjects(w);
		addRobots(w);
		REQUIRE( w != NULL );
		if (w->wallsType == World::WALLS_CIRCULAR)
		{
			REQUIRE( (w->r <= MAX_RADIUS && w->r >= MIN_RADIUS) );
			REQUIRE( w->objects.size() <= (MAX_DYNAMIC_OBJECTS + MAX_ROBOTS) );
			if (DEBUG)
			{
				cout << "[CIRCULAR WORLD]" << endl;
				cout << "- radius : " << w->r << endl;
			}
		}
		else if (w->wallsType == World::WALLS_SQUARE)
		{
			REQUIRE( (w->w <= MAX_WIDTH && w->w >= MIN_WIDTH) );
			REQUIRE( (w->h <= MAX_HEIGHT && w->h >= MIN_HEIGHT) );
			if (DEBUG)
			{
				cout << "[SQUARED WORLD]" << endl;
				cout << "- w : " << w->w << endl;
				cout << "- h : " << w->h << endl;
			}
		}
		else
		{
			//TODO: Deal with WALLS_NONE ?
			assert(false);
		}
		// Get this out of if to prevent duplication
		REQUIRE( w->objects.size() <= (MAX_DYNAMIC_OBJECTS + MAX_ROBOTS) );
		if (DEBUG)
			cout << "- number of objects : " << w->objects.size() << endl;
		for (PhysicalObject* p : w->objects)
		{
			Thymio2* t = randomThymio(w);
			PhysicalObject* o = randomObject(w);
			if (DEBUG)
			{
				cout << "Actual object : " << typeid(*p).name() << endl;
			}
			// TODO : Isolate this part in a genericity test since Robots are
			// generated by the same method RandomWorld-> randomRobot(World* w)
			REQUIRE( (typeid(*p) == typeid(*t) || typeid(*p) == typeid(*o)) );
		}
	}
}
*/
