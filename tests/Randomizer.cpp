/*
   Enki - a fast 2D robot simulator
   Copyright © 2017 Nicolas Palard <nicolas.palard@etu.u-bordeaux.fr>
   Copyright © 2017 Mathieu Lirzin <mathieu.lirzin@etu.u-bordeaux.fr>

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

#include "Randomizer.h"
#include <random>
#include <chrono>

/*!	\file Randomizer.cpp
	\brief Module 'random' which can be used for generating objects and worlds.
*/

namespace Enki
{
	// Generate a seed and log it to OS.  The Logging is made to be able to
	// reproduce the tests using this module.
	static int genSeed(std::ostream& os = std::cerr)
	{
		auto timepoint = std::chrono::system_clock::now();
		int seed = timepoint.time_since_epoch().count();
		os << "seed: " << seed << std::endl;
		return seed;
	}

	int randomNumber(int min, int max)
	{
		// XXX: UniformRand from Random.h doesn't seem to really produce uniform
		// random values.  Use a random generator from the STL instead.

		// Since mt19937 engine takes a lot of space and time to generate avoid
		// building it every time by making it static.
		static std::mt19937 engine(genSeed());
		std::uniform_int_distribution<> distr(min, max);
		return distr(engine);
	}

	Color randomColor()
	{
		// Generate a random color
		int r = randomNumber(0,255);
		int g = randomNumber(0,255);
		int b = randomNumber(0,255);
		return Color((float)r/255.0, (float)g/255.0, (float)b/255.0);
	}

	PhysicalObject* randomObject(World* world)
	{
		// Generate a "random" PhysicalObject
		const double amount = 9;
		const double radius = randomNumber(3,9);
		const double height = randomNumber(10, 20);

		Polygone p;
		for (double a = 0; a < 2*M_PI; a += 2*M_PI/amount)
			p.push_back(Point(radius * cos(a), radius * sin(a)));

		PhysicalObject* o = new PhysicalObject();
		PhysicalObject::Hull hull(PhysicalObject::Part(p, height));
		o->setCustomHull(hull, -1);
		Color color = randomColor();
		o->setColor(color);
		int posx = randomNumber(0, world->w);
		int posy = randomNumber(0, world->h);
		o->pos = Point(posx, posy);
		return o;
	}

	Robot* randomRobot(World* world)
	{
		int type = (ONLY_THYMIOS)
			? THYMIO2_
			: randomNumber(1, NUMBER_OF_ROBOTS_TYPES);

		Robot* r;
		switch (type)
		{
			case THYMIO2_:
				r = randomThymio(world);
				break;
			case EPUCK_:
				r = randomEPuck(world);
				break;
			case SBOT_:
				r = randomSbot(world);
				break;
			case MARXBOT_:
				r = randomMarxbot(world);
				break;
			case KHEPERA_:
				r = randomKhepera(world);
				break;
		}
		return r;
	}

	Thymio2* randomThymio(World* world)
	{
		// Generate a random Thymio2
		Thymio2 *thymio = new Thymio2;
		int posx, posy;
		if (world->wallsType == Enki::World::WALLS_CIRCULAR)
		{
			int radius = world->r;
			// XXX: This assumes that the world isn't centered in 0,0
			int center = radius/2;
			posx = randomNumber(0, radius);
			posy = randomNumber(0, radius);
			while ((posx - center) * (posx - center) +
				   (posy - center) * (posy - center) > (radius * radius))
			{
				posx = randomNumber(0, radius);
				posy = randomNumber(0, radius);
			}
		}
		else if (world->wallsType == Enki::World::WALLS_SQUARE)
		{
			posx = randomNumber(0, world->w);
			posy = randomNumber(0, world->h);
		}
		thymio->pos = Point(posx, posy);

		thymio->setLedColor(Thymio2::TOP, randomColor());
		thymio->setLedColor(Thymio2::BOTTOM_LEFT, randomColor());
		thymio->setLedColor(Thymio2::BOTTOM_RIGHT, randomColor());

		// Copy paste from Playground.cpp
		thymio->setLedIntensity(Thymio2::BUTTON_UP,1.0);
		thymio->setLedIntensity(Thymio2::BUTTON_DOWN,1.0);
		thymio->setLedIntensity(Thymio2::BUTTON_LEFT,1.0);
		thymio->setLedIntensity(Thymio2::BUTTON_RIGHT,1.0);

		thymio->setLedIntensity(Thymio2::RING_0,1.0);
		thymio->setLedIntensity(Thymio2::RING_1,1.0);
		thymio->setLedIntensity(Thymio2::RING_2,1.0);
		thymio->setLedIntensity(Thymio2::RING_3,1.0);
		thymio->setLedIntensity(Thymio2::RING_4,1.0);
		thymio->setLedIntensity(Thymio2::RING_5,1.0);
		thymio->setLedIntensity(Thymio2::RING_6,1.0);
		thymio->setLedIntensity(Thymio2::RING_7,1.0);

		thymio->setLedIntensity(Thymio2::IR_FRONT_0,1.0);
		thymio->setLedIntensity(Thymio2::IR_FRONT_1,1.0);
		thymio->setLedIntensity(Thymio2::IR_FRONT_2,1.0);
		thymio->setLedIntensity(Thymio2::IR_FRONT_3,1.0);
		thymio->setLedIntensity(Thymio2::IR_FRONT_4,1.0);
		thymio->setLedIntensity(Thymio2::IR_FRONT_5,1.0);

		thymio->setLedIntensity(Thymio2::IR_BACK_0,1.0);
		thymio->setLedIntensity(Thymio2::IR_BACK_1,1.0);

		thymio->setLedIntensity(Thymio2::LEFT_RED,1.0);
		thymio->setLedIntensity(Thymio2::LEFT_BLUE,1.0);
		thymio->setLedIntensity(Thymio2::RIGHT_BLUE,1.0);
		thymio->setLedIntensity(Thymio2::RIGHT_RED,1.0);

		thymio->leftSpeed = randomNumber(1, 5);
		thymio->rightSpeed = randomNumber(1, 5);

		return thymio;
	}

	EPuck* randomEPuck(Enki::World* w)
	{
		std::cerr << "[EPUCK] Unimplemented method" << std::endl;
		assert(false);
		return NULL;
	}

	Sbot* randomSbot(Enki::World* w)
	{
		std::cerr << "[SBOT] Unimplemented method" << std::endl;
		assert(false);
		return NULL;
	}

	Marxbot* randomMarxbot(Enki::World* w)
	{
		std::cerr << "[MARXBOT] Unimplemented method" << std::endl;
		assert(false);
		return NULL;
	}

	Khepera* randomKhepera(Enki::World* w)
	{
		std::cerr << "[KHEPERA] Unimplemented method" << std::endl;
		assert(false);
		return NULL;
	}

	void addObjects(World* world)
	{
		int objectsNumber = randomNumber(1, MAX_DYNAMIC_OBJECTS);
		for (int i = 0; i < objectsNumber; i++)
		{
			PhysicalObject* o = randomObject(world);
			world->addObject(o);
		}
	}

	void addRobots(World* world)
	{
		// Random number of thymios between 1 and MAX_ROBOTS
		int robotNumber = randomNumber(1, MAX_ROBOTS);
		for (int i = 0 ; i < robotNumber ; i++)
		{
			Robot* r = randomRobot(world);
			world->addObject(r);
		}
	}

	World* randomWorld()
	{
		World* world;
		int circularWorld = randomNumber(0,1);
		if (circularWorld)
		{
			int radius = randomNumber(MIN_RADIUS, MAX_RADIUS);
			world = new World(radius);
		}
		else
		{
			int height = randomNumber(MIN_HEIGHT, MAX_HEIGHT);
			int width = randomNumber(MIN_WIDTH, MAX_WIDTH);
			world = new World(width, height);
		}

		return world;
	}
}
