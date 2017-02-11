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

#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <enki/Serialize.h>
#include "Randomizer.h"

const double EPSILON = 0.0001;
const int ITERATION_NUMBER = 10;

using namespace Enki;
using namespace std;

static void printWorld(World* w)
{
	cerr << "- Walls type : " << w->wallsType << endl;
	if (w->wallsType == World::WALLS_CIRCULAR)
	{
		cerr << "- World size : " << w->r << endl;
	}
	else
	{
		cerr << "- World size : " << w->w << "x" << w->h << endl;
	}
	cerr << "- Number of objects : " << w->objects.size() << endl;
	for (PhysicalObject* o : w->objects)
	{
		cerr << "--- Type : " << typeid(*o).name() << endl;
		cerr << "--- PhysicalObject : " << o->pos.x << ";" << o->pos.y << endl;
		cerr << "--- Color : " << endl
			<< "----- " << o->getColor().r() << endl
			<< "----- " << o->getColor().g() << endl
			<< "----- " << o->getColor().b() << endl
			<< "----- " << o->getColor().a() << endl;
	}
}

static bool equalsPoint(Point p1, Point p2)
{
	return (p1.x == p2.x && p1.y == p2.y);
}

static bool equalsColor(Color c1, Color c2)
{
	return (fabs(c1.r() - c2.r()) < EPSILON &&
		fabs(c1.g() - c2.g()) < EPSILON &&
		fabs(c1.b() - c2.b()) < EPSILON &&
		fabs(c1.a() - c2.a()) < EPSILON);
}

// Return whether or not the thymios t1 and t2 are equals
// In order to test the equality we only test the position
// the ledcolor and the angle.
static bool equalsThymio(Thymio2* t1, Thymio2* t2)
{
	if (!equalsPoint(t1->pos, t2->pos))
	{
		cerr << "[T] Not same position" << endl;
		return false;
	}
	for (int i = 0; i < Thymio2::LED_COUNT; i++)
	{
		if (!equalsColor(t1->getColorLed((Thymio2::LedIndex)i),
						 t2->getColorLed((Thymio2::LedIndex)i)))
		{
			cerr << "[T] Not the same color" << endl;
			return false;
		}
	}
	return true;

}

static bool equalsPhysicalObjects(PhysicalObject* o1, PhysicalObject* o2)
{
	// TODO: Implement this function when serialziation/deserialization will be done.
	return true;
}

static bool equalsWorld(World* w1, World* w2)
{
	if (w1->wallsType != w2->wallsType)
	{
		cerr << "[W] Not same wallsType" << endl;
		return false;
	}
	// TODO: Compare ground texture ?
	if ((w1->h != w2->h) || (w1->w != w2->w))
	{
		cerr << "[W] Not same size" << endl;
		return false;
	}
	if (w1->objects.size() != w2->objects.size())
	{
		cerr << "[W] Not same number of objects" << endl;
		return false;
	}

	World::ObjectsIterator it1 = w1->objects.begin();
	World::ObjectsIterator it2 = w2->objects.begin();
	// 2nd part of this boolean expression is useless because of the previous
	// condition but keep it for a better lisibility and understandability.
	while (it1 != w1->objects.end() && it2 != w2->objects.end())
	{
		Thymio2* t1 = dynamic_cast<Thymio2*>(*it1);
		if (t1 != NULL)
		{
			Thymio2* t2 = dynamic_cast<Thymio2*>(*it2);
			if (t2 == NULL || !equalsThymio(t1, t2))
			{
				return false;
			}
			it1++; it2++;
			continue;
		}
		PhysicalObject* o1 = dynamic_cast<PhysicalObject*>(*it1);
		if (o1 == NULL)
		{
			return false;
		}
		PhysicalObject* o2 = dynamic_cast<PhysicalObject*>(*it2);
		if (o2 == NULL || !equalsPhysicalObjects(o1, o2))
		{
			return false;

		}
		it1++; it2++;
	}
	return true;
}

TEST_CASE( "Serialization", "[Serialization Reproducibility]" ) {
	SECTION( "[S] World" ) {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			World* w = randomWorld();

			ostringstream* outputStream = new ostringstream();
			serializeWorld(w, outputStream);

			ostringstream* outputStream2 = new ostringstream();
			serializeWorld(w, outputStream2);

			REQUIRE( outputStream->str() == outputStream2->str() );
		}
	}

	SECTION( "[S] Thymio2" ) {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			World* w = randomWorld();
			Thymio2* t = randomThymio(w);

			ostringstream* outputStream = new ostringstream();
			serializeThymio(t, outputStream);

			ostringstream* outputStream2 = new ostringstream();
			serializeThymio(t, outputStream2);

			REQUIRE( outputStream->str() == outputStream2->str() );
		}
	}

	SECTION( "[S] Color" ) {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			Color c = randomColor();

			ostringstream* outputStream = new ostringstream();
			serializeColor(c, outputStream);

			ostringstream* outputStream2 = new ostringstream();
			serializeColor(c, outputStream2);

			REQUIRE( outputStream->str() == outputStream2->str() );
		}
	}

	SECTION( "[S] World with one Thymio") {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			World* w = randomWorld();
			Thymio2* t = randomThymio(w);
			w->addObject(t);

			std::string outputString = serialize(w);
			std::string outputString2 = serialize(w);

			REQUIRE( outputString == outputString2 );
		}
	}
}

TEST_CASE( "Deserialization", "[Deserialization Reproducibility]") {
	SECTION( "[D] Empty World" ) {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			World* w = randomWorld();

			ostringstream* outputStream = new ostringstream();
			serializeWorld(w, outputStream);

			World* w1 = deserializeWorld(outputStream->str());
			REQUIRE( equalsWorld(w, w1) );

			World* w2 = deserializeWorld(outputStream->str());
			// this assume that w == w2
			REQUIRE( equalsWorld(w1, w2) );
		}
	}

	SECTION( "[D] Thymio2" ) {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			World* w = randomWorld();
			Thymio2* t = randomThymio(w);

			ostringstream* outputStream = new ostringstream();
			serializeThymio(t, outputStream);

			Thymio2* t1 = deserializeThymio(outputStream->str());
			REQUIRE( equalsThymio(t, t1) );

			Thymio2* t2 = deserializeThymio(outputStream->str());
			REQUIRE( equalsThymio(t1, t2) );
		}
	}

	SECTION( "[D] Color" ) {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			Color c = randomColor();

			ostringstream* outputStream = new ostringstream();
			serializeColor(c, outputStream);

			Color c1 = deserializeColor(outputStream->str());
			REQUIRE( equalsColor(c, c1) );

			Color c2 = deserializeColor(outputStream->str());
			REQUIRE( equalsColor(c1, c2) );
		}
	}

	SECTION( "[D] World with one Thymio" ) {
		for (int i = 0; i < ITERATION_NUMBER; i++)
		{
			World* w = randomWorld();
			Thymio2* t = randomThymio(w);
			w->addObject(t);

			std::string outputString = serialize(w);
			World* w1 = deserialize(outputString);

			REQUIRE( equalsWorld(w, w1) );

			World* w2 = deserialize(outputString);
			REQUIRE( equalsWorld(w1, w2) );
		}
	}
}
