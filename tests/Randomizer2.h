#ifndef __RANDOMIZER2_H
#define __RANDOMIZER2_H

#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/s-bot/Sbot.h>
#include <enki/robots/khepera/Khepera.h>
#include <enki/robots/marxbot/Marxbot.h>

namespace Enki
{
	const int NUMBER_OF_ROBOTS_TYPES = 5;
	// Those enumeration items end with a '_' so that they don't collide with
	// the ones used in the serialization module.
	enum ROBOT_TYPES { THYMIO2_, EPUCK_, SBOT_, MARXBOT_, KHEPERA_ };

	const int MAX_HEIGHT = 800;
	const int MIN_HEIGHT = 200;
	const int MAX_WIDTH = 800;
	const int MIN_WIDTH = 200;
	const int MIN_RADIUS = 300;
	const int MAX_RADIUS = 500;

	class Randomizer
	{
	private:
		World* world;

	public:
		Randomizer(World* w);
		Randomizer();
		~Randomizer()
		{};

		World* getWorld();
		void resetWorld();
		World* randomizeWorld();

		PhysicalObject* generateObject();
		PhysicalObject* generatePhysicalObject();
		Robot* generateRobot();
		Thymio2* generateThymio();
		EPuck* generateEPuck();
		Khepera* generateKhepera();
		Sbot* generateSbot();
		Marxbot* generateMarxbot();

		Point generatePoint();
		Color generateColor();
		PhysicalObject::Hull generateHull();

		float generateFloat(const float &range);
		unsigned generateInt(const unsigned &range);
	};
}
#endif
