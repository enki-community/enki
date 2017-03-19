#ifndef __RANDOMIZER2_H
#define __RANDOMIZER2_H

#include <random>
#include <chrono>

#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/s-bot/Sbot.h>
#include <enki/robots/khepera/Khepera.h>
#include <enki/robots/marxbot/Marxbot.h>

namespace Enki
{
	const int NUMBER_OF_ROBOTS_TYPES = 5;

	const int MAX_HEIGHT = 800;
	const int MIN_HEIGHT = 200;
	const int MAX_WIDTH = 800;
	const int MIN_WIDTH = 200;
	const int MIN_RADIUS = 300;
	const int MAX_RADIUS = 500;

	class Randomizer
	{
	public:
		enum ROBOT_TYPES {
			THYMIO2_,
			EPUCK_,
			SBOT_,
			MARXBOT_,
			KHEPERA_,
		};

	private:
		World* world;
		long long int seed;
		// We use Mersenne Twister 19937 engine because
		// it's the most popular engine for
		// PRNG (Pseudo random number generation)
		std::mt19937 randomEngine;

	public:
		Randomizer(World* w);
		Randomizer();
		~Randomizer()
		{};

		long long int getSeed();
		void setSeed(long long int);

		World* getWorld();
		void resetWorld();
		World* randomizeWorld();

		PhysicalObject* generateObject();
		PhysicalObject* generatePhysicalObject(bool displayable = false, int hullSize = -1);
		Robot* generateRobot();
		Thymio2* generateThymio();
		EPuck* generateEPuck();
		Khepera* generateKhepera();
		Sbot* generateSbot();
		Marxbot* generateMarxbot();

		Point generatePoint();
		Color generateColor();
		PhysicalObject::Hull generateHull(int hullSize);
		PhysicalObject::Part generateRectanglePart();
		PhysicalObject::Part generateComplexPart();
		Polygone generateConvexPolygone(int polygonSize = -1);

		float generateFloat(const float& min, const float &max);
		int generateInt(const int &min, const int &max);
	};
}
#endif
