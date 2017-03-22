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

/*!
   \file Randomizer.h
   \brief This file provides a way for random world generation and
   random object generation.
*/

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
		// Mersenne Twister engine procude better output
		// than the default engine but it is way slower
		// /!\ This is not thread safe.
		std::mt19937 randomEngine;

		std::uniform_real_distribution<> color_distr = std::uniform_real_distribution<>(0, 1);
		std::uniform_int_distribution<> bool_distr = std::uniform_int_distribution<>(0, 1);

	public:
		/*!
		   \brief Constructor of a Randomizer.
		    A randomizer aims to generate random Enki Worlds and Enki
		   Objects.
		   \param w a pre-constructed world in which it will generate objects.
		*/
		Randomizer(World* w, long long int seed=-1);

		/*!
		   \brief Constructor of a Randomizer.
		   A randomizer aims to generate random Enki Worlds and Enki
		   Objects.
		   This constructor will generate a random world because no
		   base world is specified.
		*/
		Randomizer(long long int seed = -1);

		~Randomizer();

		/*!
		   \brief This function allows debug by returning the seed
		   of the current random engine.
		   \return the seed of the current random engine.
		*/
		long long int getSeed();

		/*!
		   \brief This functions allows to set a custom seed to
		   the current random engine.
		   \param seed the seed to set.
		*/
		void setSeed(const long long int &seed);

		/*!
		   \brief This functions allows the user to work on a
		   random world outside of the randomizer module.
		   This can be used to tweak worlds.
		   \return the actual randomized world.
		*/
		World* getWorld();

		/*!
		   \brief This function is used to start a new world generation.
		*/
		void resetWorld();

		/*!
		   \brief This function aims to create an empty randomized world.
		   It is used in the constructor when no world is gave.
		   It only creates the basis of a World : it's type and size.
		   \return the created world.
		*/
		World* randomizeWorld();

		/*!
		   \brief This function generates any kind of enki objects
		   such as robots and physical objects.
		   \return the generated object.
		*/
		PhysicalObject* generateObject();

		/*!
		   \brief This function generates a PhysicalObject that is not
		   a robot.
		   \param hullSize the hull size can be specified.
		   \return the generated object.
		*/
		PhysicalObject* generatePhysicalObject(const int &hullSize = -1);

		/*!
		   \brief This function generates a Robot.
		   It can generate: Thymio2, SBot, Epuck, Marxbot, Khepera
		   \return the generated robot.
		*/
		Robot* generateRobot();
		Thymio2* generateThymio();
		EPuck* generateEPuck();
		Khepera* generateKhepera();
		Sbot* generateSbot();
		Marxbot* generateMarxbot();

		/*!
		   \brief This function generates a Point.
		   Generated points depends on the world size which
		   means it cannot generate Points outside the current
		   "working" world.
		   \return return the generated point.
		*/
		Point generatePoint();

		/*!
		   \brief This function generates a Color.
		   \return the generated color.
		*/
		Color generateColor();

		/*!
		   \brief This function provide a way of generating random
		   complex and displayable hulls.
		   \param hullSize the size of the generated hull
		   (the number of Parts composing the hull).
		   \return the generated hull.
		*/
		PhysicalObject::Hull generateHull(const int &hullSize);

		/*!
		   \brief This function generates a basic rectangle part.
		   \return the generated part.
		*/
		PhysicalObject::Part generateRectanglePart();

		/*!
		   \brief This function generates a complex Part made of random
		   polygones.
		   \return the generated part.
		*/
		PhysicalObject::Part generateComplexPart();

		/*!
		   \brief This function generates random convex polygones.
		   \param polygonSize the number of points defining the polygone.
		   \return the generated polygone.
		*/
		Polygone generateConvexPolygone(const int &polygoneSize = 3);

		/*!
		   \brief Generate a random float within a specified range.
		   \param min the min possible value.
		   \param max the max possible value.
		   \return the generated float.
		*/
		float randFloat(const float& min, const float &max);

		/*!
		   \brief Generate a random int within a specified range.
		   \param min the min possible value.
		   \param max the max possible value.
		   \return the generated int.
		*/
		int randInt(const int &min, const int &max);

		/*!
			\brief Generate a random float between 0 and 1
			\return the generated float.
		*/
		float randColor();

		/*!
			\brief Generate a random int between 0 and 1
			\return the generated int.
		*/
		int randBool();
	};
}
#endif
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

		float colorFloat();
		float generateFloat(const float& min, const float &max);
		int generateInt(const int &min, const int &max);
	};
}
#endif
