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

#ifndef __RANDOMIZER_H
#define __RANDOMIZER_H

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
	\brief This file provides methods for random world generation and
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

		/* We use Mersenne Twister 19937 engine because
		it's the most popular engine for
		PRNG (Pseudo random number generation)
		Mersenne Twister engine procude better output
		than the default engine but it is way slower
		Also the default_randome_engine behavior isn't fixed a it directly depends on the implementation of the libc++ which can change depending on the OS.

		/!\ Careful: This is not thread safe.
		*/
		std::mt19937 randomEngine;

		std::uniform_real_distribution<> color_distr = std::uniform_real_distribution<>(0.0, 1.0);
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
		World* randWorld();

		/*!
			\brief This function generates any kind of enki objects
			such as robots and physical objects.
			\return the generated object.
		*/
		PhysicalObject* randObject();

		/*!
			\brief This function generates a PhysicalObject that is not
			a robot.
			\param hullSize the hull size can be specified.
			\return the generated object.
		*/
		PhysicalObject* randPhysicalObject(const int &hullSize = -1);

		/*!
			\brief This function generates a Robot.
			It can generate: Thymio2, SBot, Epuck, Marxbot, Khepera
			\return the generated robot.
		*/
		Robot* randRobot();
		Thymio2* randThymio();
		EPuck* randEPuck();
		Khepera* randKhepera();
		Sbot* randSbot();
		Marxbot* randMarxbot();

		/*!
			\brief This function generates a Point.
			Generated points depends on the world size which
			means it cannot generate Points outside the current
			"working" world.
			\return return the generated point.
		*/
		Point randPoint();

		/*!
			\brief This function generates a Color.
			\return the generated color.
		*/
		Color randColor();

		/*!
			\brief This function provide a way of generating random
			complex and displayable hulls.
			\param hullSize the size of the generated hull
			(the number of Parts composing the hull).
			\return the generated hull.
		*/

		Texture randTexture(const int &nbColor = -1);

		Textures randTextures(const int &nbTexture = -1);


		PhysicalObject::Hull randHull(const int &hullSize);

		/*!
			\brief This function generates a basic rectangle part.
			\return the generated part.
		*/
		PhysicalObject::Part randRectanglePart();

		/*!
			\brief This function generates a complex Part made of random
			polygones.
			\return the generated part.
		*/
		PhysicalObject::Part randComplexPart();

		/*!
			\brief This function generates random convex polygones.
			\param polygonSize the number of points defining the polygone.
			\return the generated polygone.
		*/
		Polygone randConvexPolygone(const int &polygoneSize = 3);

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
		float randColorFloat();

		/*!
			\brief Generate a random int between 0 and 1
			\return the generated int.
		*/
		int randBool();
	};
}
#endif
