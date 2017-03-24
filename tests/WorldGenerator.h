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

#ifndef __WORLD_GENERATOR_H
#define __WORLD_GENERATOR_H

#include <enki/PhysicalEngine.h>
#include "Randomizer.h"
#include <iostream>

/*!
	\file WorldGenerator.h
	\brief This file provide a simple way of generating
	complex worlds
*/

namespace Enki
{
	#define PHYSICAL_OBJECTS_ 5
	#define ONLY_ROBOTS_ 6
	#define ANYTHING_ 7

	class WorldGenerator
	{

	private:
		World* world;
		Randomizer* randomizer;
	public:
		/*!
			\brief World generator constructor.
			A WorldGenerator is used to simply create complex worlds.
			\return a new WorldGenerator object.
		*/
		WorldGenerator();

		/*!
			\brief World generator constructor.
			\param width the width of the world to generate
			\param height the height of the world to generate
			\return a new WorldGenerator object with a pre-generated world
			of width x height.
			The pre-generated world wallsType will be WALLS_SQUARED.
		*/
		WorldGenerator(const int &width, const int &height);

		/*!
			\brief World generator constructor.
			\param radius the radius of the world to generate
			\return a new WorldGenerator object with a pre-generated world.
			The pre-generated world wallsType will be WALLS_CIRCULAR.
		*/
		WorldGenerator(const int &radius);
		~WorldGenerator();

		/*!
			\brief This function aims to provide a way of adding a non-random physical object to the current "working" world.
			A physical object can be a PhysicalObject or a Robot of any type.
			\param o a pointer to a PhysicalObject.
			\return true if the object has been successfully added into the world, otherwise false.
		*/
		bool add(PhysicalObject* o);

		/*!
			\brief  This function aims to provide a way of adding a bunch of non-random hand-made physical objects to the current "working" world.
			\param vec a vector of PhysicalObject*
			\return true if all the objects contained in the vector have been added to the world, otherwise false.
		*/
		bool add(std::vector<PhysicalObject*> vec);

		/*!
			\brief This functions aims to provide a way of adding anything you want in the current "working" world.
			\param type the type of the objects you want to add.
			If type == -1, it will randomly add objects to the world
			\param number the number of objects of type "type" you want to add.
			This parameter is optional, and if not specified, the generator will add up to 30 objects of type "type" into the current working world.
			\return true if all the objects have been added to the world, otherwise false.
		*/
		bool add(const int &type, const int &number = 0);

		/*!
			\brief This function aims to provide a way of getting the current
			World object.
			\return a pointer to the current world.
		*/
		World* getWorld();

		/*!
			\brief This functions aims to provide a way of reseting the current
			world generation.
			It should be used to start a new generation instead of re-creating
			a world generator object.
		*/
		void resetWorld();

		/*!
			\brief This is a debugging function that allows you to get the current Randomizer Object used to generate worlds.
			The Randomizer provides a lot of methods in order to add pseudo random generated objects to the world.
			See the end-to-end test-file: e2eWorldGen.cpp for more details.
		*/
		Randomizer* getRandomizer();
	};
}
#endif
