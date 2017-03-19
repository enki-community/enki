#ifndef __WORLD_GENERATOR_H
#define __WORLD_GENERATOR_H

#include <enki/PhysicalEngine.h>
#include "Randomizer2.h"
#include <iostream>

namespace Enki
{
	// Those enumeration items end with a '_' so that they don't collide with
	// the ones used in the serialization module.

	class WorldGenerator
	{

	private:
		World* world;
		Randomizer* randomizer;
	public:
		WorldGenerator();
		WorldGenerator(const int width, const int height);
		WorldGenerator(const int radius);
		~WorldGenerator();

		bool add(PhysicalObject* o); // Can be used to add PhysicalObjects such as Robots & Objects
		bool add(std::vector<PhysicalObject*> vec);
		bool add(int type, int number = 0); // Add depending on the type

		World* getWorld();
		void resetWorld();

		Randomizer* getRandomizer();
	};
}
#endif
