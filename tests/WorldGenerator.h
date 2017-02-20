#ifndef __WORLD_GENERATOR_H
#define __WORLD_GENERATOR_H

#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include "Randomizer2.h"
#include <iostream>

namespace Enki
{

	class WorldGenerator{
	private:
		World* world;
		Randomizer* randomizer;
	public:
		WorldGenerator();
		WorldGenerator(int width, int height);
		WorldGenerator(int radius);
		~WorldGenerator();

		bool addRobot(Robot* r); // Can be used to add Thymio, EPuck etc
		int addRobots(int number = 0);
		int addRobots(std::vector<Robot*> vec);

		World* getWorld();
		void resetWorld();
	};
}
#endif
