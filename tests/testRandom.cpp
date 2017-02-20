#include <iostream>
#include "WorldGenerator.h"
#include "Randomizer2.h"

using namespace Enki;
int main()
{
	WorldGenerator* gen = new WorldGenerator();
	gen->addRobots(10);
	World* w = gen->getWorld();
	Randomizer rd(w);
	Robot* r = rd.generateThymio();
	std::cout << "World type " << w->wallsType << " & objects number " << w->objects.size() << std::endl;
}
