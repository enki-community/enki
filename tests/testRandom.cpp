#include <iostream>
#include "WorldGenerator.h"
#include "Randomizer2.h"
#include <typeinfo>

using namespace Enki;
int main()
{
	WorldGenerator* gen = new WorldGenerator;
	gen->add(-1);
	World* w = gen->getWorld();
	Randomizer rd(w);
	Robot* r = rd.generateThymio();
	std::cout << "World type " << w->wallsType << " & objects number " << w->objects.size() << std::endl;
	for(auto s : w->objects)
		{
			std::cerr << typeid(*s).name() << std::endl;
		}

}
