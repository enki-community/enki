#include <iostream>
#include "WorldGenerator.h"
#include "Randomizer2.h"
#include <typeinfo>

using namespace Enki;
int main()
{
	WorldGenerator* gen = new WorldGenerator;
	gen->add(PHYSICAL_OBJECTS_, 10);
	World* w = gen->getWorld();
	std::cout << "World[" << w->wallsType << "]\n -WxHxR: " << w->w << "x" << w->h <<"x" << w->r <<"\n -OBJ NUM: " << w->objects.size() << std::endl << " -OBJ LIST:" << std::endl;
	for(auto s : w->objects)
		{
			std::cout << "  *" << typeid(*s).name() << std::endl;
		}

}
