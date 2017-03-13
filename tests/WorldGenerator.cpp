#include "WorldGenerator.h"

namespace Enki
{

	WorldGenerator::WorldGenerator()
	{
		this->randomizer = new Randomizer();
	}

	WorldGenerator::WorldGenerator(const int width, const int height)
	{
		World* w = new World(width, height);
		this->randomizer = new Randomizer(w);
	}

	WorldGenerator::WorldGenerator(const int radius)
	{
		World* w = new World(radius);
		this->randomizer = new Randomizer(w);
	}

	WorldGenerator::~WorldGenerator()
	{
		delete randomizer;
	}

	bool WorldGenerator::add(PhysicalObject* o)
	{
		World* w = this->randomizer->getWorld();
		int size = w->objects.size();
		w->addObject(o);
		return size != w->objects.size();
	}

	bool WorldGenerator::add(int type, int number)
	{
		int cpt(0);
		number = number ? number : this->randomizer->generateInt(30);
		for(int i = 0 ; i < number ; ++i)
		{
			PhysicalObject* o;
			/*
			type can have up to 3 values :
				0 : Add only robots
				1 : Add only objects
			*/
			if (type == 0)
			{
				o = this->randomizer->generateRobot();
			}
			else if (type == 1)
			{
				o = this->randomizer->generatePhysicalObject();
			}
			else
			{
				o = this->randomizer->generateObject();
			}

			if(WorldGenerator::add(o))
				cpt++;
		}
		return cpt == number - 1;
	}

	bool WorldGenerator::add(std::vector<PhysicalObject*> vec)
	{
		int cpt(0);
		for(auto object : vec)
		{
			if(WorldGenerator::add(object))
				cpt++;
		}
		return cpt == vec.size() - 1;
	}

	World* WorldGenerator::getWorld()
	{
		return this->randomizer->getWorld();
	}

	void WorldGenerator::resetWorld()
	{
		this->randomizer->resetWorld();
	}
}
