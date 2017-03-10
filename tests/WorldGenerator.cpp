#include "WorldGenerator.h"

namespace Enki
{

	WorldGenerator::WorldGenerator()
	{
		this->randomizer = new Randomizer();
	}

	WorldGenerator::WorldGenerator(int width, int height)
	{
		World* w = new World(width, height);
		this->randomizer = new Randomizer(w);
	}

	WorldGenerator::WorldGenerator(int radius)
	{
		World* w = new World(radius);
		this->randomizer = new Randomizer(w);
	}

	WorldGenerator::~WorldGenerator()
	{
		delete randomizer;
	}

	bool WorldGenerator::addRobot(Robot* r)
	{
		World* w = this->randomizer->getWorld();
		int size = w->objects.size();
		w->addObject(r);
		return size != w->objects.size();
	}

	int WorldGenerator::addRobots(int number)
	{
		int cpt(0);
		number = number ? number : this->randomizer->generateInt(30);
		for(int i = 0 ; i < number ; ++i)
		{
			Robot* r = this->randomizer->generateRobot();
			if(WorldGenerator::addRobot(r))
				cpt++;
		}
		return cpt;
	}

	int WorldGenerator::addRobots(std::vector<Robot*> vec)
	{
		int cpt(0);
		for(auto robot : vec)
		{
			if(WorldGenerator::addRobot(robot))
				cpt++;
		}
		return cpt;
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
