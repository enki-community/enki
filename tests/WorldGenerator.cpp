#include "WorldGenerator.h"

namespace Enki
{

	WorldGenerator::WorldGenerator()
	{
		this->world = new World();
		this->randomizer = new Randomizer(this->world);
	}

	WorldGenerator::WorldGenerator(int width, int height)
	{
		this->world = new World(width, height);
		this->randomizer = new Randomizer(this->world);
	}

	WorldGenerator::WorldGenerator(int radius)
	{
		this->world = new World(radius);
		this->randomizer = new Randomizer(this->world);
	}

	WorldGenerator::~WorldGenerator()
	{
		delete world;
		delete randomizer;
	}

	bool WorldGenerator::addRobot(Robot* r)
	{
		int size = this->world->objects.size();
		this->world->addObject(r);
		return size != this->world->objects.size();
	}

	int WorldGenerator::addRobots(int number)
	{
		int cpt(0);
		number = number ? number : 30;
		for(int i = 0 ; i < number ; ++i)
		{
			Robot* r = randomizer->generateRobot();
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
		return this->world;
	}

	void WorldGenerator::resetWorld()
	{
		delete this->world;
		this->world = new World();
	}
}
