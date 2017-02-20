#include "Randomizer2.h"

using namespace Enki;

Randomizer::Randomizer(World* world)
{
	this->world = world;
}

Robot* Randomizer::generateRobot()
{
	unsigned type = generateInt(1);
	Robot* r;
	switch(type)
	{
		case 0:
		{
			r = generateThymio();
			break;
		}
		case 1:
		{
			r = generateEPuck();
			break;
		}
		default:
		{
			break;
		}
	}

	return r;
}

Point Randomizer::generatePoint()
{
	float posx, posy;
	if(this->world->wallsType == World::WALLS_SQUARE)
	{
		posx = generateFloat(this->world->w);
		posy = generateFloat(this->world->h);
	}
	else if(this->world->wallsType == World::WALLS_CIRCULAR)
	{
		posx = 10;
		posy = 20;
	}
	return Point(posx, posy);
}


Thymio2* Randomizer::generateThymio()
{
	Thymio2* thymio = new Thymio2();
	thymio->pos = generatePoint();
	return thymio;
}

EPuck* Randomizer::generateEPuck()
{
	return new EPuck();
}

float Randomizer::generateFloat(float range)
{
	return UniformRand(0, range)();
}

unsigned Randomizer::generateInt(unsigned range)
{
	return Enki::intRand(range);
}
