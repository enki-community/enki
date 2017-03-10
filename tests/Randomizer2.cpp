#include "Randomizer2.h"

using namespace Enki;

Randomizer::Randomizer(World* world)
{
	this->world = world;
}

Randomizer::Randomizer()
{
	this->world = randomizeWorld();
}

World* Randomizer::getWorld()
{
	return this->world;
}

void Randomizer::resetWorld()
{
	this->world = new World();
}

World* Randomizer::randomizeWorld()
{
	int wallsType = generateInt(2);
	if(wallsType == World::WALLS_SQUARE)
	{
		int width = generateInt(MAX_WIDTH - MIN_WIDTH) + MIN_WIDTH;
		int height = generateInt(MAX_HEIGHT - MIN_HEIGHT) + MIN_HEIGHT;
		std::cerr << "WxH : " << width << "x" << height << std::endl;
		return new World(width, height);
	}
	else
	{
		int radius = generateInt(MAX_RADIUS - MIN_RADIUS) + MIN_RADIUS;
		std::cerr << "R : " << radius << std::endl;
		return new World(radius);
	}
}
PhysicalObject* Randomizer::generateObject()
{
	PhysicalObject* object = new PhysicalObject();
	object->pos = generatePoint();
	object->setColor(generateColor());
	object->setCustomHull(generateHull(), -1);
	return object;
}

Robot* Randomizer::generateRobot()
{
	Robot* r;
	unsigned type = generateInt(NUMBER_OF_ROBOTS_TYPES);
	switch (type)
	{
		case THYMIO2_:
			r = generateThymio();
			break;
		case EPUCK_:
			r = generateEPuck();
			break;
		case SBOT_:
			r = generateSbot();
			break;
		case MARXBOT_:
			r = generateMarxbot();
			break;
		case KHEPERA_:
			r = generateKhepera();
			break;
	}
	return r;
}

Thymio2* Randomizer::generateThymio()
{
	Thymio2* thymio = new Thymio2();

	thymio->setLedColor(Thymio2::TOP, generateColor());
	thymio->setLedColor(Thymio2::BOTTOM_LEFT, generateColor());
	thymio->setLedColor(Thymio2::BOTTOM_RIGHT, generateColor());

	thymio->setLedIntensity(Thymio2::BUTTON_UP, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::BUTTON_DOWN, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::BUTTON_LEFT, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::BUTTON_RIGHT, generateFloat(1.0));

	thymio->setLedIntensity(Thymio2::RING_0, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RING_1, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RING_2, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RING_3, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RING_4, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RING_5, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RING_6, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RING_7, generateFloat(1.0));

	thymio->setLedIntensity(Thymio2::IR_FRONT_0, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_1, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_2, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_3, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_4, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_5, generateFloat(1.0));

	thymio->setLedIntensity(Thymio2::IR_BACK_0, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::IR_BACK_1, generateFloat(1.0));

	thymio->setLedIntensity(Thymio2::LEFT_RED, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::LEFT_BLUE, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RIGHT_BLUE, generateFloat(1.0));
	thymio->setLedIntensity(Thymio2::RIGHT_RED, generateFloat(1.0));

	thymio->pos = generatePoint();
	return thymio;
}

EPuck* Randomizer::generateEPuck()
{
	EPuck* epuck = new EPuck();
	epuck->pos = generatePoint();
	return epuck;
}

Khepera* Randomizer::generateKhepera()
{
	Khepera* khepera = new Khepera();
	khepera->pos = generatePoint();
	return khepera;
}

Sbot* Randomizer::generateSbot()
{
	Sbot* sbot = new Sbot();
	sbot->pos = generatePoint();
	sbot->setColor(generateColor());
	return sbot;
}

Marxbot* Randomizer::generateMarxbot()
{
	Marxbot* marxbot = new Marxbot();
	marxbot->pos = generatePoint();
	marxbot->setColor(generateColor());
	return marxbot;
}

PhysicalObject::Hull Randomizer::generateHull()
{
	return PhysicalObject::Hull();
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
		bool in = 0;
		while(!in)
		{
			posx = generateFloat(this->world->r);
			posy = generateFloat(this->world->r);
			in = (posx * posx) + (posy * posy) <= this->world->r * this->world->r ? true : false;
		}
	}
	std::cerr << "XxY : " << posx << "x" << posy << std::endl;
	return Point(posx, posy);
}

Color Randomizer::generateColor()
{
	float r = generateFloat(1);
	float g = generateFloat(1);
	float b = generateFloat(1);
	return Color(r, g, b);
}

float Randomizer::generateFloat(const float &range)
{
	return UniformRand(0, range)();
}

unsigned Randomizer::generateInt(const unsigned &range)
{
	return intRand(range);
}
