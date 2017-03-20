#include "Randomizer2.h"

using namespace Enki;

Randomizer::Randomizer(World* world)
{
	this->seed = std::chrono::system_clock::now().time_since_epoch().count();
	this->randomEngine.seed(this->seed);
	this->world = world;
	std::cerr << "seed: " << this->seed << std::endl;
}

Randomizer::Randomizer()
{
	this->seed = std::chrono::system_clock::now().time_since_epoch().count();
	this->randomEngine.seed(this->seed);
	this->world = randomizeWorld();
	std::cerr << "seed: " << this->seed << std::endl;
}

Randomizer::~Randomizer()
{
	delete this->world;
}

long long int Randomizer::getSeed()
{
	return this->seed;
}

void Randomizer::setSeed(const long long int &seed)
{
	this->seed = seed;
	this->randomEngine.seed(this->seed);
	std::cerr << "set seed: " << this->seed << std::endl;
}

World* Randomizer::getWorld()
{
	return this->world;
}

void Randomizer::resetWorld()
{
	delete this->world;
	this->world = new World();
}

World* Randomizer::randomizeWorld()
{
	int wallsType = generateInt(0, 1);
	if(wallsType == World::WALLS_SQUARE)
	{
		int width = generateInt(MIN_WIDTH, MAX_WIDTH);
		int height = generateInt(MIN_HEIGHT, MAX_HEIGHT);
		return new World(width, height);
	}
	else
	{
		int radius = generateInt(MIN_RADIUS, MAX_RADIUS);
		return new World(radius);
	}
}
PhysicalObject* Randomizer::generateObject()
{
	int which = generateInt(0, 1);
	if(which)
	{
		return generateRobot();
	}
	else
	{
		return generatePhysicalObject();
	}
}

PhysicalObject* Randomizer::generatePhysicalObject(const int &hullSize)
{
	PhysicalObject* object = new PhysicalObject();
	object->pos = generatePoint();
	object->setColor(generateColor());

	int cylindric = generateInt(0, 1);

	if(cylindric)
	{
		object->setCylindric(generateFloat(1.0, 5.0), generateFloat(1.0, 5.0), generateFloat(1.0, 5.0));
	}
	else
	{
		object->setCustomHull(generateHull(hullSize), hullSize);
	}

	return object;
}

Robot* Randomizer::generateRobot()
{
	Robot* r;
	unsigned type = generateInt(0, NUMBER_OF_ROBOTS_TYPES - 1);
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

	thymio->setLedIntensity(Thymio2::BUTTON_UP, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::BUTTON_DOWN, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::BUTTON_LEFT, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::BUTTON_RIGHT, generateFloat(0.0, 1.0));

	thymio->setLedIntensity(Thymio2::RING_0, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RING_1, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RING_2, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RING_3, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RING_4, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RING_5, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RING_6, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RING_7, generateFloat(0.0, 1.0));

	thymio->setLedIntensity(Thymio2::IR_FRONT_0, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_1, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_2, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_3, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_4, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::IR_FRONT_5, generateFloat(0.0, 1.0));

	thymio->setLedIntensity(Thymio2::IR_BACK_0, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::IR_BACK_1, generateFloat(0.0, 1.0));

	thymio->setLedIntensity(Thymio2::LEFT_RED, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::LEFT_BLUE, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RIGHT_BLUE, generateFloat(0.0, 1.0));
	thymio->setLedIntensity(Thymio2::RIGHT_RED, generateFloat(0.0, 1.0));

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

PhysicalObject::Hull Randomizer::generateHull(const int &hullSize)
{
	PhysicalObject::Hull hull;

	int size = hullSize <= 0 ? generateInt(1, 30) : hullSize;
	for(int i = 0 ; i < size ; i++)
	{
		int complex = generateInt(0, 1);
		if(complex)
			hull.push_back(generateComplexPart());
		else
			hull.push_back(generateRectanglePart());
	}
	return hull;
}

PhysicalObject::Part Randomizer::generateComplexPart()
{
	Polygone p = generateConvexPolygone(generateInt(3, 30));
	return PhysicalObject::Part(p, generateFloat(1.0, 5.0));
}

PhysicalObject::Part Randomizer::generateRectanglePart()
{
	float size1 = generateFloat(1, 30), size2 = generateFloat(1, 30), height = generateFloat(1, 30);
	return PhysicalObject::Part(size1, size2, height);
}

Polygone Randomizer::generateConvexPolygone(const int &polygoneSize)
{
	int size = polygoneSize < 3 ? 3 : polygoneSize;

	int radius = generateInt(1, 50);
	int center_x = 0;
	int center_y = 0;

	// This uses simple circle based trigonometry to compute a convex polygon.
	std::vector<float> points;
	for(int i = 0 ; i < size ; i++)
	{
		points.push_back(generateFloat(0, 2*M_PI));
	}

	std::sort(points.begin(), points.end());

	Polygone p;
	
	for(int i = 0 ; i < points.size() ; i++)
	{
		float posx = center_x + radius * cos(points.at(i));
		float posy = center_y + radius * sin(points.at(i));
		p << Point(posx, posy);
	}

	points.clear();

	return p;
}

Point Randomizer::generatePoint()
{
	float posx, posy;
	if(this->world->wallsType == World::WALLS_SQUARE)
	{
		posx = generateFloat(0, this->world->w);
		posy = generateFloat(0, this->world->h);
	}
	else if(this->world->wallsType == World::WALLS_CIRCULAR)
	{
		bool in = 0;
		while(!in)
		{
			posx = generateFloat(0, this->world->r);
			posy = generateFloat(0, this->world->r);
			in = (posx * posx) + (posy * posy) <= this->world->r * this->world->r ? true : false;
		}
	}
	return Point(posx, posy);
}

Color Randomizer::generateColor()
{
	float r = generateFloat(0.0, 1.0);
	float g = generateFloat(0.0, 1.0);
	float b = generateFloat(0.0, 1.0);
	return Color(r, g, b);
}

float Randomizer::generateFloat(const float& min, const float &max)
{
	return std::uniform_real_distribution<>(min, max)(this->randomEngine);
}

int Randomizer::generateInt(const int &min, const int &max)
{
	return std::uniform_int_distribution<>(min, max)(this->randomEngine);
}
