#include "Randomizer2.h"

using namespace Enki;

Randomizer::Randomizer(World* world, long long int seed)
{

	this->seed = seed == -1  ? std::chrono::system_clock::now().time_since_epoch().count() : seed;
	this->randomEngine.seed(this->seed);
	this->world = world;
	std::cerr << "seed: " << this->seed << std::endl;
}

Randomizer::Randomizer(long long int seed)
{
	/*
	Since c++11 you can call something like "super" constructor.
	Something like : Randomizer(randomizeWorld(), seed); would
	be more convinient but this doesn't work as the seed needs to be set
	before any call of randomizeWorld();
	That's why there is a code duplication
	*/
	this->seed = seed == -1 ? std::chrono::system_clock::now().time_since_epoch().count() : seed;
	this->randomEngine.seed(this->seed);
	this->world = randWorld();
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

World* Randomizer::randWorld()
{
	int wallsType = randInt(0, 2);
	if(wallsType == World::WALLS_SQUARE || wallsType == World::WALLS_NONE)
	{
		int width = randInt(MIN_WIDTH, MAX_WIDTH);
		int height = randInt(MIN_HEIGHT, MAX_HEIGHT);
		return new World(width, height);
	}
	else if(wallsType == World::WALLS_CIRCULAR)
	{
		int radius = randInt(MIN_RADIUS, MAX_RADIUS);
		return new World(radius);
	}
}
PhysicalObject* Randomizer::randObject()
{
	int which = randBool();
	if(which)
	{
		return randRobot();
	}
	else
	{
		return randPhysicalObject();
	}
}

PhysicalObject* Randomizer::randPhysicalObject(const int &hullSize)
{
	PhysicalObject* object = new PhysicalObject();
	object->pos = randPoint();
	object->setColor(randColor());

	int cylindric = randBool();

	if(cylindric)
	{
		object->setCylindric(randFloat(1.0, 5.0), randFloat(1.0, 5.0), randFloat(1.0, 5.0));
	}
	else
	{
		object->setCustomHull(randHull(hullSize), hullSize);
	}

	return object;
}

Robot* Randomizer::randRobot()
{
	Robot* r;
	unsigned type = randInt(0, NUMBER_OF_ROBOTS_TYPES - 1);
	switch (type)
	{
		case THYMIO2_:
			r = randThymio();
			break;
		case EPUCK_:
			r = randEPuck();
			break;
		case SBOT_:
			r = randSbot();
			break;
		case MARXBOT_:
			r = randMarxbot();
			break;
		case KHEPERA_:
			r = randKhepera();
			break;
	}
	return r;
}

Thymio2* Randomizer::randThymio()
{
	Thymio2* thymio = new Thymio2();

	thymio->setLedColor(Thymio2::TOP, randColorFloat());
	thymio->setLedColor(Thymio2::BOTTOM_LEFT, randColorFloat());
	thymio->setLedColor(Thymio2::BOTTOM_RIGHT, randColorFloat());

	thymio->setLedIntensity(Thymio2::BUTTON_UP, randColorFloat());
	thymio->setLedIntensity(Thymio2::BUTTON_DOWN, randColorFloat());
	thymio->setLedIntensity(Thymio2::BUTTON_LEFT, randColorFloat());
	thymio->setLedIntensity(Thymio2::BUTTON_RIGHT, randColorFloat());

	thymio->setLedIntensity(Thymio2::RING_0, randColorFloat());
	thymio->setLedIntensity(Thymio2::RING_1, randColorFloat());
	thymio->setLedIntensity(Thymio2::RING_2, randColorFloat());
	thymio->setLedIntensity(Thymio2::RING_3, randColorFloat());
	thymio->setLedIntensity(Thymio2::RING_4, randColorFloat());
	thymio->setLedIntensity(Thymio2::RING_5, randColorFloat());
	thymio->setLedIntensity(Thymio2::RING_6, randColorFloat());
	thymio->setLedIntensity(Thymio2::RING_7, randColorFloat());

	thymio->setLedIntensity(Thymio2::IR_FRONT_0, randColorFloat());
	thymio->setLedIntensity(Thymio2::IR_FRONT_1, randColorFloat());
	thymio->setLedIntensity(Thymio2::IR_FRONT_2, randColorFloat());
	thymio->setLedIntensity(Thymio2::IR_FRONT_3, randColorFloat());
	thymio->setLedIntensity(Thymio2::IR_FRONT_4, randColorFloat());
	thymio->setLedIntensity(Thymio2::IR_FRONT_5, randColorFloat());

	thymio->setLedIntensity(Thymio2::IR_BACK_0, randColorFloat());
	thymio->setLedIntensity(Thymio2::IR_BACK_1, randColorFloat());

	thymio->setLedIntensity(Thymio2::LEFT_RED, randColorFloat());
	thymio->setLedIntensity(Thymio2::LEFT_BLUE, randColorFloat());
	thymio->setLedIntensity(Thymio2::RIGHT_BLUE, randColorFloat());
	thymio->setLedIntensity(Thymio2::RIGHT_RED, randColorFloat());

	thymio->pos = randPoint();
	return thymio;
}

EPuck* Randomizer::randEPuck()
{
	EPuck* epuck = new EPuck();
	epuck->pos = randPoint();
	return epuck;
}

Khepera* Randomizer::randKhepera()
{
	Khepera* khepera = new Khepera();
	khepera->pos = randPoint();
	return khepera;
}

Sbot* Randomizer::randSbot()
{
	Sbot* sbot = new Sbot();
	sbot->pos = randPoint();
	sbot->setColor(randColor());
	return sbot;
}

Marxbot* Randomizer::randMarxbot()
{
	Marxbot* marxbot = new Marxbot();
	marxbot->pos = randPoint();
	marxbot->setColor(randColor());
	return marxbot;
}

PhysicalObject::Hull Randomizer::randHull(const int &hullSize)
{
	PhysicalObject::Hull hull;

	int size = hullSize <= 0 ? randInt(1, 30) : hullSize;
	for(int i = 0 ; i < size ; i++)
	{
		int complex = randBool();
		if(complex)
			hull.push_back(randComplexPart());
		else
			hull.push_back(randRectanglePart());
	}
	return hull;
}

PhysicalObject::Part Randomizer::randComplexPart()
{
	Polygone p = randConvexPolygone(randInt(3, 30));
	return PhysicalObject::Part(p, randFloat(1.0, 5.0));
}

PhysicalObject::Part Randomizer::randRectanglePart()
{
	float size1 = randFloat(1.0, 30.0), size2 = randFloat(1.0, 30.0), height = randFloat(1.0, 30.0);
	return PhysicalObject::Part(size1, size2, height);
}

Polygone Randomizer::randConvexPolygone(const int &polygoneSize)
{
	int size = polygoneSize < 3 ? 3 : polygoneSize;

	int radius = randInt(1, 50);
	int center_x = 0;
	int center_y = 0;

	// This uses simple circle based trigonometry to compute a convex polygon.
	std::vector<float> points;
	for(int i = 0 ; i < size ; i++)
	{
		points.push_back(randFloat(0.0, 2*M_PI));
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

Point Randomizer::randPoint()
{
	float posx, posy;
	if(this->world->wallsType == World::WALLS_SQUARE)
	{
		posx = randFloat(0, this->world->w);
		posy = randFloat(0, this->world->h);
	}
	else if(this->world->wallsType == World::WALLS_CIRCULAR)
	{
		bool in = 0;
		while(!in)
		{
			posx = randFloat(0, this->world->r);
			posy = randFloat(0, this->world->r);
			in = (posx * posx) + (posy * posy) <= this->world->r * this->world->r ? true : false;
		}
	}
	return Point(posx, posy);
}

Color Randomizer::randColor()
{
	float r = randColorFloat();
	float g = randColorFloat();
	float b = randColorFloat();
	return Color(r, g, b);
}

float Randomizer::randFloat(const float& min, const float &max)
{
	return std::uniform_real_distribution<>(min, max)(this->randomEngine);
}

int Randomizer::randInt(const int &min, const int &max)
{
	return std::uniform_int_distribution<>(min, max)(this->randomEngine);
}

float Randomizer::randColorFloat()
{
	return this->color_distr(this->randomEngine);
}

int Randomizer::randBool()
{
	return this->bool_distr(this->randomEngine);
}
