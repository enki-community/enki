/*
   Enki - a fast 2D robot simulator
   Copyright © 2017 Nicolas Palard <nicolas.palard@etu.u-bordeaux.fr>
   Copyright © 2017 Mathieu Lirzin <mathieu.lirzin@etu.u-bordeaux.fr>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Randomizer.h"

using namespace Enki;

Randomizer::Randomizer(World* world, long long int seed)
{

	this->seed = seed == -1 ?
		std::chrono::system_clock::now().time_since_epoch().count() :
		seed;
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
	this->seed = seed == -1 ?
		std::chrono::system_clock::now().time_since_epoch().count() :
		seed;
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
	int wallsType = randInt(0, 1);

	Color color = randColor();

	// Not working
	//World::GroundTexture = randGroundTexture();

	if (wallsType == World::WALLS_SQUARE)
	{
		int width = randInt(MIN_WIDTH, MAX_WIDTH);
		int height = randInt(MIN_HEIGHT, MAX_HEIGHT);
		return new World(width, height, color);
	}
	else if (wallsType == World::WALLS_CIRCULAR)
	{
		int radius = randInt(MIN_RADIUS, MAX_RADIUS);
		return new World(radius, color);
	}
}

PhysicalObject* Randomizer::randObject()
{
	int which = randBool();
	return which ?
		randRobot() :
		randPhysicalObject();
}

PhysicalObject* Randomizer::randPhysicalObject(const int &hullSize)
{
	PhysicalObject* object = new PhysicalObject();
	object->pos = randPoint();
	object->setColor(randColor());

	// Disable customHull due to multiple bugs.
	int cylindric = 1;//randBool();
	if (cylindric)
	{
		object->setCylindric(randFloat(1.0, 5.0), randFloat(1.0, 5.0), randFloat(1.0, 5.0));
	}
	else
	{
		// This generates a bug that moves the object out of the world
		// when a complex part is computed with a big height.
		object->setCustomHull(randHull(hullSize), randInt(1, 50));
	}

	return object;
}

Robot* Randomizer::randRobot(int type)
{
	Robot* r;
	type = type == -1 ?
		randInt(0, NUMBER_OF_ROBOTS_TYPES - 1) :
		type;
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

PhysicalObject::Hull Randomizer::randHull(int hullSize)
{
	PhysicalObject::Hull hull;

	hullSize = hullSize <= 0 ?
		randInt(1, 30) :
		hullSize;
	for (int i = 0 ; i < hullSize ; i++)
	{
		int complex = randBool();
		if (complex)
			hull.push_back(randComplexPart());
		else
			hull.push_back(randRectanglePart());
	}

	return hull;
}

PhysicalObject::Part Randomizer::randComplexPart()
{
	Polygone p = randConvexPolygone(randInt(3, 30));
	bool textured = randBool();
	if (textured)
	{
		Textures t = randTextures(p.size());
		return PhysicalObject::Part(p, randFloat(1.0, 5.0), t);
	}

	return PhysicalObject::Part(p, randFloat(1.0, 5.0));
}

PhysicalObject::Part Randomizer::randRectanglePart()
{
	float size1 = randFloat(1.0, 30.0), size2 = randFloat(1.0, 30.0), height = randFloat(1.0, 30.0);
	return PhysicalObject::Part(size1, size2, height);
}

Polygone Randomizer::randConvexPolygone(int polygoneSize)
{
	polygoneSize = polygoneSize < 3 ?
		3 :
		polygoneSize;

	// This uses simple circle based trigonometry to compute a convex polygon.
	std::vector<float> points;
	for (int i = 0 ; i < polygoneSize ; i++)
	{
		points.push_back(randFloat(0.0, 2*M_PI));
	}

	std::sort(points.begin(), points.end());

	Polygone p;
	int center_x = 0, center_y = 0;
	int radius = randInt(1, 50);
	for (int i = 0 ; i < points.size() ; i++)
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
	if (this->world->wallsType == World::WALLS_SQUARE)
	{
		posx = randFloat(0, this->world->w);
		posy = randFloat(0, this->world->h);
	}
	else if (this->world->wallsType == World::WALLS_CIRCULAR)
	{
		bool in = 0;
		while (!in)
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

Texture Randomizer::randTexture(int nbColor)
{
	nbColor = nbColor <= 0 ?
		randInt(1, 5) :
		nbColor;
	Texture t;
	for (int i = 0 ; i < nbColor ; i++)
	{
		t.push_back(randColor());
	}

	return t;
}

Textures Randomizer::randTextures(int nbTexture)
{
	nbTexture = nbTexture <= 0 ?
		randInt(1, 5) :
		nbTexture;
	Textures t;
	for (int i = 0 ; i < nbTexture ; i++)
	{
		t.push_back(randTexture());
	}

	return t;
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
