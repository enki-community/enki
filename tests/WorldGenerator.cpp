/*
   Enki - a fast 2D robot simulator
   Copyright © 2017 Nicolas Palard <nicolas.palard@etu.u-bordeaux.fr>

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

#include "WorldGenerator.h"

namespace Enki
{

	WorldGenerator::WorldGenerator()
	{
		this->randomizer = new Randomizer();
	}

	WorldGenerator::WorldGenerator(const int &width, const int &height)
	{
		World* w = new World(width, height);
		this->randomizer = new Randomizer(w);
	}

	WorldGenerator::WorldGenerator(const int &radius)
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

	bool WorldGenerator::add(const int &type, const int &number)
	{
		int cpt(0);
		int num = number ? number : this->randomizer->randInt(1, 30);
		while(cpt < num)
		{
			PhysicalObject* o;
			/*
			type can have up to 7 values :
				0 -> 4 : Only robots (of a specified type) (enum)
				5 : Physical Object
				6 : Any kind of robots
				7 : Anything (PO / R)
			*/
			switch (type) {
				case Randomizer::THYMIO2_ :
				case Randomizer::EPUCK_ :
				case Randomizer::SBOT_ :
				case Randomizer::MARXBOT_ :
				case Randomizer::KHEPERA_ :
					o = this->randomizer->randRobot(type);
					break;
				case PHYSICAL_OBJECTS_ :
					o = this->randomizer->randPhysicalObject();
					break;
				case ONLY_ROBOTS_ :
					o = this->randomizer->randRobot();
					break;
				case ANYTHING_ :
					o = this->randomizer->randObject();
					break;
				default:
					o = this->randomizer->randObject();
			}
			if (WorldGenerator::add(o))
				cpt++;
		}
		return cpt == number - 1;
	}

	bool WorldGenerator::add(std::vector<PhysicalObject*> vec)
	{
		int cpt(0);
		for (auto object : vec)
		{
			if (WorldGenerator::add(object))
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

	Randomizer* WorldGenerator::getRandomizer()
	{
		return this->randomizer;
	}
}
