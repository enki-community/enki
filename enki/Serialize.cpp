/*
  Enki - a fast 2D robot simulator
  Copyright © 2017 Jimmy Gouraud <jimmy.gouraud@etu.u-bordeaux.fr>
  Copyright © 2017 Mathieu Lirzin <mathieu.lirzin@etu.u-bordeaux.fr>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Serialize.h"

namespace Enki
{
	using namespace std;

	string serialize(World* world)
	{
		ostringstream oss;

		serializeWorld(world, oss);
		serializeObjects(world, oss);

		return oss.str();
	}

	void deserializeUdpate(World* world, const string& strUpdate)
	{
		size_t found = strUpdate.find_first_of(OBJECT_SEPARATOR);
		World* remoteWorld = deserialize(strUpdate);

		vector<Thymio2*> th;
		for (auto& object : remoteWorld->objects)
		{
			th.push_back(dynamic_cast<Thymio2*>(object));
		}

		int i = 0;
		for (auto& object : world->objects)
		{
			Thymio2* thymio = dynamic_cast<Thymio2*>(object);
			if (i < th.size())
			{
				thymio->pos = th.at(i)->pos;
				thymio->angle = th.at(i)->angle;
				i++;
			}
		}
	}

	World* deserialize(const string& strSerialize)
	{
		size_t pos = strSerialize.find_first_of(OBJECT_SEPARATOR);
		World* world = deserializeWorld(strSerialize.substr(0, pos));

		deserializeObjects(world, strSerialize.substr(pos + 1, strSerialize.size()).c_str());

		return world;
	}

	void serializeObjects(World* world, ostream& os)
	{
		for (auto& object : world->objects)
		{
			if (Thymio2* thymio = dynamic_cast<Thymio2*>(object))
			{
				os << THYMIO2 << TYPE_SEPARATOR;
				serializeThymio(thymio, os);
			}
			else if (PhysicalObject* po = dynamic_cast<PhysicalObject*>(object))
			{
				os << PHYSICAL_OBJECT << OBJECT_SEPARATOR;
				serializePhysObj(po, os);
			}
			else
			{
				cerr << " ===  serialize - NOT DEFINED === " << endl;
			}
		}
	}

	void serializeWorld(World* world, ostream& os)
	{
		os << world->wallsType << TYPE_SEPARATOR;

		switch (world->wallsType)
		{
		case World::WALLS_SQUARE:
			os << world->w << TYPE_SEPARATOR
			   << world->h << TYPE_SEPARATOR;
			break;
		case World::WALLS_CIRCULAR:
			os << world->r << TYPE_SEPARATOR;
			break;
		default:
			break;
		}

		serializeColor(world->color, os);

		World::GroundTexture texture = world->groundTexture;
		os << texture.width << TYPE_SEPARATOR
		   << texture.height << TYPE_SEPARATOR;
		for (int i = 0; i < texture.data.size(); i++)
		{
			os << texture.data[i] << TYPE_SEPARATOR;
		}
		os << OBJECT_SEPARATOR;
	}

	void serializeThymio(Thymio2* thymio, ostream& os)
	{
		serializePoint(thymio->pos, os);
		os << thymio->angle << TYPE_SEPARATOR;

		for (int i = 0; i < Thymio2::LED_COUNT; i++)
		{
			serializeColor(thymio->getColorLed((Thymio2::LedIndex)i), os);
		}
		os << OBJECT_SEPARATOR;
	}

	void serializePhysObj(PhysicalObject* po, ostream& os)
	{
		serializePoint(po->pos, os);
		os << po->angle << TYPE_SEPARATOR;
		//<< serialize_hull(po->getHull()) // TODO Hull hull
		serializeColor(po->getColor(), os);
		os << OBJECT_SEPARATOR;
	}

	void serializeColor(const Color &color, ostream& os)
	{
		os << color.r() << TYPE_SEPARATOR
		   << color.g() << TYPE_SEPARATOR
		   << color.b() << TYPE_SEPARATOR
		   << color.a() << TYPE_SEPARATOR
		   << MEMBER_SEPARATOR;
	}

	void serializePoint(const Point &pos, ostream& os)
	{
		os << pos.x << TYPE_SEPARATOR
		   << pos.y << TYPE_SEPARATOR;
	}

	double getValue(size_t pos1, size_t pos2, const string& strValue)
	{
		return stod(strValue.substr(pos1, pos2 - pos1));
	}

	void deserializeObjects (World* world, const string& strObjects)
	{
		size_t pos1 = strObjects.find_first_of(TYPE_SEPARATOR);
		if (pos1 == string::npos)
		{
			return;
		}

		size_t pos2 = strObjects.find_first_of(OBJECT_SEPARATOR);

		int type = atoi(strObjects.substr(0, pos2).c_str());
		switch(type)
		{
		case THYMIO2:
			world->addObject(deserializeThymio(strObjects.substr(pos1 + 1, pos2).c_str()));
			break;
		case PHYSICAL_OBJECT:
			world->addObject(deserializePhysObj(strObjects.substr(pos1 + 1, pos2).c_str()));
			break;
		default:
			cerr << " ===  Error Deserialize - objects not defined ! === " << endl;
			break;
		}

		deserializeObjects(world, strObjects.substr(pos2 + 1, strObjects.size()).c_str());
	}

	World* deserializeWorld(const string& strWorld)
	{
		size_t pos1 = 0;
		size_t pos2 = strWorld.find_first_of(TYPE_SEPARATOR);
		int wallsType = atoi(strWorld.substr(pos1, pos2).c_str());

		switch (wallsType)
		{
		case World::WALLS_SQUARE:
		{
			pos1 = pos2 + 1;
			pos2 = strWorld.find_first_of(TYPE_SEPARATOR, pos1);
			double w = getValue(pos1, pos2, strWorld);

			pos1 = pos2 + 1;
			pos2 = strWorld.find_first_of(TYPE_SEPARATOR, pos1);
			double h = getValue(pos1, pos2, strWorld);

			pos1 = pos2 + 1;
			pos2 = strWorld.find_first_of(MEMBER_SEPARATOR, pos1);
			Color color = deserializeColor(strWorld.substr(pos1, pos2 - pos1));

			// TODO: the groundTexture
			return new World(w, h, color, World::GroundTexture());
		}
		case World::WALLS_CIRCULAR:
		{
			pos1 = pos2 + 1;
			pos2 = strWorld.find_first_of(TYPE_SEPARATOR, pos1);
			double r = getValue(pos1, pos2, strWorld);

			pos1 = pos2 + 1;
			pos2 = strWorld.find_first_of(MEMBER_SEPARATOR, pos1);
			Color color = deserializeColor(strWorld.substr(pos1, pos2 - pos1));

			return new World(r, color, World::GroundTexture());
		}
		default:
			return new World();
		}
	}

	Thymio2* deserializeThymio(const string& strThymio)
	{
		Thymio2* thymio = new Thymio2();

		size_t pos1 = 0;
		size_t pos2 = strThymio.find_first_of(TYPE_SEPARATOR);
		pos2 = strThymio.find_first_of(TYPE_SEPARATOR, pos2 + 1);
		thymio->pos = deserializePoint(strThymio.substr(pos1, pos2));

		pos1 = pos2 + 1;
		pos2 = strThymio.find_first_of(TYPE_SEPARATOR, pos1);
		thymio->angle = getValue(pos1, pos2, strThymio);

		pos1 = pos2 + 1;
		pos2 = strThymio.find_first_of(MEMBER_SEPARATOR, pos1);
		Color led;
		for (int i = 0; i < Thymio2::LED_COUNT; i++)
		{
			led = deserializeColor(strThymio.substr(pos1, pos2 - pos1));
			thymio->setLedColor((Thymio2::LedIndex)i, led);

			pos1 = pos2 + 1;
			pos2 = strThymio.find_first_of(MEMBER_SEPARATOR, pos1);
		}

		return thymio;
	}

	PhysicalObject* deserializePhysObj(const string& strPo)
	{
		return new PhysicalObject();
	}

	Color deserializeColor(const string& strColor)
	{
		size_t pos1 = 0;
		size_t pos2 = strColor.find_first_of(TYPE_SEPARATOR);
		double r = getValue(pos1, pos2, strColor);

		pos1 = pos2 + 1;
		pos2 = strColor.find_first_of(TYPE_SEPARATOR, pos1);
		double g = getValue(pos1, pos2, strColor);

		pos1 = pos2 + 1;
		pos2 = strColor.find_first_of(TYPE_SEPARATOR, pos1);
		double b = getValue(pos1, pos2, strColor);

		pos1 = pos2 + 1;
		pos2 = strColor.find_first_of(TYPE_SEPARATOR, pos1);
		double a = getValue(pos1, pos2, strColor);

		return Color(r, g, b, a);
	}

	Point deserializePoint(const string& strPoint)
	{
		size_t pos1 = 0;
		size_t pos2 = strPoint.find_first_of(TYPE_SEPARATOR);
		double x = getValue(pos1, pos2, strPoint);

		pos1 = pos2 + 1;
		pos2 = strPoint.find_first_of(TYPE_SEPARATOR, pos1);
		double y = getValue(pos1, pos2, strPoint);

		return Point(x, y);
	}
}
