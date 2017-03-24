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

#ifndef SERIALIZE_H
#define SERIALIZE_H

#include <enki/robots/thymio2/Thymio2.h>

/*! \file Serialize.h
    \brief The serialization & deserialization

    For the serialization:
      - complex objects ends with OBJECT_SEPARATOR = ':'
        eg: World:Thymio:PhysicalObject:...:
      - objects of a lower complexity ends with MEMBER_SEPARATOR = '&'
        eg: Color&Point&...&
      - basic types (int, float, double, etc.) ends with TYPE_SEPARATOR = ';'
      eg: int;float;double;
*/

enum ROBOT_TYPES { WORLD, THYMIO2, PHYSICAL_OBJECT };

const char OBJECT_SEPARATOR = ':';
const char MEMBER_SEPARATOR = '&';
const char TYPE_SEPARATOR = ';';

namespace Enki
{
	//! Return the string serialization of world
	std::string serialize(World *world);
	//! Return a World with its objects with the string serialization.
	World* deserialize(const std::string& strSerialization);
	//! Update the world with the string serialization.
	void deserializeUdpate(World * world, const std::string& strUpdate);

	//! Serialize the world and add the string serialization to the os
	void serializeWorld(World *world, std::ostream& os);
	//! Serialize objects of the world and add the string serialization to the os
	void serializeObjects(World *world, std::ostream& os);
	//! Serialize the Thymio and add the string serialization to the os
	void serializeThymio(Thymio2 *thymio, std::ostream& os);
	//! Serialize the PhysicalObject and add the string serialization to the os
	void serializePhysObj(PhysicalObject* po, std::ostream& os);
	//! Serialize the Color and add the string serialization to the os
	void serializeColor(const Color &color, std::ostream& os);
	//! Serialize the Point and add the string serialization to the os
	void serializePoint(const Point &pos, std::ostream& os);

	//! Return a World without objects with the string serialization.
	World* deserializeWorld(const std::string& strWorld);
	//! Complete the world with objects with the string serialization.
	void deserializeObjects (World* world, const std::string& strObjects);
	//! Return a Thymio with the string serialization.
	Thymio2* deserializeThymio(const std::string& strThymio);
	//! Return a PhysicalObject with the string serialization.
	PhysicalObject* deserializePhysObj(const std::string& strPo);
	//! Return a Color with the string serialization.
	Color deserializeColor(const std::string& strColor);
	//! Return a Point with the string serialization.
	Point deserializePoint(const std::string& strPoint);

	//! Return the value (double)
	double getValue(size_t pos1, size_t pos2, const std::string& strValue);
}

#endif // SERIALIZE_H
