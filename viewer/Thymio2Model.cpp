/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "Thymio2Model.h"
#include "objects/Objects.h"

//! Asserts a dynamic cast.	Similar to the one in boost/cast.hpp
template<typename Derived, typename Base>
inline Derived polymorphic_downcast(Base base)
{
	Derived derived = dynamic_cast<Derived>(base);
	assert(derived);
	return derived;
}

namespace Enki
{
	Thymio2Model::Thymio2Model(ViewerWidget* v)
	{
		viewer = v;

		lists.resize(2);
		lists[0] = GenThymio2Body();
		lists[1] = GenThymio2Wheel();
	}

	void Thymio2Model::cleanup(ViewerWidget* viewer)
	{
		for (int i = 0; i < textures.size(); i++)
			viewer->deleteTexture(textures[i]);
		for (int i = 0; i < lists.size(); i++)
			glDeleteLists(lists[i], 1);
	}

	void Thymio2Model::draw(PhysicalObject* object) const
	{
		Thymio2* thymio = polymorphic_downcast<Thymio2*>(object);
		
		const double wheelRadius = 2.1;
		const double wheelCirc = 2 * M_PI * wheelRadius;

		// body
		glColor3d(object->getColor().components[0], object->getColor().components[1], object->getColor().components[2]);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);
		glPushMatrix();
		glTranslatef(0.3,0,0);
		glCallList(lists[0]);
		glPopMatrix();

		// wheels
		glColor3d(object->getColor().components[0], object->getColor().components[1], object->getColor().components[2]);
		glBindTexture(GL_TEXTURE_2D, 0);

		glPushMatrix();
		glTranslatef(-2.3,0,wheelRadius);
		glRotated(90.f, 0, 0, 1);
			glPushMatrix();
			glTranslatef(-4.25,0,0);
			glRotated((fmod(thymio->rightOdometry, wheelCirc) * 360) / wheelCirc, 1, 0, 0);
			glCallList(lists[1]);
			glPopMatrix();

			glPushMatrix();
			glTranslatef(4.25,0,0);
			glRotated(180.f, 0, 0, 1);
			glRotated((fmod(-thymio->leftOdometry, wheelCirc) * 360) / wheelCirc, 1, 0, 0);
			glCallList(lists[1]);
			glPopMatrix();
		glPopMatrix();
	}
} // namespace Enki


