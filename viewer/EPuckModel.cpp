/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2013 Stephane Magnenat <stephane at magnenat dot net>
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

#include "EPuckModel.h"
#include "objects/Objects.h"
#include <enki/robots/marxbot/Marxbot.h>
#include <enki/robots/e-puck/EPuck.h>

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
	EPuckModel::EPuckModel(ViewerWidget* viewer)
	{
		textures.resize(2);
		textures[0] = viewer->bindTexture(QPixmap(QString(":/textures/epuck.png")), GL_TEXTURE_2D);
		textures[1] = viewer->bindTexture(QPixmap(QString(":/textures/epuckr.png")), GL_TEXTURE_2D, GL_LUMINANCE8);
		lists.resize(5);
		lists[0] = GenEPuckBody();
		lists[1] = GenEPuckRest();
		lists[2] = GenEPuckRing();
		lists[3] = GenEPuckWheelLeft();
		lists[4] = GenEPuckWheelRight();
	}
	
	void EPuckModel::cleanup(ViewerWidget* viewer)
	{
		for (int i = 0; i < textures.size(); i++)
			viewer->deleteTexture(textures[i]);
		for (int i = 0; i < lists.size(); i++)
			glDeleteLists(lists[i], 1);
	}
	
	void EPuckModel::draw(PhysicalObject* object) const
	{
		DifferentialWheeled* dw = polymorphic_downcast<DifferentialWheeled*>(object);
		
		const double wheelRadius = 2.1;
		const double wheelCirc = 2 * M_PI * wheelRadius;
		const double radiosityScale = 1.01;
		
		glTranslated(0, 0, wheelRadius);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		
		glColor3d(1, 1, 1);
		
		glCallList(lists[0]);
		
		glCallList(lists[1]);
		
		//glColor3d(1-object->getColor().components[0], 1+object->getColor().components[1], 1+object->getColor().components[2]);
		glColor3d(0.6+object->getColor().components[0]-0.3*object->getColor().components[1]-0.3*object->getColor().components[2], 0.6+object->getColor().components[1]-0.3*object->getColor().components[0]-0.3*object->getColor().components[2], 0.6+object->getColor().components[2]-0.3*object->getColor().components[0]-0.3*object->getColor().components[1]);
		glCallList(lists[2]);
		
		glColor3d(1, 1, 1);
		
		// wheels
		glPushMatrix();
		glRotated((fmod(dw->leftOdometry, wheelCirc) * 360) / wheelCirc, 0, 1, 0);
		glCallList(lists[3]);
		glPopMatrix();
		
		glPushMatrix();
		glRotated((fmod(dw->rightOdometry, wheelCirc) * 360) / wheelCirc, 0, 1, 0);
		glCallList(lists[4]);
		glPopMatrix();
		
		// shadow
		glBindTexture(GL_TEXTURE_2D, textures[1]);
		glDisable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_ZERO, GL_SRC_COLOR);
		
		// bottom shadow
		glPushMatrix();
		// disable writing of z-buffer
		glDepthMask( GL_FALSE );
		//glTranslated(0, 0, -wheelRadius+0.01);
		glTranslated(0, 0, -wheelRadius);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glBegin(GL_QUADS);
		glTexCoord2f(0.49f, 0.01f);
		glVertex2f(-5.f, -5.f);
		glTexCoord2f(0.49f, 0.49f);
		glVertex2f(5.f, -5.f);
		glTexCoord2f(0.01f, 0.49f);
		glVertex2f(5.f, 5.f);
		glTexCoord2f(0.01f, 0.01f);
		glVertex2f(-5.f, 5.f);
		glEnd();
		glDisable(GL_POLYGON_OFFSET_FILL);
		glDepthMask( GL_TRUE );
		glPopMatrix();
		
		// wheel shadow
		glPushMatrix();
		glScaled(radiosityScale, radiosityScale, radiosityScale);
		glTranslated(0, -0.025, 0);
		glCallList(lists[3]);
		glPopMatrix();
		
		glPushMatrix();
		glScaled(radiosityScale, radiosityScale, radiosityScale);
		glTranslated(0, 0.025, 0);
		glCallList(lists[4]);
		glPopMatrix();
		
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_BLEND);
		glEnable(GL_LIGHTING);
		
		glDisable(GL_TEXTURE_2D);
	}
	
	void EPuckModel::drawSpecial(PhysicalObject* object, int param) const
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_ONE, GL_ONE);
		glDisable(GL_TEXTURE_2D);
		glCallList(lists[0]);
		glDisable(GL_BLEND);
	}
} // namespace Enki