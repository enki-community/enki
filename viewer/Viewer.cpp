/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2008 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://lis.epfl.ch/enki
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

#include "objects/Objects.h"
#include "Viewer.h"
#include <Viewer.moc>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/marxbot/Marxbot.h>
#include <enki/robots/alice/Alice.h>
#include <QApplication>
#include <QtGui>

/*!	\file Viewer.cpp
	\brief Implementation of the Qt-based viewer widget
*/

static void initTexturesResources()
{
	Q_INIT_RESOURCE(enki_viewer_textures);
}

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
	#define rad2deg (180 / M_PI)
	#define clamp(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))
	
	// simple display list, one per instance
	class SimpleDisplayList : public ViewerWidget::ViewerUserData
	{
	public:
		GLuint list;
	
	public:
		SimpleDisplayList()
		{
			list = glGenLists(1);
			deletedWithObject = true;
		}
		
		virtual void draw(PhysicalObject* object) const
		{
			glColor3d(object->getColor().components[0], object->getColor().components[1], object->getColor().components[2]);
			glCallList(list);
		}
		
		virtual ~SimpleDisplayList()
		{
			glDeleteLists(list, 1);
		}
	};
	
	ViewerWidget::CustomRobotModel::CustomRobotModel()
	{
		deletedWithObject = false;
	}
	
	class EPuckModel : public ViewerWidget::CustomRobotModel
	{
	public:
		EPuckModel(ViewerWidget* viewer)
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
		
		void cleanup(ViewerWidget* viewer)
		{
			for (int i = 0; i < textures.size(); i++)
				viewer->deleteTexture(textures[i]);
			for (int i = 0; i < lists.size(); i++)
				glDeleteLists(lists[i], 1);
		}
		
		virtual void draw(PhysicalObject* object) const
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
		
		virtual void drawSpecial(PhysicalObject* object, int param) const
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE);
			glDisable(GL_TEXTURE_2D);
			glCallList(lists[0]);
			glDisable(GL_BLEND);
		}
	};
	
	
	class MarxbotModel : public ViewerWidget::CustomRobotModel
	{
	public:
		MarxbotModel(ViewerWidget* viewer)
		{
			textures.resize(1);
			textures[0] = viewer->bindTexture(QPixmap(QString(":/textures/marxbot.png")), GL_TEXTURE_2D);
			lists.resize(2);
			lists[0] = GenMarxbotBase();
			lists[1] = GenMarxbotWheel();
		}
		
		void cleanup(ViewerWidget* viewer)
		{
			for (int i = 0; i < textures.size(); i++)
				viewer->deleteTexture(textures[i]);
			for (int i = 0; i < lists.size(); i++)
				glDeleteLists(lists[i], 1);
		}
		
		virtual void draw(PhysicalObject* object) const
		{
			DifferentialWheeled* dw = polymorphic_downcast<DifferentialWheeled*>(object);
			
			const double wheelRadius = 2.9;
			const double wheelCirc = 2 * M_PI * wheelRadius;
			
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, textures[0]);
			glColor3d(1, 1, 1);
			
			
			// body
			glPushMatrix();
			glCallList(lists[0]);
			glPopMatrix();
			
			// wheels
			glPushMatrix();
			glTranslatef(0,0,wheelRadius);
				glPushMatrix();
				glRotated((fmod(dw->rightOdometry, wheelCirc) * 360) / wheelCirc, 0, 1, 0);
				glCallList(lists[1]);
				glPopMatrix();
				glPushMatrix();
				glRotated(180.f, 0, 0, 1);
				glRotated((fmod(-dw->leftOdometry, wheelCirc) * 360) / wheelCirc, 0, 1, 0);
				glCallList(lists[1]);
				glPopMatrix();
			glPopMatrix();
			
			glDisable(GL_TEXTURE_2D);
			
			
			/*const double radiosityScale = 1.01;
			
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
			
			glDisable(GL_TEXTURE_2D);*/
		}
	};
	
	
	ViewerWidget::ViewerWidget(World *world, QWidget *parent) :
		QGLWidget(parent),
		world(world),
		mouseGrabbed(false),
		worldList(0),
		yaw(-M_PI/2),
		pitch((3*M_PI)/8),
		pos(-world->w * 0.5, -world->h * 0.2),
		altitude(world->h * 0.5),
		doDumpFrames(false),
		dumpFramesCounter(0)
	{
		initTexturesResources();
	}
	
	ViewerWidget::~ViewerWidget()
	{
		world->disconnectExternalObjectsUserData();
		if (isValid())
		{
			glDeleteLists(worldList, 1);
			deleteTexture (worldTexture);
			deleteTexture (wallTexture);
		}
		
		ManagedObjectsMapIterator i(managedObjects);
		while (i.hasNext())
		{
			i.next();
			ViewerUserData* data = i.value();
			data->cleanup(this);
			delete data;
		}
	}
	
	void ViewerWidget::restartDumpFrames()
	{
		dumpFramesCounter = 0;
	}
	
	void ViewerWidget::setDumpFrames(bool doDump)
	{
		doDumpFrames = doDump;
	}
	
	void ViewerWidget::renderSegment(const Segment& segment, double height)
	{
		Vector v = segment.b - segment.a;
		Vector vu = v.unitary();
		Vector n = Vector(v.y, -v.x).unitary();
		
		// draw sides
		glNormal3d(n.x, n.y, 0);
		glBegin(GL_QUADS);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d(segment.a.x, segment.a.y, 0);
		glTexCoord2f(0.99f, 0.5f);
		glVertex3d(segment.b.x, segment.b.y, 0);
		glTexCoord2f(0.99f, 0.99f);
		glVertex3d(segment.b.x, segment.b.y, height);
		glTexCoord2f(0.5f, 0.99f);
		glVertex3d(segment.a.x, segment.a.y, height);
		glEnd();
	}
	
	void ViewerWidget::renderSegmentShadow(const Segment& segment, double height)
	{
		Vector v = segment.b - segment.a;
		Vector vu = v.unitary();
		Vector n = Vector(v.y, -v.x).unitary();
		
		// draw ground
		glEnable(GL_BLEND);
		glBlendFunc(GL_ZERO, GL_SRC_COLOR);
		glDepthMask( GL_FALSE );
		glEnable(GL_POLYGON_OFFSET_FILL);
		
		Vector dvpm = Vector(vu.y, -vu.x) * height;
		
		glNormal3d(0, 0, 1);
		glBegin(GL_QUADS);
		glTexCoord2f(0.5f, 0.01f);
		glVertex3d(segment.a.x + dvpm.x, segment.a.y + dvpm.y, 0);
		glTexCoord2f(0.99f, 0.01f);
		glVertex3d(segment.b.x + dvpm.x, segment.b.y + dvpm.y, 0);
		glTexCoord2f(0.99f, 0.5f);
		glVertex3d(segment.b.x, segment.b.y, 0);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d(segment.a.x, segment.a.y, 0);
		glEnd();
		
		glDisable(GL_POLYGON_OFFSET_FILL);
		glDepthMask( GL_TRUE );
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_BLEND);
	}
	
	void ViewerWidget::renderInterSegmentShadow(const Vector& a, const Vector& b, const Vector& c, double height)
	{
		Vector prev_v = b - a;
		Vector prev_vu = prev_v.unitary();
		Vector prev_dvpm = Vector(prev_vu.y, -prev_vu.x) * height;
		
		Vector next_v = c - b;
		Vector next_vu = next_v.unitary();
		Vector next_dvpm = Vector(next_vu.y, -next_vu.x) * height;
		
		// draw ground
		glEnable(GL_BLEND);
		glBlendFunc(GL_ZERO, GL_SRC_COLOR);
		glDepthMask( GL_FALSE );
		glEnable(GL_POLYGON_OFFSET_FILL);
		
		glNormal3d(0, 0, 1);
		glBegin(GL_TRIANGLES);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d(b.x, b.y, 0);
		glTexCoord2f(0.5f, 0.01f);
		glVertex3d(b.x + prev_dvpm.x, b.y + prev_dvpm.y, 0);
		glVertex3d(b.x + next_dvpm.x, b.y + next_dvpm.y, 0);
		glEnd();
		
		glDisable(GL_POLYGON_OFFSET_FILL);
		glDepthMask( GL_TRUE );
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_BLEND);
	}
	
	void ViewerWidget::renderWorldSegment(const Segment& segment)
	{
		Vector v = segment.b - segment.a;
		Vector vu = v.unitary();
		Vector n = Vector(v.y, -v.x).unitary();
		int count = (int)(v.norm()-20) / 10;
		double l = (v.norm()-20) / (double)count;
		Vector dv = vu * l;
		Vector dvm = vu * 10;
		Vector dvpm = Vector(vu.y, -vu.x) * 10;
		Point pos = segment.a;
		
		// draw corner
		glNormal3d(n.x, n.y, 0);
		glBegin(GL_QUADS);
		glTexCoord2f(0.01f, 0.5f);
		glVertex3d(pos.x, pos.y, 0);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d((pos+dvm).x, (pos+dvm).y, 0);
		glTexCoord2f(0.5f, 0.99f);
		glVertex3d((pos+dvm).x, (pos+dvm).y, 10);
		glTexCoord2f(0.01f, 0.99f);
		glVertex3d(pos.x, pos.y, 10);
		glEnd();
		
		glNormal3d(vu.x, vu.y, 0);
		glBegin(GL_QUADS);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d(pos.x + dvpm.x, pos.y + dvpm.y, 0);
		glTexCoord2f(0.01f, 0.5f);
		glVertex3d(pos.x, pos.y, 0);
		glTexCoord2f(0.01f, 0.99f);
		glVertex3d(pos.x, pos.y, 10);
		glTexCoord2f(0.5f, 0.99f);
		glVertex3d(pos.x + dvpm.x, pos.y + dvpm.y, 10);
		glEnd();
		
		glNormal3d(0, 0, 1);
		glBegin(GL_QUADS);
		glTexCoord2f(0.01f, 0.01f);
		glVertex3d(pos.x + dvpm.x, pos.y + dvpm.y, 0);
		glTexCoord2f(0.5f, 0.01f);
		glVertex3d((pos+dvm).x + dvpm.x, (pos+dvm).y + dvpm.y, 0);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d((pos+dvm).x, (pos+dvm).y, 0);
		glTexCoord2f(0.01f, 0.5f);
		glVertex3d(pos.x, pos.y, 0);
		glEnd();
		
		pos += vu*10;
		
		for (int i = 0; i < count; i++)
		{
			// draw sides
			glNormal3d(n.x, n.y, 0);
			glBegin(GL_QUADS);
			glTexCoord2f(0.5f, 0.5f);
			glVertex3d(pos.x, pos.y, 0);
			glTexCoord2f(0.99f, 0.5f);
			glVertex3d((pos+dv).x, (pos+dv).y, 0);
			glTexCoord2f(0.99f, 0.99f);
			glVertex3d((pos+dv).x, (pos+dv).y, 10);
			glTexCoord2f(0.5f, 0.99f);
			glVertex3d(pos.x, pos.y, 10);
			glEnd();
			
			// draw ground
			glNormal3d(0, 0, 1);
			glBegin(GL_QUADS);
			glTexCoord2f(0.5f, 0.01f);
			glVertex3d(pos.x + dvpm.x, pos.y + dvpm.y, 0);
			glTexCoord2f(0.99f, 0.01f);
			glVertex3d((pos+dv).x + dvpm.x, (pos+dv).y + dvpm.y, 0);
			glTexCoord2f(0.99f, 0.5f);
			glVertex3d((pos+dv).x, (pos+dv).y, 0);
			glTexCoord2f(0.5f, 0.5f);
			glVertex3d(pos.x, pos.y, 0);
			glEnd();
			
			pos += dv;
		}
		
	}
	
	void ViewerWidget::renderWorld()
	{
		const double wallsHeight = 10;
		const double infPlanSize = 3000;
		
		glNewList(worldList, GL_COMPILE);
		
		glNormal3d(0, 0, 1);
		glColor3d(world->wallsColor.r(), world->wallsColor.g(), world->wallsColor.b());
		
		glDisable(GL_LIGHTING);
		
		switch (world->wallsType)
		{
			case World::WALLS_SQUARE:
			{
				// TODO: use world texture if any
				glBegin(GL_QUADS);
				glVertex3d(-infPlanSize, -infPlanSize, wallsHeight);
				glVertex3d(infPlanSize+world->w, -infPlanSize, wallsHeight);
				glVertex3d(infPlanSize+world->w, 0, wallsHeight);
				glVertex3d(-infPlanSize, 0, wallsHeight);
				
				glVertex3d(-infPlanSize, world->h, wallsHeight);
				glVertex3d(infPlanSize+world->w, world->h, wallsHeight);
				glVertex3d(infPlanSize+world->w, world->h+infPlanSize, wallsHeight);
				glVertex3d(-infPlanSize, world->h+infPlanSize, wallsHeight);
				
				glVertex3d(-infPlanSize, 0, wallsHeight);
				glVertex3d(0, 0, wallsHeight);
				glVertex3d(0, world->h, wallsHeight);
				glVertex3d(-infPlanSize, world->h, wallsHeight);
				
				glVertex3d(world->w, 0, wallsHeight);
				glVertex3d(world->w+infPlanSize, 0, wallsHeight);
				glVertex3d(world->w+infPlanSize, world->h, wallsHeight);
				glVertex3d(world->w, world->h, wallsHeight);
				glEnd();
				
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, worldTexture);
				glColor3d(world->wallsColor.r() / .90980392, world->wallsColor.g() / .90980392, world->wallsColor.b() / .90980392);
				
				renderWorldSegment(Segment(world->w, 0, 0, 0));
				renderWorldSegment(Segment(world->w, world->h, world->w, 0));
				renderWorldSegment(Segment(0, world->h, world->w, world->h));
				renderWorldSegment(Segment(0, 0, 0, world->h));
				
				glDisable(GL_TEXTURE_2D);
				
				glNormal3d(0, 0, 1);
				glColor3d(world->wallsColor.r(), world->wallsColor.g(), world->wallsColor.b());
				glBegin(GL_QUADS);
				glVertex3d(10, 10, 0);
				glVertex3d(world->w-10, 10, 0);
				glVertex3d(world->w-10, world->h-10, 0);
				glVertex3d(10, world->h-10, 0);
				glEnd();
				
			}
			break;
			
			case World::WALLS_CIRCULAR:
			{
				const int segmentCount = (int)((world->r*2.*M_PI) / 10.);
				for (int i = 0; i < segmentCount; ++i)
				{
					const double angStart(((double)i * 2. * M_PI) / (double)segmentCount);
					const double angEnd(((double)(i+1) * 2. * M_PI) / (double)segmentCount);
					const double angMid((angStart+angEnd)/2);
					const double innerR(world->r - 10);
					const double r(world->r);
					
					glBegin(GL_TRIANGLES);
					glVertex3d(0, 0, 0);
					glVertex3d(cos(angStart) * innerR, sin(angStart) * innerR, 0);
					glVertex3d(cos(angEnd) * innerR, sin(angEnd) * innerR, 0);
					glEnd();
					
					glEnable(GL_TEXTURE_2D);
					glBindTexture(GL_TEXTURE_2D, worldTexture);
					glColor3d(world->wallsColor.r() / .90980392, world->wallsColor.g() / .90980392, world->wallsColor.b() / .90980392);
					
					// draw sides
					glNormal3d(-cos(angMid), -sin(angMid), 0);
					glBegin(GL_QUADS);
					glTexCoord2f(0.5f, 0.5f);
					glVertex3d(cos(angEnd)*r, sin(angEnd)*r, 0);
					glTexCoord2f(0.99f, 0.5f);
					glVertex3d(cos(angStart)*r, sin(angStart)*r, 0);
					glTexCoord2f(0.99f, 0.99f);
					glVertex3d(cos(angStart)*r, sin(angStart)*r, 10);
					glTexCoord2f(0.5f, 0.99f);
					glVertex3d(cos(angEnd)*r, sin(angEnd)*r, 10);
					glEnd();
					
					// draw ground
					glNormal3d(0, 0, 1);
					glBegin(GL_QUADS);
					glTexCoord2f(0.5f, 0.01f);
					glVertex3d(cos(angEnd) * innerR, sin(angEnd) * innerR, 0);
					glTexCoord2f(0.99f, 0.01f);
					glVertex3d(cos(angStart) * innerR, sin(angStart) * innerR, 0);
					glTexCoord2f(0.99f, 0.5f);
					glVertex3d(cos(angStart) * r, sin(angStart) * r, 0);
					glTexCoord2f(0.5f, 0.5f);
					glVertex3d(cos(angEnd) * r, sin(angEnd) * r, 0);
					glEnd();
					
					glDisable(GL_TEXTURE_2D);
					
					glNormal3d(0, 0, 1);
					glColor3d(world->wallsColor.r(), world->wallsColor.g(), world->wallsColor.b());
					glBegin(GL_QUADS);
					glVertex3d(cos(angStart)*r, sin(angStart)*r, 10);
					glVertex3d(cos(angStart)*(r+infPlanSize), sin(angStart)*(r+infPlanSize), 10);
					glVertex3d(cos(angEnd)*(r+infPlanSize), sin(angEnd)*(r+infPlanSize), 10);
					glVertex3d(cos(angEnd)*r, sin(angEnd)*r, 10);
					glEnd();
				}
			}
			break;
			
			default:
			{
				glBegin(GL_QUADS);
				glVertex3d(-infPlanSize, -infPlanSize, 0);
				glVertex3d(world->w+infPlanSize, -infPlanSize, 0);
				glVertex3d(world->w+infPlanSize, world->h+infPlanSize, 0);
				glVertex3d(-infPlanSize, world->h+infPlanSize, 0);
				glEnd();
			}
			break;
		}
		
		glEnable(GL_LIGHTING);
		
		glEndList();
	}
	
	void ViewerWidget::renderSimpleObject(PhysicalObject *object)
	{
		SimpleDisplayList *userData = new SimpleDisplayList;
		object->userData = userData;
		glNewList(userData->list, GL_COMPILE);
		
		if (!object->getHull().empty())
		{
			glDisable(GL_LIGHTING);
			for (PhysicalObject::Hull::const_iterator it = object->getHull().begin(); it != object->getHull().end(); ++it)
			{
				const Polygone& shape = it->getShape();
				const double height = it->getHeight();
				const Color& color = object->getColor();
				size_t segmentCount = shape.size();
				
				// TODO: use object texture if any
				
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, wallTexture);
				
				// sides
				for (size_t i = 0; i < segmentCount; ++i)
				{
					// TODO: ugly, separate function for object and shadow
					glColor3d(color.components[0], color.components[1], color.components[2]);
					renderSegment(Segment(shape[i], shape[(i+1) % segmentCount] ), height);
					glColor3d(1, 1, 1);
					renderSegmentShadow(Segment(shape[i], shape[(i+1) % segmentCount] ), height);
					renderInterSegmentShadow(
						shape[i],
						shape[(i+1) % segmentCount],
						shape[(i+2) % segmentCount],
						height
					);
				}
				
				glDisable(GL_TEXTURE_2D);
				
				// top
				glColor3d(color.components[0], color.components[1], color.components[2]);
				glNormal3d(1, 1, 0);
				glBegin(GL_TRIANGLE_FAN);
				for (size_t i = 0; i < segmentCount; ++i)
					glVertex3d(shape[i].x, shape[i].y, height);
				glEnd();
			}
			glEnable(GL_LIGHTING);
		}
		else
		{
			GLUquadric * quadratic = gluNewQuadric();
			assert(quadratic);
			
			// sides
			gluCylinder(quadratic, object->getRadius(), object->getRadius(), object->getHeight(), 32, 1);
			
			// top
			glTranslated(0, 0, object->getHeight());
			gluDisk(quadratic, 0, object->getRadius(), 32, 1);
			
			gluDeleteQuadric(quadratic);
		}
		
		renderObjectHook(object);
		
		glEndList();
	}
	
	//! Called on GL initialisation to render application specific meshed objects, for instance application specific robots
	void ViewerWidget::renderObjectsTypesHook()
	{
	
	}
	
	//! Called inside the creation of the object display list in local object coordinate
	void ViewerWidget::renderObjectHook(PhysicalObject *object)
	{
		// dir on top of robots
		if (dynamic_cast<Robot*>(object))
		{
			glColor3d(0, 0, 0);
			glBegin(GL_TRIANGLES);
			glVertex3d(2, 0, object->getHeight() + 0.01);
			glVertex3d(-2, 1, object->getHeight() + 0.01);
			glVertex3d(-2, -1, object->getHeight() + 0.01);
			glEnd();
		}
	}
	
	//! Called when object is displayed, after the display list, with the current world matrix
	void ViewerWidget::displayObjectHook(PhysicalObject *object)
	{
	
	}
	
	//! Called when the drawing of the scene is completed.
	void ViewerWidget::sceneCompletedHook()
	{
		/*// overlay debug info
		if (mouseGrabbed)
			renderText(5, 15, QString("Mouse grabbed, yaw: %0, pitch: %1").arg(yaw).arg(pitch));
		*/
	}
	
	void ViewerWidget::initializeGL()
	{
		glClearColor(world->wallsColor.r(), world->wallsColor.g(), world->wallsColor.b(), 1.0);
		
		float LightAmbient[] = {0.6, 0.6, 0.6, 1};
		float LightDiffuse[] = {1.2, 1.2, 1.2, 1};
		float defaultColor[] = {0.5, 0.5, 0.5, 1};
		glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
		glEnable(GL_LIGHT0);
		
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, defaultColor);
		
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);
		
		glShadeModel(GL_SMOOTH);
		glEnable(GL_LIGHTING);
		glEnable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		
		glPolygonOffset(-1.01f, 0.f);
		
		/*GLfloat density = 0.001;
 		GLfloat fogColor[4] = {0.95, 0.95, 0.95, 1.0};
		glFogi (GL_FOG_MODE, GL_EXP);
		glFogfv (GL_FOG_COLOR, fogColor);
		glFogf (GL_FOG_DENSITY, density);
		glHint (GL_FOG_HINT, GL_NICEST);
		glEnable (GL_FOG);*/
		
		worldTexture = bindTexture(QPixmap(QString(":/textures/world.png")), GL_TEXTURE_2D, GL_LUMINANCE8);
		wallTexture = bindTexture(QPixmap(QString(":/textures/wall.png")), GL_TEXTURE_2D, GL_LUMINANCE8);
		worldList = glGenLists(1);
		renderWorld();
		
		// render all static types
		managedObjects[&typeid(EPuck)] = new EPuckModel(this);
		managedObjects[&typeid(Marxbot)] = new MarxbotModel(this);
		
		// let subclass manage their static types
		renderObjectsTypesHook();
		
		startTimer(30);
	}
	
	void ViewerWidget::paintGL()
	{
		// clean screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		float aspectRatio = (float)width() / (float)height();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(-1 * aspectRatio, 1 * aspectRatio, -1, 1, 2, 2000);
		
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		glRotated(-90, 1, 0, 0);
		glRotated(rad2deg * pitch, 1, 0, 0);
		glRotated(90, 0, 0, 1);
		glRotated(rad2deg * yaw, 0, 0, 1);
		
		glTranslated(pos.x(), pos.y(), -altitude);
		
		float LightPosition[] = {world->w/2, world->h/2, 60, 1};
		glLightfv(GL_LIGHT0, GL_POSITION,LightPosition);
		
		// draw world and all objects
		glCallList(worldList);
		for (World::ObjectsIterator it = world->objects.begin(); it != world->objects.end(); ++it)
		{
			// if required, render this object
			if (!(*it)->userData)
			{
				bool found = false;
				const std::type_info* typeToSearch = &typeid(**it);
				
				// search the alias map
				ManagedObjectsAliasesMapIterator aliasIt(managedObjectsAliases);
				while (aliasIt.hasNext())
				{
					aliasIt.next();
					if (*aliasIt.key() == *typeToSearch)
					{
						typeToSearch = aliasIt.value();
						break;
					}
				}
				
				// search the real map
				ManagedObjectsMapIterator dataIt(managedObjects);
				while (dataIt.hasNext())
				{
					dataIt.next();
					if (*dataIt.key() == (*typeToSearch))
					{
						(*it)->userData = dataIt.value();
						found = true;
						break;
					}
				}
				
				
				if (!found)
					renderSimpleObject(*it);
			}
			
			glPushMatrix();
			
			glTranslated((*it)->pos.x, (*it)->pos.y, 0);
			glRotated(rad2deg * (*it)->angle, 0, 0, 1);
			
			ViewerUserData* userData = polymorphic_downcast<ViewerUserData *>((*it)->userData);
			userData->draw(*it);
			displayObjectHook(*it);
			
			glPopMatrix();
		}
		
		sceneCompletedHook();
		
		if (doDumpFrames)
			grabFrameBuffer().save(QString("enkiviewer-frame%1.png").arg((int)dumpFramesCounter++, (int)8, (int)10, QChar('0')));
	}
	
	void ViewerWidget::resizeGL(int width, int height)
	{
		glViewport(0, 0, width, height);
	}
	
	void ViewerWidget::timerEvent(QTimerEvent * event)
	{
		world->step(1./30., 3);
		updateGL();
	}
	
	void ViewerWidget::mousePressEvent(QMouseEvent *event)
	{
		mouseGrabPos = event->pos();
		/*if (event->button() == Qt::RightButton)
		{
			mouseGrabbed = true;
			mouseGrabPos = event->pos();
		}*/
	}
	
	void ViewerWidget::mouseReleaseEvent(QMouseEvent * event)
	{
		/*if (event->button() == Qt::RightButton)
			mouseGrabbed = false;*/
	}
	
	void ViewerWidget::mouseMoveEvent(QMouseEvent *event)
	{
		if (event->modifiers() & Qt::ControlModifier)
		{
			QPoint diff = event->pos() - mouseGrabPos;
			if (event->buttons() & Qt::LeftButton)
			{
				if (event->modifiers() & Qt::ShiftModifier)
				{
					pos.rx() += 0.5 * cos(yaw) * (double)diff.y() + 0.5 * sin(yaw) * (double)diff.x();
					pos.ry() += 0.5 * sin(yaw) * -(double)diff.y() + 0.5 * cos(yaw) * (double)diff.x();
				}
				else
				{
					yaw += 0.01 * (double)diff.x();
					pitch = clamp(pitch + 0.01 * (double)diff.y(), -M_PI / 2, M_PI / 2);
				}
			}
			else if (event->buttons() & Qt::RightButton)
			{
				if (event->modifiers() & Qt::ShiftModifier)
				{
					altitude += -(double)diff.y();
				}
				else
				{
					// TODO: zoom
				}
			}
			
			mouseGrabPos = event->pos();
		}
		
		/*if (mouseGrabbed)
		{
			QPoint diff = event->pos() - mouseGrabPos;
			
			if (event->modifiers() & Qt::ShiftModifier)
			{
				if (event->modifiers() & Qt::ControlModifier)
				{
					altitude += -(double)diff.y();
				}
				else
				{
					pos.rx() += 0.5 * cos(yaw) * (double)diff.y() + 0.5 * sin(yaw) * (double)diff.x();
					pos.ry() += 0.5 * sin(yaw) * -(double)diff.y() + 0.5 * cos(yaw) * (double)diff.x();
				}
			}
			else
			{
				yaw += 0.01 * (double)diff.x();
				pitch = clamp(pitch + 0.01 * (double)diff.y(), -M_PI / 2, M_PI / 2);
			}
			mouseGrabPos = event->pos();
		}*/
	}
	
	void ViewerWidget::wheelEvent(QWheelEvent * event)
	{
		/*if (event->modifiers() & Qt::ShiftModifier)
		{
			altitude += (double)event->delta() / 100;
		}*/
		// TODO: zoom
	}
}
