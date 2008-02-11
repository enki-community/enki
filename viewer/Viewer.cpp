/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2006 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006 Laboratory of Robotics Systems, EPFL, Lausanne
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

#include "Viewer.h"
#include "Viewer.moc"
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/alice/Alice.h>
#include <QApplication>
#include <QtGui>

/*!	\file Viewer.cpp
	\brief Implementation of the Qt-based viewer widget
*/

namespace Enki
{
	#define rad2deg (180 / M_PI)
	#define clamp(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))
	
	ViewerWidget::ViewerWidget(World *world, QWidget *parent) :
		QGLWidget(parent),
		world(world),
		mouseGrabbed(false),
		worldList(0),
		yaw(-M_PI/2),
		pitch((3*M_PI)/8),
		pos(-world->w * 0.5, -world->h * 0.2),
		altitude(world->h * 0.5)
	{
		
	}
	
	ViewerWidget::~ViewerWidget()
	{
		if (isValid())
			glDeleteLists(worldList, 1);
	}
	
	void ViewerWidget::renderSegment(const Segment& segment, double height)
	{
		Vector v = segment.b - segment.a;
		Vector n = Vector(v.y, -v.x).unitary();
		glNormal3d(n.x, n.y, 0);
		
		glBegin(GL_QUADS);
		glVertex3d(segment.a.x, segment.a.y, 0);
		glVertex3d(segment.b.x, segment.b.y, 0);
		glVertex3d(segment.b.x, segment.b.y, height);
		glVertex3d(segment.a.x, segment.a.y, height);
		glEnd();
	}
	
	void ViewerWidget::renderWorld()
	{
		glNewList(worldList, GL_COMPILE);
		
		glColor3d(0.6, 0.6, 0.6);
		
		glNormal3d(0, 0, 1);
		
		glBegin(GL_QUADS);
		glVertex3d(0, 0, 0);
		glVertex3d(world->w, 0, 0);
		glVertex3d(world->w, world->h, 0);
		glVertex3d(0, world->h, 0);
		glEnd();
		
		// TODO: use world texture if any
		if (world->useWalls)
		{
			const double wallsHeight = 10;
			
			renderSegment(Segment(world->w, 0, 0, 0), wallsHeight);
			renderSegment(Segment(world->w, world->h, world->w, 0), wallsHeight);
			renderSegment(Segment(0, world->h, world->w, world->h), wallsHeight);
			renderSegment(Segment(0, 0, 0, world->h), wallsHeight);
		}
		glEndList();
	}
	
	void ViewerWidget::renderObject(PhysicalObject *object)
	{
		DisplayListUserData *userData = new DisplayListUserData;
		object->userData = userData;
		glNewList(userData->list, GL_COMPILE);
		
		if (object->boundingSurface)
		{
			// TODO: use object texture if any
			size_t segmentCount = object->boundingSurface->size();
			
			// sides
			for (size_t i = 0; i < segmentCount; ++i)
				renderSegment(Segment((*object->boundingSurface)[i], (*object->boundingSurface)[(i+1) % segmentCount] ), object->height);
			
			// top
			glNormal3d(1, 1, 0);
			glBegin(GL_TRIANGLE_FAN);
			for (size_t i = 0; i < segmentCount; ++i)
				glVertex3d((*object->boundingSurface)[i].x, (*object->boundingSurface)[i].y, object->height);
			glEnd();
		}
		else
		{
			GLUquadric * quadratic = gluNewQuadric();
			assert(quadratic);
			
			// sides
			gluCylinder(quadratic, object->r, object->r, object->height, 32, 1);
			
			// top
			glTranslated(0,0,object->height);
			gluDisk(quadratic, 0, object->r, 32, 1);
			
			gluDeleteQuadric(quadratic);
		}
		
		renderObjectHook(object);
		
		glEndList();
	}
	
	//! Called inside the creation of the object display list in local object coordinate
	void ViewerWidget::renderObjectHook(PhysicalObject *object)
	{
		// dir on top of robots
		if (dynamic_cast<Robot*>(object))
		{
			glColor3d(0, 0, 0);
			glBegin(GL_TRIANGLES);
			glVertex3d(2, 0, object->height+0.01);
			glVertex3d(-2, 1, object->height+0.01);
			glVertex3d(-2, -1, object->height+0.01);
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
		glClearColor(0.6, 0.7, 1.0, 0.0);
		
		float LightAmbient[] = {0.5, 0.5, 0.5, 1};
		float LightDiffuse[] = {1, 1, 1, 1};
		glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
		glEnable(GL_LIGHT0);
		
		glShadeModel(GL_SMOOTH);
		glEnable(GL_LIGHTING);
		glDisable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_COLOR_MATERIAL);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
		worldList = glGenLists(1);
		renderWorld();
		
		startTimer(30);
	}
	
	void ViewerWidget::paintGL()
	{
		// clean screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
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
				renderObject(*it);
			
			glPushMatrix();
			
			glTranslated((*it)->pos.x, (*it)->pos.y, 0);
			glRotated(rad2deg * (*it)->angle, 0, 0, 1);
			glColor3d((*it)->color.components[0], (*it)->color.components[1], (*it)->color.components[2]);
			
			DisplayListUserData *userData = dynamic_cast<DisplayListUserData *>((*it)->userData);
			assert(userData);
			glCallList(userData->list);
			displayObjectHook(*it);
			
			glPopMatrix();
		}
		
		sceneCompletedHook();
	}
	
	void ViewerWidget::resizeGL(int width, int height)
	{
		float aspectRatio = (float)width / (float)height;
		
		// setup viewport, projection etc.:
		glViewport(0, 0, width, height);
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(-4 * aspectRatio, 4 * aspectRatio, -4, 4, 4, 500);
		
		glMatrixMode(GL_MODELVIEW);
	}
	
	void ViewerWidget::timerEvent(QTimerEvent * event)
	{
		world->step(1./30.);
		updateGL();
	}
	
	void ViewerWidget::mousePressEvent(QMouseEvent *event)
	{
		if (event->button() == Qt::RightButton)
		{
			mouseGrabbed = true;
			mouseGrabPos = event->pos();
		}
	}
	
	void ViewerWidget::mouseReleaseEvent(QMouseEvent * event)
	{
		if (event->button() == Qt::RightButton)
			mouseGrabbed = false;
	}
	
	void ViewerWidget::mouseMoveEvent(QMouseEvent *event)
	{
		if (mouseGrabbed)
		{
			QPoint diff = event->pos() - mouseGrabPos;
			
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
			mouseGrabPos = event->pos();
		}
	}
	
	void ViewerWidget::wheelEvent(QWheelEvent * event)
	{
		if (event->modifiers() & Qt::ShiftModifier)
		{
			altitude += (double)event->delta() / 100;
		}
	}
}
