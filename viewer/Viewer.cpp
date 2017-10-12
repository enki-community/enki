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

#include "objects/Objects.h"
#include "Viewer.h"
#include "EPuckModel.h"
#include <enki/robots/e-puck/EPuck.h>
#include "MarxbotModel.h"
#include <enki/robots/marxbot/Marxbot.h>
#include "Thymio2Model.h"
#include <enki/robots/thymio2/Thymio2.h>

#ifdef Q_OS_WIN
	#ifndef GL_BGRA
		// Windows only ships with OpenGL 1.1, while GL_BGRA is defined in version 1.2
		#define GL_BGRA GL_BGRA_EXT
	#endif // GL_BGRA
#endif // Q_OS_WIN
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
	
	//! Create a camera at 0
	ViewerWidget::CameraPose::CameraPose():
		altitude(0),
		yaw(0),
		pitch(0)
	{
	}
	
	//! Create a camera centered on a given world
	ViewerWidget::CameraPose::CameraPose(const World *world):
		pos(QPointF(world->w * 0.5, -qMax(0., world->r * 0.9))),
		altitude(qMax(qMax(world->w, world->h), world->r*2) * 0.9),
		yaw(M_PI/2),
		pitch(-(3*M_PI)/8)
	{
	}
	
	//! Create a camera at a given pos
	ViewerWidget::CameraPose::CameraPose(const QPointF& pos, double altitude, double yaw, double pitch):
		pos(pos),
		altitude(altitude),
		yaw(yaw),
		pitch(pitch)
	{
	}
	
	ViewerWidget::UpdatableCameraPose::UpdatableCameraPose():
		userYaw(0),
		radius(20)
	{
		update();
	}
	
	ViewerWidget::UpdatableCameraPose::UpdatableCameraPose(const World *world):
		CameraPose(world),
		userYaw(M_PI/2),
		radius(20)
	{
		update();
	}
	
	ViewerWidget::UpdatableCameraPose::UpdatableCameraPose(const QPointF& pos, double altitude, double yaw, double pitch):
		CameraPose(pos, altitude, yaw, pitch),
		userYaw(yaw),
		radius(20)
	{
		update();
	}
	
	ViewerWidget::UpdatableCameraPose& ViewerWidget::UpdatableCameraPose::operator=(const CameraPose& pose)
	{
		CameraPose::operator=(pose);
		userYaw = yaw;
		return *this;
	}
	
	//! Update base coordinate system from pose
	void ViewerWidget::UpdatableCameraPose::update()
	{
		yaw = userYaw;
		forward = QVector3D( cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch) );
		left = QVector3D::crossProduct(QVector3D(0,0,1), forward).normalized();
		up = QVector3D::crossProduct(forward, left).normalized();
	}
	
	//! Update base coordinate system from target and current orientation, override pose
	void ViewerWidget::UpdatableCameraPose::updateTracking(double targetAngle, const QVector3D& targetPosition, double zNear)
	{
		yaw = userYaw + targetAngle;
		forward = QVector3D( cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch) );
		left = QVector3D::crossProduct(QVector3D(0,0,1), forward).normalized();
		up = QVector3D::crossProduct(forward, left).normalized();
	
		pos.rx() = targetPosition.x() - radius*forward.x();
		pos.ry() = targetPosition.y() - radius*forward.y();
		altitude = targetPosition.z() + zNear*1.01 - radius*forward.z();
	}
	
	ViewerWidget::InfoMessage::InfoMessage(const QString& message, double persistance, const QColor& color, const QUrl& link):
		message(message),
		persistance(persistance),
		color(color),
		link(link)
	{}

	ViewerWidget::ViewerWidget(World *world, QWidget *parent) :
		QGLWidget(parent),
		timerPeriodMs(30),
		camera(world),
		doDumpFrames(false),
		dumpFramesCounter(0),
		world(world),
		worldList(0),
		messageListWidth(0),
		messageListHeight(0),
		fontMetrics(QFont()),
		mouseGrabbed(false),
		wallsHeight(10),
		trackingView(false),
		pointedObject(0),
		selectedObject(0),
		movingObject(false),
		mouseLeftButtonRobot(0),
		mouseRightButtonRobot(0),
		mouseMiddleButtonRobot(0)
	{
		initTexturesResources();
		elapsedTime = double(30)/1000.; // average second between two frames, can be updated each frame to better precision
		
		startTimer(timerPeriodMs);
	}
	
	ViewerWidget::~ViewerWidget()
	{
		world->disconnectExternalObjectsUserData();
		if (isValid())
		{
			deleteTexture(helpWidget);
			deleteTexture(centerWidget);
			deleteTexture(selectionTexture);
			glDeleteLists(worldList, 1);
			deleteTexture (worldTexture);
			deleteTexture (wallTexture);
			if (world->hasGroundTexture())
				glDeleteTextures(1, &worldGroundTexture);
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
	
	World* ViewerWidget::getWorld() const
	{
		return world;
	}
	
	ViewerWidget::CameraPose ViewerWidget::getCamera() const
	{
		return camera;
	}

	QVector3D ViewerWidget::getPointedPoint() const
	{
		return pointedPoint;
	}

	PhysicalObject* ViewerWidget::getPointedObject() const
	{
		return pointedObject;
	}

	PhysicalObject* ViewerWidget::getSelectedObject() const
	{
		return selectedObject;
	}

	bool ViewerWidget::isTrackingActivated() const
	{
		return trackingView;
	}

	bool ViewerWidget::isMovableByPicking(PhysicalObject* object) const
	{
		if (!object)
			return 0;
		std::map<PhysicalObject*, ExtendedAttributes>::const_iterator it = objectExtendedAttributesList.find(object);
		if (it != objectExtendedAttributesList.end())
			return it->second.movableByPicking;
		else
			return object->getMass() >= 0;
	}

	/*!
		\brief Specify if an object is movable by picking.
		By default all object are not movable, so it's prefered to only specify object wich are mouvable by picking.
		This function add an extended attribute to an object using an association table, so it's to the user responsibility to
		delete the extended parameter if object is remove to the world, using the "removeExtendedAttributes" function
	*/
	void ViewerWidget::setMovableByPicking(PhysicalObject* object, bool movable)
	{
		objectExtendedAttributesList[object].movableByPicking = movable;
	}

	void ViewerWidget::setCamera(const QPointF& pos, double altitude, double yaw, double pitch)
	{
		camera.pos = pos;
		camera.altitude = altitude;
		camera.yaw = yaw;
		camera.userYaw = yaw;
		camera.pitch = pitch;
	}
	
	void ViewerWidget::setCamera(double x, double y, double altitude, double yaw, double pitch)
	{
		setCamera(QPointF(x,y), altitude, yaw, pitch);
	}
	
	void ViewerWidget::restartDumpFrames()
	{
		dumpFramesCounter = 0;
	}
	
	void ViewerWidget::setDumpFrames(bool doDump)
	{
		doDumpFrames = doDump;
	}
	
	void ViewerWidget::setTracking(bool doTrack)
	{
		const bool willTrack(doTrack && selectedObject);
		if (!trackingView && willTrack)
		{
			nonTrackingCamera = camera;
			camera.userYaw = 0;
			camera.radius = selectedObject->getRadius() * 4;
		}
		else if (trackingView && !willTrack)
		{
			camera = nonTrackingCamera;
		}
		trackingView = willTrack;
	}
	
	void ViewerWidget::toggleTracking()
	{
		setTracking(!trackingView);
	}

	void ViewerWidget::addInfoMessage(const QString& message, double persistance, const QColor& color, const QUrl& link)
	{
		// add or update message in the list
		for (MessageList::iterator it = messageList.begin(); it!=messageList.end(); it++)
		{
			if (it->message == message)
			{
				it->persistance = persistance;
				return;
			}
		}
		messageList.push_back(InfoMessage(message, persistance, color, link));
		
		// compute the size of the list
		computeInfoMessageAreaSize();
	}

	void ViewerWidget::showHelp()
	{
		addInfoMessage(trUtf8("Available controls:"));
		addInfoMessage(trUtf8("• F1 key: show this help message"));
		addInfoMessage(trUtf8("• left click on object: select object"));
		addInfoMessage(trUtf8("• left click outside object: de-select object"));
		addInfoMessage(trUtf8("• left drag: if object selected, move it, otherwise move camera"));
		addInfoMessage(trUtf8("• right drag: if object selected, rotate it, otherwise rotate camera"));
		addInfoMessage(trUtf8("• mouse wheel/left drag + shift: zoom camera"));
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
		int count = ((int)(v.norm()-20) / 10+1);
		double l = (v.norm()-20) / (double)count;
		Vector dv = vu * l;
		Vector dvm = vu * 10;
		Vector dvpm = Vector(vu.y, -vu.x) * 10;
		
		// vertical part
		Point pos = segment.a;
		
		glColor3d(world->color.r(), world->color.g(), world->color.b());
		
		// draw corner
		glNormal3d(n.x, n.y, 0);
		glBegin(GL_QUADS);
		glTexCoord2f(0.01f, 0.5f);
		glVertex3d(pos.x, pos.y, 0);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d((pos+dvm).x, (pos+dvm).y, 0);
		glTexCoord2f(0.5f, 0.99f);
		glVertex3d((pos+dvm).x, (pos+dvm).y, wallsHeight);
		glTexCoord2f(0.01f, 0.99f);
		glVertex3d(pos.x, pos.y, wallsHeight);
		glEnd();
		
		glNormal3d(vu.x, vu.y, 0);
		glBegin(GL_QUADS);
		glTexCoord2f(0.5f, 0.5f);
		glVertex3d(pos.x + dvpm.x, pos.y + dvpm.y, 0);
		glTexCoord2f(0.01f, 0.5f);
		glVertex3d(pos.x, pos.y, 0);
		glTexCoord2f(0.01f, 0.99f);
		glVertex3d(pos.x, pos.y, wallsHeight);
		glTexCoord2f(0.5f, 0.99f);
		glVertex3d(pos.x + dvpm.x, pos.y + dvpm.y, wallsHeight);
		glEnd();
		
		pos += vu*10;
		
		// draw sides
		for (int i = 0; i < count; i++)
		{
			glNormal3d(n.x, n.y, 0);
			glBegin(GL_QUADS);
			glTexCoord2f(0.5f, 0.5f);
			glVertex3d(pos.x, pos.y, 0);
			glTexCoord2f(0.99f, 0.5f);
			glVertex3d((pos+dv).x, (pos+dv).y, 0);
			glTexCoord2f(0.99f, 0.99f);
			glVertex3d((pos+dv).x, (pos+dv).y, wallsHeight);
			glTexCoord2f(0.5f, 0.99f);
			glVertex3d(pos.x, pos.y, wallsHeight);
			glEnd();
			
			pos += dv;
		}
		
		// shadow part
		pos = segment.a;
		
		glColor3d(1, 1, 1);
		glEnable(GL_BLEND);
		glBlendFunc(GL_ZERO, GL_SRC_COLOR);
		glDepthMask( GL_FALSE );
		glEnable(GL_POLYGON_OFFSET_FILL);
		
		// draw corner ground
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
		
		// draw side ground
		for (int i = 0; i < count; i++)
		{
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
		
		glDisable(GL_POLYGON_OFFSET_FILL);
		glDepthMask( GL_TRUE );
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_BLEND);
	}
	
	void ViewerWidget::renderWorld()
	{
		const double infPlanSize = 3000;
		
		glNewList(worldList, GL_COMPILE);
		
		glNormal3d(0, 0, 1);
		glColor3d(world->color.r(), world->color.g(), world->color.b());
		
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
				
				if (world->hasGroundTexture())
				{
					glEnable(GL_TEXTURE_2D);
					glBindTexture(GL_TEXTURE_2D, worldGroundTexture);
				}
				
				glNormal3d(0, 0, 1);
				glColor3d(world->color.r(), world->color.g(), world->color.b());
				glBegin(GL_QUADS);
				glTexCoord2f(0.0f, 0.0f);
				glVertex3d(0, 0, 0);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3d(world->w, 0, 0);
				glTexCoord2f(1.0f, 1.0f);
				glVertex3d(world->w, world->h, 0);
				glTexCoord2f(0.0f, 1.0f);
				glVertex3d(0, world->h, 0);
				glEnd();
				
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, worldTexture);
				
				renderWorldSegment(Segment(world->w, 0, 0, 0));
				renderWorldSegment(Segment(world->w, world->h, world->w, 0));
				renderWorldSegment(Segment(0, world->h, world->w, world->h));
				renderWorldSegment(Segment(0, 0, 0, world->h));
				
				glDisable(GL_TEXTURE_2D);
			}
			break;
			
			case World::WALLS_CIRCULAR:
			{
				const double r(world->r);
				const int segmentCount = std::max(24, int((r*2.*M_PI) / 10.));
				for (int i = 0; i < segmentCount; ++i)
				{
					const double angStart(((double)i * 2. * M_PI) / (double)segmentCount);
					const double angEnd(((double)(i+1) * 2. * M_PI) / (double)segmentCount);
					const double angMid((angStart+angEnd)/2);
					const double innerR(r - 10);
					
					glDisable(GL_TEXTURE_2D);
					glNormal3d(0, 0, 1);
					glColor3d(world->color.r(), world->color.g(), world->color.b());
					
					// draw to infinity
					glBegin(GL_QUADS);
					glVertex3d(cos(angStart)*r, sin(angStart)*r, 10);
					glVertex3d(cos(angStart)*(r+infPlanSize), sin(angStart)*(r+infPlanSize), 10);
					glVertex3d(cos(angEnd)*(r+infPlanSize), sin(angEnd)*(r+infPlanSize), 10);
					glVertex3d(cos(angEnd)*r, sin(angEnd)*r, 10);
					glEnd();
					
					// draw ground center
					if (world->hasGroundTexture())
					{
						glEnable(GL_TEXTURE_2D);
						glBindTexture(GL_TEXTURE_2D, worldGroundTexture);
					}
					
					glBegin(GL_TRIANGLES);
					glTexCoord2f(0.5f, 0.5f);
					glVertex3d(0, 0, 0);
					glTexCoord2f(0.5f+0.5f*cosf(angStart), 0.5f+0.5f*sinf(angStart));
					glVertex3d(cos(angStart) * r, sin(angStart) * r, 0);
					glTexCoord2f(0.5f+0.5f*cosf(angEnd), 0.5f+0.5f*sinf(angEnd));
					glVertex3d(cos(angEnd) * r, sin(angEnd) * r, 0);
					glEnd();
					
					glEnable(GL_TEXTURE_2D);
					glBindTexture(GL_TEXTURE_2D, worldTexture);
					
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
					
					// draw ground shadow
					glColor3d(1, 1, 1);
					glEnable(GL_BLEND);
					glBlendFunc(GL_ZERO, GL_SRC_COLOR);
					glDepthMask( GL_FALSE );
					glEnable(GL_POLYGON_OFFSET_FILL);
					
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
					
					glDisable(GL_POLYGON_OFFSET_FILL);
					glDepthMask( GL_TRUE );
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glDisable(GL_BLEND);
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
	
	void ViewerWidget::renderShape(const Polygon& shape, const double height, const Color& color)
	{
		const size_t segmentCount = shape.size();
		
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
	
	void ViewerWidget::renderSimpleObject(PhysicalObject *object)
	{
		SimpleDisplayList *userData = new SimpleDisplayList;
		object->userData = userData;
		glNewList(userData->list, GL_COMPILE);
		
		glDisable(GL_LIGHTING);
		if (!object->getHull().empty())
		{
			for (PhysicalObject::Hull::const_iterator it = object->getHull().begin(); it != object->getHull().end(); ++it)
			{
				renderShape(it->getShape(), it->getHeight(), object->getColor());
			}
		}
		else
		{
			Polygon shape;
			const size_t segmentCount(32);
			shape.reserve(segmentCount);
			const double radius(object->getRadius());
			for (size_t i=0; i<segmentCount; ++i)
			{
				const double alpha(i*2.*M_PI/double(segmentCount));
				shape.push_back(Point(cos(alpha) * radius, sin(alpha) * radius));
			}
			renderShape(shape, object->getHeight(), object->getColor());
		}
		glEnable(GL_LIGHTING);
		
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
		glClearColor(world->color.r(), world->color.g(), world->color.b(), 1.0);
		
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
		
		helpWidget = bindTexture(QPixmap(QString(":/widgets/help.png")), GL_TEXTURE_2D, GL_RGBA);
		centerWidget = bindTexture(QPixmap(QString(":/widgets/center.png")), GL_TEXTURE_2D, GL_RGBA);
		
		selectionTexture = bindTexture(QPixmap(QString(":/textures/selection.png")), GL_TEXTURE_2D, GL_RGBA);
		worldTexture = bindTexture(QPixmap(QString(":/textures/world.png")), GL_TEXTURE_2D, GL_LUMINANCE8);
		wallTexture = bindTexture(QPixmap(QString(":/textures/wall.png")), GL_TEXTURE_2D, GL_LUMINANCE8);
		if (world->hasGroundTexture())
		{
			glGenTextures(1, &worldGroundTexture);
			glBindTexture(GL_TEXTURE_2D, worldGroundTexture);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, world->groundTexture.width, world->groundTexture.height, 0, GL_BGRA, GL_UNSIGNED_BYTE, &world->groundTexture.data[0]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		}
		worldList = glGenLists(1);
		renderWorld();
		
		// render all static types
		managedObjects[&typeid(EPuck)] = new EPuckModel(this);
		managedObjects[&typeid(Marxbot)] = new MarxbotModel(this);
		managedObjects[&typeid(Thymio2)] = new Thymio2Model(this);
		
		// let subclass manage their static types
		renderObjectsTypesHook();
	}
	
	void ViewerWidget::renderScene(double left, double right, double bottom, double top, double zNear, double zFar)
	{
		//float aspectRatio = (float)width() / (float)height();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(left, right, bottom, top, zNear, zFar);//(-aspectRatio, aspectRatio, -1, 1, 2, 2000);
		
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		glRotated(-90, 1, 0, 0);
		glRotated(rad2deg * -camera.pitch, 1, 0, 0);
		glRotated(90, 0, 0, 1);
		glRotated(rad2deg * -camera.yaw, 0, 0, 1);
		
		glTranslated(-camera.pos.x(), -camera.pos.y(), -camera.altitude);
		
		GLfloat LightPosition[] = {(GLfloat)world->w/2, (GLfloat)world->h/2, 60, 1};
		glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
		
		glCallList(worldList);
		for (World::ObjectsIterator it = world->objects.begin(); it != world->objects.end(); ++it)
		{
			// if required, initialize this object (display list)
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
			
			// draw object
			glPushMatrix();
			
			glTranslated((*it)->pos.x, (*it)->pos.y, 0);
			glRotated(rad2deg * (*it)->angle, 0, 0, 1);
			
			ViewerUserData* userData = polymorphic_downcast<ViewerUserData *>((*it)->userData);

			userData->draw(*it);
			displayObjectHook(*it);
			
			glPopMatrix();
		}

		// if an object is selected
		if (selectedObject)
		{
			glPushMatrix();
			
			glTranslated(selectedObject->pos.x, selectedObject->pos.y, 0);
			glRotated(rad2deg * selectedObject->angle, 0, 0, 1);
			
			// if it is being move, draw the object as it has not been drawn before
			if (movingObject)
			{
				ViewerUserData* userData = polymorphic_downcast<ViewerUserData *>(selectedObject->userData);
				userData->draw(selectedObject);
				displayObjectHook(selectedObject);
			}
			
			// draw the selection circle
			glEnable(GL_BLEND);
			glEnable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glBindTexture(GL_TEXTURE_2D, selectionTexture);
			glColor4d(1,1,1,1);
			glBegin(GL_QUADS);
				const double r(selectedObject->getRadius() * 1.5);
				glTexCoord2f(0.f, 0.f); glVertex3d(-r, -r, 0.1);
				glTexCoord2f(1.f, 0.f); glVertex3d(r, -r, 0.1);
				glTexCoord2f(1.f, 1.f); glVertex3d(r, r, 0.1);
				glTexCoord2f(0.f, 1.f); glVertex3d(-r, r, 0.1);
			glEnd();
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_BLEND);
			
			glPopMatrix();
		}
	}

	void ViewerWidget::picking(double left, double right, double bottom, double top, double zNear, double zFar)
	{
		pointedObject = 0;
		QPoint cursorPosition = mapFromGlobal(QCursor::pos());

		if (!rect().contains(cursorPosition,true)) // window does not contain cursor
			return;

		// prepare matricies for invertion
		QMatrix4x4 projection;
			projection.setToIdentity();
			projection.frustum(left, right, bottom, top, zNear, zFar);
		QMatrix4x4 modelview;
			modelview.setToIdentity();
			modelview.rotate(-90, 1, 0, 0);
			modelview.rotate(rad2deg * -camera.pitch, 1, 0, 0);
			modelview.rotate(90, 0, 0, 1);
			modelview.rotate(rad2deg * -camera.yaw, 0, 0, 1);
			modelview.translate(-camera.pos.x(), -camera.pos.y(), -camera.altitude);
		QMatrix4x4 transformMatrix = (projection*modelview).inverted();

		// cursor position in viewport coordinates
		const double fragmentX = double(cursorPosition.x() - width()/2) / (width()/2);
		const double fragmentY = double(height() - cursorPosition.y() - height()/2) / (height()/2);
		float depth;
		glReadPixels( cursorPosition.x(), height() - cursorPosition.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth );

		QVector4D input(fragmentX, fragmentY, 2*depth - 1, 1);
		input = transformMatrix*input;

		if (input.w() != 0.0) // valid pointed point
		{
			pointedPoint = QVector3D(input.x(), input.y(), input.z());
			pointedPoint /= input.w();
		}
		else
			return;

		// prepare to find which object is pointed
		Point cursor2Dpoint(pointedPoint.x(),pointedPoint.y());
		const double cursorRadius = 0.2f;
		for (World::ObjectsIterator it = world->objects.begin(); it != world->objects.end(); ++it)
		{
			const Vector distOCtoOC = (*it)->pos - cursor2Dpoint;		// distance between object bounding circle center and pointed point
			const double addedRay = (*it)->getRadius() + cursorRadius;	// sum of bounded circle radius
			if (distOCtoOC.norm2() <= (addedRay*addedRay)) 			// cursor point colide bounding circle
			{
				if (!(*it)->getHull().empty())				// check pointer circle and object hull
				{
					PhysicalObject::Hull hull = (*it)->getHull();
					for (PhysicalObject::Hull::const_iterator it2 = hull.begin(); it2 != hull.end(); ++it2) // check all convex shape of hull
					{
						const Polygon shape = it2->getTransformedShape();
						bool inside(true);
						for (size_t i = 0; i < shape.size(); i++)
						{
							if (shape.getSegment(i).dist(cursor2Dpoint) < -cursorRadius)
							{
								inside = false;
								break;
							}
						}
						if (inside)
						{
							pointedObject = *it;
							return;
						}
					}
				}
				else
				{
					// object circle collide cursor circle => test already done !
					pointedObject = *it;
					return;
				}
			}
		}
	}
	
	void ViewerWidget::glVertex2Screen(int x, int y)
	{
		glVertex2f(-1. + (x * 2.) / width(), 1. - (y * 2.) / height());
	}
	
	void ViewerWidget::displayWidgets()
	{
		glEnable(GL_BLEND);
		glEnable(GL_TEXTURE_2D);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		glColor4d(1,1,1,1);
		const int margin(24);
		const int size(48);
		
		glBindTexture(GL_TEXTURE_2D, helpWidget);
		glBegin(GL_QUADS);
		{
			const int yPos(0);
			glTexCoord2f(0.f, 0.f); glVertex2Screen(width() - margin - size, margin + size + yPos);
			glTexCoord2f(1.f, 0.f); glVertex2Screen(width() - margin, margin + size + yPos);
			glTexCoord2f(1.f, 1.f); glVertex2Screen(width() - margin, margin + yPos);
			glTexCoord2f(0.f, 1.f); glVertex2Screen(width() - margin - size, margin + yPos);
		}
		glEnd();
		
		glBindTexture(GL_TEXTURE_2D, centerWidget);
		glBegin(GL_QUADS);
		{
			const int yPos(48+12);
			glTexCoord2f(0.f, 0.f); glVertex2Screen(width() - margin - size, margin + size + yPos);
			glTexCoord2f(1.f, 0.f); glVertex2Screen(width() - margin, margin + size + yPos);
			glTexCoord2f(1.f, 1.f); glVertex2Screen(width() - margin, margin + yPos);
			glTexCoord2f(0.f, 1.f); glVertex2Screen(width() - margin - size, margin + yPos);
		}
		glEnd();
		
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_BLEND);
	}
	
	void ViewerWidget::clickWidget(QMouseEvent *event)
	{
		if (event->y() > 24 && event->y() < 72)
			helpActivated();
		else if (event->y() > 24+48+12 && event->y() < 72+48+12)
			camera = UpdatableCameraPose(world);
	}
	
	void ViewerWidget::displayMessages()
	{
		// bound message list
		while (messageList.size() > 20)
			messageList.pop_front();
		
		// if none, return
		if (messageList.empty())
			return;
		
		// fill background
		glEnable(GL_BLEND);
		glColor4d(1,1,1,0.8);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glBegin(GL_QUADS);
			glVertex2Screen(0, messageListHeight);
			glVertex2Screen(messageListWidth, messageListHeight);
			glVertex2Screen(messageListWidth, 0);
			glVertex2Screen(0, 0);
		glEnd();
		glDisable(GL_BLEND);
		
		// draw messages
		const int lineSpacing(fontMetrics.lineSpacing()+3);
		unsigned i = 0;
		const size_t messageListSize(messageList.size());
		for (MessageList::iterator it = messageList.begin(); it != messageList.end(); i++)
		{
			QColor color(it->color);
			color.setAlphaF(clamp(it->persistance, 0., 1.));
			qglColor(color);
			
			renderText(10, 5 + (i+1)*lineSpacing, it->message);

			if (it->persistance >= 0)
			{
				it->persistance -= elapsedTime;
				++it;
			}
			else
				it = messageList.erase(it);
		}
		if (messageList.size() != messageListSize)
			computeInfoMessageAreaSize();
	}
	
	void ViewerWidget::computeInfoMessageAreaSize()
	{
		messageListWidth = 0;
		for (MessageList::iterator it = messageList.begin(); it != messageList.end(); ++it)
			messageListWidth = std::max(messageListWidth, fontMetrics.width(it->message));
		const int lineSpacing(fontMetrics.lineSpacing()+3);
		messageListWidth += 20; 
		messageListHeight = messageList.size() * lineSpacing;
		if (messageListHeight)
			messageListHeight += 20;
	}

	void ViewerWidget::paintGL()
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		const double znear = 0.5;
		if (trackingView && selectedObject)
			camera.updateTracking(selectedObject->angle, QVector3D(selectedObject->pos.x, selectedObject->pos.y, selectedObject->getHeight()), znear);
		else
			camera.update();

		const double aspectRatio = double(width()) / double(height());
		renderScene(-aspectRatio*0.5*znear, aspectRatio*0.5*znear, -0.5*znear, 0.5*znear, znear, 2000);
		sceneCompletedHook();
		
		picking(-aspectRatio*0.5*znear, aspectRatio*0.5*znear, -0.5*znear, 0.5*znear, znear, 2000);
		
		displayMessages();
		displayWidgets();

		if (doDumpFrames)
			grabFrameBuffer().save(QString("enkiviewer-frame%1.png").arg(dumpFramesCounter++, (int)8, (int)10, QChar('0')));
	}
	
	void ViewerWidget::resizeGL(int width, int height)
	{
		glViewport(0, 0, width, height);
	}
	
	void ViewerWidget::keyPressEvent(QKeyEvent* event)
	{
		if (event->key() == Qt::Key_F1)
			helpActivated();
	}

	void ViewerWidget::mousePressEvent(QMouseEvent *event)
	{
		// initialization
		mouseGrabPos = event->pos();

		// change selected object
		if (event->button() == Qt::LeftButton)
		{
			if (event->x() > width() - 72 && event->x() < width() - 24)
			{
				clickWidget(event);
			}
			else if (!messageList.empty() && event->x() < messageListWidth && event->y() < messageListHeight)
			{
				const int messageIndex((event->y() - 5) / fontMetrics.lineSpacing());
				if (messageIndex >= 0 && messageIndex < messageList.size())
				{
					MessageList::iterator it(messageList.begin());
					std::advance(it, messageIndex);
					const QUrl link(it->link);
					if (!link.isEmpty())
						QDesktopServices::openUrl(link);
				}
			}
			else
			{
				if (selectedObject != pointedObject)
					setTracking(false);
				// only allow to deselect object if we are not actually moving it
				if (!movingObject)
				{
					if (isMovableByPicking(pointedObject))
						selectedObject = pointedObject;
					else
						selectedObject = 0;
				}
			}
		}

		// if pointed object is a robot call the clicked interaction function
		Robot* robot = dynamic_cast<Robot*>(pointedObject);
		if (robot)
		{
			Vector pointedPointXY(pointedPoint.x(), pointedPoint.y());
			pointedPointXY -= robot->pos;
			pointedPointXY = Matrix22(-robot->angle) * pointedPointXY;
			if (event->button() == Qt::LeftButton)
			{
				robot->mousePressEvent(PhysicalObject::MOUSE_BUTTON_LEFT, pointedPointXY.x, pointedPointXY.y, pointedPoint.z());
				mouseLeftButtonRobot = robot;
			}
			if (event->button() == Qt::RightButton)
			{
				robot->mousePressEvent(PhysicalObject::MOUSE_BUTTON_RIGHT, pointedPointXY.x, pointedPointXY.y, pointedPoint.z());
				mouseRightButtonRobot = robot;
			}
			if (event->button() == Qt::MiddleButton)
			{
				robot->mousePressEvent(PhysicalObject::MOUSE_BUTTON_MIDDLE, pointedPointXY.x, pointedPointXY.y, pointedPoint.z());
				mouseMiddleButtonRobot = robot;
			}
		}
	}
	
	void ViewerWidget::mouseReleaseEvent(QMouseEvent * event)
	{
		// make sure the selected object is in the world
		if (selectedObject)
		{
			world->addObject(selectedObject);
			movingObject = false;
		}
		
		// release previously-pressed buttons
		if ((event->button() == Qt::LeftButton) && mouseLeftButtonRobot)
		{
			mouseLeftButtonRobot->mouseReleaseEvent(PhysicalObject::MOUSE_BUTTON_LEFT);
			mouseLeftButtonRobot = 0;
		}
		if ((event->button() == Qt::RightButton) && mouseRightButtonRobot)
		{
			mouseRightButtonRobot->mouseReleaseEvent(PhysicalObject::MOUSE_BUTTON_RIGHT);
			mouseRightButtonRobot = 0;
		}
		if ((event->button() == Qt::MiddleButton) && mouseMiddleButtonRobot)
		{
			mouseMiddleButtonRobot->mouseReleaseEvent(PhysicalObject::MOUSE_BUTTON_MIDDLE);
			mouseMiddleButtonRobot = 0;
		}
	}
	
	void ViewerWidget::mouseMoveEvent(QMouseEvent *event)
	{
		if (!trackingView && selectedObject)
		{
			// object movements
			
			// rotate
			if (event->buttons() & Qt::RightButton)
			{
				if (!movingObject)
					world->removeObject(selectedObject);
				movingObject = true;

				const QPoint diff = event->pos() - mouseGrabPos;
				const double sensitivity = 10;
				selectedObject->angle -= sensitivity * (double)diff.x() / (1+width());
				mouseGrabPos = event->pos();
			}
			
			// translate
			else if (event->buttons() & Qt::LeftButton)
			{
				if ((event->pos() - mouseGrabPos).manhattanLength() > 10)
				{
					if (!movingObject)
						world->removeObject(selectedObject);
					movingObject = true;

					selectedObject->pos = Point(pointedPoint.x(),pointedPoint.y());
					selectedObject->speed = Vector(0,0);
					selectedObject->angSpeed = 0;
				}
			}
		}
		else
		{
			// camera movements
			
			// translate
			if ((event->buttons() & Qt::LeftButton) && (!trackingView || (event->modifiers() & Qt::ShiftModifier)))
			{
				const QPoint diff = event->pos() - mouseGrabPos;
				if (event->modifiers() & Qt::ShiftModifier)
				{
					const double sensitivity = -(1 + 0.1*camera.altitude) * 0.1;
					camera.pos.rx() += sensitivity * diff.y()*camera.forward.x();
					camera.pos.ry() += sensitivity * diff.y()*camera.forward.y();
					camera.altitude += sensitivity * diff.y()*camera.forward.z();
				}
				else
				{
					const double sensibility = 20 + 2.*camera.altitude;
					const double sizeFactor = 1 + (width() + height()) / 2;
					camera.pos.rx() -= sensibility * (diff.x()*camera.left.x() + diff.y()*camera.up.x()) / sizeFactor;
					camera.pos.ry() -= sensibility * (diff.x()*camera.left.y() + diff.y()*camera.up.y()) / sizeFactor;
					camera.altitude -= sensibility * (diff.x()*camera.left.z() + diff.y()*camera.up.z()) / sizeFactor;
				}
				camera.altitude = std::max(camera.altitude, 0.);
				mouseGrabPos = event->pos();
			}
			
			// rotate
			else if (event->buttons() & Qt::RightButton)
			{
				const QPoint diff = event->pos() - mouseGrabPos;
				const double sensitivity = 4;
				camera.userYaw -= sensitivity * (double)diff.x() / (1+width());

				const double delta = 0.01;
				camera.pitch = clamp(camera.pitch - sensitivity * (double)diff.y() / (1+height()), -M_PI / 2 + delta, M_PI / 2 - delta);

				mouseGrabPos = event->pos();
			}
		}
	}
	
	void ViewerWidget::mouseDoubleClickEvent(QMouseEvent *event)
	{
		setTracking(true);
	}
	
	void ViewerWidget::wheelEvent(QWheelEvent * event)
	{
		// zoom
		if (trackingView)
		{
			camera.radius *= 1 - 0.0003*event->delta();
			if (camera.radius < 1.0)
				camera.radius = 1.0;
		}

		// translate camera
		else
		{
			const double sensitivity = (1 + 0.1*camera.altitude) * 0.003;
			camera.pos.rx() += sensitivity * event->delta()*camera.forward.x();
			camera.pos.ry() += sensitivity * event->delta()*camera.forward.y();
			camera.altitude += sensitivity * event->delta()*camera.forward.z();
			camera.altitude = std::max(camera.altitude, 0.);
		}
	}
	
	void ViewerWidget::timerEvent(QTimerEvent * event)
	{
		world->step(double(timerPeriodMs)/1000., 3);
		updateGL();
	}
	
	//! Help button or F1 have been pressed
	void ViewerWidget::helpActivated()
	{
		showHelp();
	}
}
