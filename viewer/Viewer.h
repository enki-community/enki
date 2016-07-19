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

#ifndef __ENKI_VIEWER_H
#define __ENKI_VIEWER_H

#include <typeinfo>
#include <QGLWidget>
#include <QPoint>
#include <QPointF>
#include <QMap>
#include <enki/Geometry.h>
#include <enki/PhysicalEngine.h>

/*!	\file Viewer.h
	\brief Definition of the Qt-based viewer widget
*/

class QTimerEvent;
class QMouseEvent;
class QWheelEvent;
class QWidget;

namespace Enki
{
	class World;
	class PhysicalObject;
	
	class ViewerWidget : public QGLWidget
	{
		Q_OBJECT
	
	public:
		const int timerPeriodMs;
		
		class ViewerUserData : public PhysicalObject::UserData
		{
		public:
			virtual void draw(PhysicalObject* object) const = 0;
			virtual void drawSpecial(PhysicalObject* object, int param = 0) const { }
			// for data managed by the viewer, called upon viewer destructor
			virtual void cleanup(ViewerWidget* viewer) { }
		};
		
		// complex robot, one per robot type stored here
		class CustomRobotModel : public ViewerUserData
		{
		public:
			QVector<GLuint> lists;
			QVector<GLuint> textures;
		
		public:
			CustomRobotModel();
		};
		
		// Camera pose
		struct CameraPose
		{
			QPointF pos; //!< (x,y) position of the camera
			double altitude; //!< altitude (z) of the camera
			double yaw; //!< yaw angle, mathematical orientation
			double pitch; //!< pitch angle, negative looking down, positive looking up
			
			CameraPose(QPointF pos, double altitude, double yaw, double pitch);
		};
		
		CameraPose camera; //!< current camera pose
		
	protected:
		World *world;
		
		GLuint worldList;
		GLuint worldTexture;
		GLuint wallTexture;
		GLuint worldGroundTexture;
		
		typedef QMap<const std::type_info*, ViewerUserData*> ManagedObjectsMap;
		typedef QMapIterator<const std::type_info*, ViewerUserData*> ManagedObjectsMapIterator;
		ManagedObjectsMap managedObjects;
		typedef QMap<const std::type_info*, const std::type_info*> ManagedObjectsAliasesMap;
		typedef QMapIterator<const std::type_info*, const std::type_info*> ManagedObjectsAliasesMapIterator;
		ManagedObjectsAliasesMap managedObjectsAliases;
		
		bool mouseGrabbed;
		QPoint mouseGrabPos;
		double wallsHeight;
		
		bool doDumpFrames;
		int dumpFramesCounter;
	
	public:
		ViewerWidget(World *world, QWidget *parent = 0);
		~ViewerWidget();
	
	public slots:
		void setCamera(QPointF pos, double altitude, double yaw, double pitch);
		void setCamera(double x, double y, double altitude, double yaw, double pitch);
		void restartDumpFrames();
		void setDumpFrames(bool doDump);
		
	protected:
		void renderInterSegmentShadow(const Vector& a, const Vector& b, const Vector& c, double height);
		void renderSegmentShadow(const Segment& segment, double height);
		void renderSegment(const Segment& segment, double height);
		void renderWorldSegment(const Segment& segment);
		void renderWorld();
		void renderShape(const Polygone& shape, const double height, const Color& color);
		void renderSimpleObject(PhysicalObject *object);
		virtual void renderObjectsTypesHook();
		virtual void renderObjectHook(PhysicalObject *object);
		virtual void displayObjectHook(PhysicalObject *object);
		virtual void sceneCompletedHook();
		
		void initializeGL();
		void paintGL();
		void resizeGL(int width, int height);
		
		void timerEvent(QTimerEvent * event);
		void mousePressEvent(QMouseEvent *event);
		void mouseReleaseEvent(QMouseEvent * event);
		void mouseMoveEvent(QMouseEvent *event);
		void wheelEvent(QWheelEvent * event);
	};
}

#endif
