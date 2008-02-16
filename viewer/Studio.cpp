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

#include "Viewer.h"
#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/alice/Alice.h>
#include <QApplication>
#include <QtGui>

/*!	\file Studio.cpp
	\brief Test of the Qt-based viewer widget
*/

using namespace Enki;

// http://qtnode.net/wiki?title=Qt_with_cmake
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	
	// Create the world and the viewer
	World world(120, 120);
	ViewerWidget viewer(&world);
	
	// Create a Khepera and position it
	/*for (int i = 0; i < 10; i++)
	{
		EPuck *ePuck = new EPuck;
		ePuck->pos = Point(100, 100);
		ePuck->leftSpeed = -21 + i * 4;
		ePuck->rightSpeed = -20 + i * 5;
		
		// objects are garbage collected by the world on destruction
		world.addObject(ePuck);
	}*/
	EPuck *epuck = new EPuck;
	epuck->pos = Point(60, 50);
	epuck->leftSpeed = 5;
	epuck->rightSpeed = 4;
	world.addObject(epuck);
	
	epuck = new EPuck;
	epuck->pos = Point(40, 50);
	epuck->color = Color(1, 0, 0);
	world.addObject(epuck);
	
	PhysicalObject* o = new PhysicalObject;
	Polygone p;
	p.push_back(Point(5,-5));
	p.push_back(Point(5, 5));
	p.push_back(Point(-5, 5));
	p.push_back(Point(-5,-5));
	o->setBoundingSurface(&p);
	o->pos = Point(100, 100);
	o->height = 10;
	o->mass = -1;
	o->color = Color(0.2,0.2,0.2);
	//world.addObject(o);
	
	viewer.show();
	
	return app.exec();
}
