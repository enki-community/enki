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
#include <iostream>

#ifdef USE_SDL
#include <SDL/SDL.h>
#endif

/*!	\file Studio.cpp
	\brief Test of the Qt-based viewer widget
*/

using namespace Enki;
using namespace std;

class EnkiPlayground : public ViewerWidget
{
protected:
	#ifdef USE_SDL
	QVector<SDL_Joystick *> joysticks;
	QVector<EPuck*> epucks;
	#endif
	
public:
	EnkiPlayground(World *world, QWidget *parent = 0) :
		ViewerWidget(world, parent)
	{
		PhysicalObject* o = new PhysicalObject;
		
		const double amount = 9;
		const double radius = 5;
		const double height = 20;
		Polygone p;
		for (double a = 0; a < 2*M_PI; a += 2*M_PI/amount)
			p.push_back(Point(radius * cos(a), radius * sin(a)));
		
		o->setShape(p, height);
		o->setMass(-1);
		o->pos = Point(100, 100);
		o->setColor(Color(0.4,0.6,0.8));
		o->commitPhysicalParameters();
		world->addObject(o);
		
		for (int i = 0; i < 20; i++)
		{
			PhysicalObject* o = new PhysicalObject;
			o->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
			o->setMass(10);
			o->setColor(Color(0.9, 0.2, 0.2));
			o->viscousFrictionTau = 10000000;
			o->viscousMomentFrictionTau = 10000000;
			o->commitPhysicalParameters();
			world->addObject(o);
		}
		
		Polygone p2;
		p2.push_back(Point(5,1));
		p2.push_back(Point(-5,1));
		p2.push_back(Point(-5,-1));
		p2.push_back(Point(5,-1));
		for (int i = 0; i < 5; i++)
		{
			PhysicalObject* o = new PhysicalObject;
			o->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
			o->setShape(p2, 3);
			o->setMass(30);
			o->setColor(Color(0.2, 0.1, 0.6));
			o->viscousFrictionTau = 10000000;
			o->viscousMomentFrictionTau = 10000000;
			o->commitPhysicalParameters();
			world->addObject(o);
		}
		
		#ifdef USE_SDL
		if((SDL_Init(SDL_INIT_JOYSTICK)==-1))
		{
			cerr << "Error : Could not initialize SDL: " << SDL_GetError() << endl;
			addDefaultsRobots(world);
			return;
		}
		
		int joystickCount = SDL_NumJoysticks();
		for (int i = 0; i < joystickCount; ++i)
		{
			SDL_Joystick* joystick = SDL_JoystickOpen(i);
			if (!joystick)
			{
				cerr << "Error: Can't open joystick " << i << endl;
				continue;
			}
			if (SDL_JoystickNumAxes(joystick) < 2)
			{
				cerr << "Error: not enough axis on joystick" << i<< endl;
				SDL_JoystickClose(joystick);
				continue;
			}
			joysticks.push_back(joystick);
			
			EPuck *epuck = new EPuck;
			epuck->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
			epucks.push_back(epuck);
			world->addObject(epuck);
		}
		
		#else
		addDefaultsRobots(world);
		#endif
	}
	
	void addDefaultsRobots(World *world)
	{
		EPuck *epuck = new EPuck;
		epuck->pos = Point(60, 50);
		//epuck->leftSpeed = 5;
		//epuck->rightSpeed = 5;
		world->addObject(epuck);
		
		epuck = new EPuck;
		epuck->pos = Point(40, 50);
		epuck->leftSpeed = 5;
		epuck->rightSpeed = 5;
		epuck->setColor(Color(1, 0, 0));
		world->addObject(epuck);
	}
	
	~EnkiPlayground()
	{
		#ifdef USE_SDL
		for (int i = 0; i < joysticks.size(); ++i)
			SDL_JoystickClose(joysticks[i]);
		SDL_Quit();
		#endif
	}
	
	virtual void timerEvent(QTimerEvent * event)
	{
		#ifdef USE_SDL
		SDL_JoystickUpdate();
		for (int i = 0; i < joysticks.size(); ++i)
		{
			#define SPEED_MAX 12.
			double x = SDL_JoystickGetAxis(joysticks[i], 0) / (32767. / SPEED_MAX);
			double y = -SDL_JoystickGetAxis(joysticks[i], 1) / (32767. / SPEED_MAX);
			EPuck* epuck = epucks[i];
			epuck->leftSpeed = y + x;
			epuck->rightSpeed = y - x;
		}
		#endif
		ViewerWidget::timerEvent(event);
	}
};

// http://qtnode.net/wiki?title=Qt_with_cmake
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	
	// Create the world and the viewer
	World world(120, 120);
	EnkiPlayground viewer(&world);
	
	viewer.show();
	
	return app.exec();
}

