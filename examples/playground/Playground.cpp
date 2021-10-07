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

#include <viewer/Viewer.h>
#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/marxbot/Marxbot.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <QApplication>
#include <QtGui>
#include <iostream>

#ifdef USE_SDL
#include <SDL.h>
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
	#endif
	QVector<EPuck*> epucks;
	QMap<PhysicalObject*, int> bullets;
	
public:
	EnkiPlayground(World *world, QWidget *parent = 0) :
		ViewerWidget(world, parent)
	{
		#define PROBLEM_CENTERED_THYMIO2
		
		#ifdef PROBLEM_CENTERED_THYMIO2
		{
			Thymio2 *thymio = new Thymio2;
			thymio->pos = Point(0, 0);
			thymio->setLedColor(Thymio2::TOP,Color(1.0,0.0,0.0,1.0));
			thymio->setLedColor(Thymio2::BOTTOM_LEFT,Color(0.0,1.0,0.0,1.0));
			thymio->setLedColor(Thymio2::BOTTOM_RIGHT,Color(0.0,0.0,1.0,1.0));

			thymio->setLedIntensity(Thymio2::BUTTON_UP,1.0);
			thymio->setLedIntensity(Thymio2::BUTTON_DOWN,1.0);
			thymio->setLedIntensity(Thymio2::BUTTON_LEFT,1.0);
			thymio->setLedIntensity(Thymio2::BUTTON_RIGHT,1.0);

			thymio->setLedIntensity(Thymio2::RING_0,1.0);
			thymio->setLedIntensity(Thymio2::RING_1,1.0);
			thymio->setLedIntensity(Thymio2::RING_2,1.0);
			thymio->setLedIntensity(Thymio2::RING_3,1.0);
			thymio->setLedIntensity(Thymio2::RING_4,1.0);
			thymio->setLedIntensity(Thymio2::RING_5,1.0);
			thymio->setLedIntensity(Thymio2::RING_6,1.0);
			thymio->setLedIntensity(Thymio2::RING_7,1.0);

			thymio->setLedIntensity(Thymio2::IR_FRONT_0,1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_1,1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_2,1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_3,1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_4,1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_5,1.0);

			thymio->setLedIntensity(Thymio2::IR_BACK_0,1.0);
			thymio->setLedIntensity(Thymio2::IR_BACK_1,1.0);

			thymio->setLedIntensity(Thymio2::LEFT_RED,1.0);
			thymio->setLedIntensity(Thymio2::LEFT_BLUE,1.0);
			thymio->setLedIntensity(Thymio2::RIGHT_BLUE,1.0);
			thymio->setLedIntensity(Thymio2::RIGHT_RED,1.0);
			thymio->leftSpeed = 3;
			thymio->rightSpeed = 4;
			world->addObject(thymio);
		}
		#endif // PROBLEM_CENTERED_THYMIO2

		#define PROBLEM_GENERIC_TOY
		#define PROBLEM_BALL_LINE
		//#define PROBLEM_LONE_EPUCK
		
		#ifdef PROBLEM_GENERIC_TOY
		{
			const double amount = 9;
			const double radius = 5;
			const double height = 20;
			Enki::Polygon p;
			for (double a = 0; a < 2*M_PI; a += 2*M_PI/amount)
				p.push_back(Point(radius * cos(a), radius * sin(a)));
			
			PhysicalObject* o = new PhysicalObject;
			PhysicalObject::Hull hull(Enki::PhysicalObject::Part(p, height));
			o->setCustomHull(hull, -1);
			o->setColor(Color(0.4,0.6,0.8));
			o->pos = Point(100, 100);
			world->addObject(o);
		}
		
		for (int i = 0; i < 10; i++)
		{
			PhysicalObject* o = new PhysicalObject;
			o->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
			o->setCylindric(1, 1, 10);
			o->setColor(Color(0.9, 0.2, 0.2));
			o->dryFrictionCoefficient = 0.01;
			world->addObject(o);
		}
		
		Enki::Polygon p2;
		p2.push_back(Point(5,1));
		p2.push_back(Point(-5,1));
		p2.push_back(Point(-5,-1));
		p2.push_back(Point(5,-1));
		for (int i = 0; i < 5; i++)
		{
			PhysicalObject* o = new PhysicalObject;
			PhysicalObject::Hull hull(Enki::PhysicalObject::Part(p2, 3));
			o->setCustomHull(hull, 30);
			o->setColor(Color(0.2, 0.1, 0.6));
			o->collisionElasticity = 0.2;
			o->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
			world->addObject(o);
		}
		
		// cross shape
		{
			PhysicalObject* o = new PhysicalObject;
			PhysicalObject::Hull hull;
			hull.push_back(Enki::PhysicalObject::Part(Enki::Polygon() << Point(5,1) << Point(-5,1) << Point(-5,-1) << Point(5,-1), 2));
			hull.push_back(Enki::PhysicalObject::Part(Enki::Polygon() << Point(1,5) << Point(-1,5) << Point(-1,-5) << Point(1,-5), 4));
			o->setCustomHull(hull, 60);
			o->setColor(Color(0.2, 0.4, 0.6));
			o->collisionElasticity = 0.2;
			o->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
			world->addObject(o);
		}
		#endif // PROBLEM_GENERIC_TOY
		
		#ifdef PROBLEM_BALL_LINE
		for (double d = 40; d < 60; d += 8)
		{
			PhysicalObject* o = new PhysicalObject;
			o->pos = Point(d, 20);
			o->setCylindric(4, 2, 10);
			o->setColor(Color(0.2, 0.2, 0.6));
			o->dryFrictionCoefficient = 0.;
			world->addObject(o);
		}
		#endif // PROBLEM_BALL_LINE
		
		#ifdef PROBLEM_LONE_EPUCK
		addDefaultsRobots(world);
		#endif // PROBLEM_LONE_EPUCK
		
		#define PROBLEM_MARXBOT
		
		#ifdef PROBLEM_MARXBOT
		Marxbot *marxbot = new Marxbot;
		marxbot->pos = Point(60, 50);
		marxbot->leftSpeed = 8;
		marxbot->rightSpeed = 2;
		world->addObject(marxbot);
		#endif // PROBLEM_MARXBOT
		
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
			//epuck->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
			epuck->pos = Point(20, 20);
			epucks.push_back(epuck);
			world->addObject(epuck);
		}
		cout << "Added " << joystickCount << " controlled e-pucks." << endl;
		#else // USE_SDL
		addDefaultsRobots(world);
		#endif // USE_SDL
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
		//world->addObject(epuck);
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
		static int fireCounter = 0;
		#ifdef USE_SDL
		SDL_JoystickUpdate();
		doDumpFrames = false;
		for (int i = 0; i < joysticks.size(); ++i)
		{
			EPuck* epuck = epucks[i];
			
			if (world->hasGroundTexture())
				cout << "Robot " << i << " is on ground of colour " << world->getGroundColor(epuck->pos) << endl;
			
			#define SPEED_MAX 13.
			//cout << "S " << epuck->infraredSensor2.getRayDist(0) << " " << epuck->infraredSensor2.getRayDist(1) << " " << epuck->infraredSensor2.getRayDist(2) << endl;
			#if 0 
			epuck->leftSpeed = - SDL_JoystickGetAxis(joysticks[i], 1) / (32767. / SPEED_MAX);
			epuck->rightSpeed = - SDL_JoystickGetAxis(joysticks[i], 4) / (32767. / SPEED_MAX);
			#else
			double x = SDL_JoystickGetAxis(joysticks[i], 0) / (32767. / SPEED_MAX);
			double y = -SDL_JoystickGetAxis(joysticks[i], 1) / (32767. / SPEED_MAX);
			epuck->leftSpeed = y + x;
			epuck->rightSpeed = y - x;
			#endif
			
			if ((SDL_JoystickGetButton(joysticks[i], 6) || SDL_JoystickGetButton(joysticks[i], 7)) &&
				(++fireCounter % 2) == 0)
			{
				PhysicalObject* o = new PhysicalObject;
				Vector delta(cos(epuck->angle), sin(epuck->angle));
				o->pos = epuck->pos + delta * 6;
				o->speed = epuck->speed + delta * 10;
				o->setCylindric(0.5, 0.5, 10);
				o->dryFrictionCoefficient = 0.01;
				o->setColor(Color(0.4, 0, 0));
				o->collisionElasticity = 1;
				bullets[o] = 300;
				world->addObject(o);
			}
			doDumpFrames |= SDL_JoystickGetButton(joysticks[i], 0);
		}
		#endif
		QMap<PhysicalObject*, int>::iterator i = bullets.begin();
		while (i != bullets.end())
		{
			QMap<PhysicalObject*, int>::iterator oi = i;
			++i;
			if (oi.value())
			{
				oi.value()--;
			}
			else
			{
				PhysicalObject* o = oi.key();
				world->removeObject(o);
				bullets.erase(oi);
				delete o;
			}
		}
		ViewerWidget::timerEvent(event);
	}
	
	virtual void sceneCompletedHook()
	{
		
	}
};

// http://qtnode.net/wiki?title=Qt_with_cmake
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	
	// Create the world and the viewer
	bool igt(app.arguments().size() > 1);
	QImage gt;
	if (igt)
	{
		QImage image(app.arguments().last());
		gt = image.convertToFormat(QImage::Format_ARGB32);
	}
	igt = !gt.isNull();
	#if QT_VERSION >= QT_VERSION_CHECK(4,7,0)
	const uint32_t *bits = (const uint32_t*)gt.constBits();
	#else
	uint32_t *bits = (uint32_t*)gt.bits();
	#endif
	World world(120, Color(0.9, 0.9, 0.9), igt ? World::GroundTexture(gt.width(), gt.height(), bits) : World::GroundTexture());
	EnkiPlayground viewer(&world);
	
	viewer.show();
	
	return app.exec();
}

