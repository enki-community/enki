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

#include "Playground.h"
#include <Playground.moc>

using namespace Enki;

EnkiPlayground::EnkiPlayground(World *world, QWidget *parent) : timerPeriodMs(30), world(world), subjectiveView(false)
{
	thymio = new Thymio2();
	thymio->pos = Point(0, 0);
	world->addObject(thymio);

	#define PROBLEM_GENERIC_TOY
	#define PROBLEM_BALL_LINE
	//#define PROBLEM_LONE_EPUCK
	
	#ifdef PROBLEM_GENERIC_TOY
	{
		const double amount = 9;
		const double radius = 5;
		const double height = 20;
		Polygone p;
		for (double a = 0; a < 2*M_PI; a += 2*M_PI/amount)
			p.push_back(Point(radius * cos(a), radius * sin(a)));
		
		PhysicalObject* o = new PhysicalObject(true);
		PhysicalObject::Hull hull(Enki::PhysicalObject::Part(p, height));
		o->setCustomHull(hull, -1);
		o->setColor(Color(0.4,0.6,0.8));
		o->pos = Point(100, 100);
		world->addObject(o);
	}
	
	for (int i = 0; i < 20; i++)
	{
		PhysicalObject* o = new PhysicalObject(true);
		o->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
		o->setCylindric(1, 1, 10);
		o->setColor(Color(0.9, 0.2, 0.2));
		o->dryFrictionCoefficient = 0.01;
		world->addObject(o);
	}
	
	Polygone p2;
	p2.push_back(Point(5,1));
	p2.push_back(Point(-5,1));
	p2.push_back(Point(-5,-1));
	p2.push_back(Point(5,-1));
	for (int i = 0; i < 5; i++)
	{
		PhysicalObject* o = new PhysicalObject(true);
		PhysicalObject::Hull hull(Enki::PhysicalObject::Part(p2, 3));
		o->setCustomHull(hull, 30);
		o->setColor(Color(0.2, 0.1, 0.6));
		o->collisionElasticity = 0.2;
		o->pos = Point(UniformRand(20, 100)(), UniformRand(20, 100)());
		world->addObject(o);
	}
	
	// cross shape
	{
		PhysicalObject* o = new PhysicalObject(true);
		PhysicalObject::Hull hull;
		hull.push_back(Enki::PhysicalObject::Part(Polygone() << Point(5,1) << Point(-5,1) << Point(-5,-1) << Point(5,-1), 2));
		hull.push_back(Enki::PhysicalObject::Part(Polygone() << Point(1,5) << Point(-1,5) << Point(-1,-5) << Point(1,-5), 4));
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
		PhysicalObject* o = new PhysicalObject(true);
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
	
	Marxbot *marxbot = new Marxbot;
	marxbot->pos = Point(60, 50);
	marxbot->leftSpeed = 8;
	marxbot->rightSpeed = 2;
	world->addObject(marxbot);
	
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

	viewer = new ViewerWidget(world);
		viewer->camera.pos = QPointF(2.6,0);
		viewer->camera.altitude = 10;
		viewer->camera.yaw = -1.57;
		viewer->camera.pitch = -1.57;
	frameCounter = new QLabel(QString("frame: "));
		frameCounter->setFixedSize(100,30);
		frameCounter->setStyleSheet("background-color:rgb(150,150,150);");
	pointedPosition = new QLabel("cursor: 0 ; 0");
		pointedPosition->setFixedSize(220,30);
		pointedPosition->setStyleSheet("background-color:rgb(150,150,150);");
	resetButton = new QPushButton("reset");
		resetButton->setFixedSize(100,30);
		connect(resetButton,SIGNAL(clicked(bool)),this,SLOT(resetCallback(bool)));
		resetButton->setStyleSheet("background-color:rgb(150,150,150);");
	centralWidget = new QWidget();
		centralWidget->setStyleSheet("background-color:rgb(0,0,0);");
	toolbar = new QHBoxLayout();
		toolbar->setContentsMargins(0,0,0,0);
	mainwidget = new QWidget(this);
		setCentralWidget(mainwidget);

	QVBoxLayout* layout1 = new QVBoxLayout();
	mainwidget->setLayout(layout1);
	QWidget* widget2 = new QWidget();
		widget2->setFixedHeight(40);
		widget2->setStyleSheet("background-color:rgb(250,150,150);");

	widget2->setLayout(toolbar);
	layout1->addWidget(widget2);
	toolbar->addWidget(resetButton);
	toolbar->addWidget(frameCounter);
	toolbar->addWidget(pointedPosition);
	toolbar->addWidget(new QWidget());
	layout1->addWidget(viewer);

	startTimer(timerPeriodMs);
	time.start();
}

EnkiPlayground::~EnkiPlayground()
{
	#ifdef USE_SDL
	for (int i = 0; i < joysticks.size(); ++i)
		SDL_JoystickClose(joysticks[i]);
	SDL_Quit();
	#endif
}

void EnkiPlayground::addDefaultsRobots(World *world)
{
	EPuck *epuck = new EPuck;
	epuck->pos = Point(60, 50);
	world->addObject(epuck);
	
	epuck = new EPuck;
	epuck->pos = Point(40, 50);
	epuck->leftSpeed = 5;
	epuck->rightSpeed = 5;
	epuck->setColor(Color(1, 0, 0));
}

void EnkiPlayground::timerEvent(QTimerEvent * event)
{
	static int fireCounter = 0;
	#ifdef USE_SDL
	SDL_JoystickUpdate();
	viewer->doDumpFrames = false;
	for (int i = 0; i < joysticks.size(); ++i)
	{
		EPuck* epuck = epucks[i];
		
		if (world->hasGroundTexture())
			cout << "Robot " << i << " is on ground of colour " << world->getGroundColor(epuck->pos) << endl;
		
		#define SPEED_MAX 13.
		double x = SDL_JoystickGetAxis(joysticks[i], 0) / (32767. / SPEED_MAX);
		double y = -SDL_JoystickGetAxis(joysticks[i], 1) / (32767. / SPEED_MAX);
		epuck->leftSpeed = y + x;
		epuck->rightSpeed = y - x;
		
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
		viewer->doDumpFrames |= SDL_JoystickGetButton(joysticks[i], 0);
	}
	if (joysticks.size() > 0 && subjectiveView)
	{
		const EPuck* epuck = epucks[0];
		Vector p(epuck->pos);
		viewer->camera.pos.setX(p.x+cos(viewer->camera.yaw)*7);
		viewer->camera.pos.setY(p.y+sin(viewer->camera.yaw)*7);
		viewer->camera.yaw = epuck->angle;
		viewer->camera.altitude = 11;
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

	// update widgets
	viewer->timerEvent(timerPeriodMs/1000.);
	frameCounter->setText(QString("frame: ") + QString::number(time.elapsed()));
	QVector3D p = viewer->getPointedPoint();
	pointedPosition->setText("cursor: (" + QString::number(p.x(),'f',2) + " , " + QString::number(p.y(),'f',2) + " , " + QString::number(p.z(),'f',2) + " )" );
	time.restart();
}

void EnkiPlayground::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_C)
	{
		subjectiveView = !subjectiveView;
		if (subjectiveView)
			viewer->camera.pitch = -M_PI/8;
		event->accept();
	}
}

void EnkiPlayground::resizeEvent(QResizeEvent* event)
{
	QMainWindow::resizeEvent(event);
	mainwidget->resize(event->size());
}

void EnkiPlayground::resetCallback(bool clicked)
{
	viewer->resetScene();
}



int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	
	// Create the world and window
	bool igt(app.arguments().size() > 1);
	QImage gt;
	if (igt) gt = QGLWidget::convertToGLFormat(QImage(app.arguments().last()));
	igt = !gt.isNull();
	#if QT_VERSION >= QT_VERSION_CHECK(4,7,0)
	World world(120, Color(0.9, 0.9, 0.9), igt ? World::GroundTexture(gt.width(), gt.height(), (const uint32_t*)gt.constBits()) : World::GroundTexture());
	#else
	World world(120, Color(0.9, 0.9, 0.9), igt ? World::GroundTexture(gt.width(), gt.height(), (uint32_t*)gt.bits()) : World::GroundTexture());
	#endif

	EnkiPlayground window(&world);
	window.resize(1000, 800);
 	window.setWindowTitle("New Enki Viewer test");
	window.show();

	return app.exec();
}

