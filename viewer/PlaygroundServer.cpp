/*
   Enki - a fast 2D robot simulator
   Copyright © 2017 Mathieu Lirzin <mathieu.lirzin@etu.u-bordeaux.fr>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <dashel/dashel.h>
#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <QApplication>
#include <QtGui>

/*!	\file PlaygroundServer.cpp
	\brief Run a simulation of a world that send its state to stdout.
*/

class DumbLogger: public Dashel::Hub
{
private:
	Dashel::Stream* stream;

public:
	DumbLogger()
	{
		stream = connect("stdout:");
	}

	void log(const std::string& str)
	{
		stream->write(str.c_str(), str.length());
	}
};

using namespace Enki;

class EnkiPlaygroundServer : public ViewerWidget
{
private:
	DumbLogger logger;

public:
	EnkiPlaygroundServer(World *world, QWidget *parent = 0) :
		ViewerWidget(world, parent),
		logger()
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

	virtual ~EnkiPlaygroundServer()
	{
	}

	virtual void timerEvent(QTimerEvent * event)
	{
		ViewerWidget::timerEvent(event);
	}

	virtual void sceneCompletedHook()
	{
		// TODO: Send something more meaningful when serialization is available.
		logger.log("sceneCompletedHook\n");
	}
};

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	World world(120, Color(0.9, 0.9, 0.9), World::GroundTexture());
	EnkiPlaygroundServer viewer(&world);

	viewer.show();

	return app.exec();
}
