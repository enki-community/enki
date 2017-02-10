/*
  Enki - a fast 2D robot simulator
  Copyright © 2017 Sébastien Pouteau <sebastien.pouteau@etu.u-bordeaux.fr>
  Copyright © 2017 Mathieu Lirzin <mathieu.lirzin@etu.u-bordeaux.fr>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cassert>
#include <enki/Network.h>
#include <enki/Serialize.h>
#include <iostream>
#include <QApplication>
#include <QtGui>
#include <thread>
#include <unistd.h>
#include <viewer/Viewer.h>

using namespace Enki;
using namespace Dashel;
using namespace std;

class ViewerServer: public ViewerWidget
{
private:
	Server* m_server;

public:
	ViewerServer(World* w, Server* s, QWidget *parent = 0) :
		ViewerWidget(w, parent)
	{
		m_server = s;
	}

	virtual void sceneCompletedHook()
	{
		m_server->sendAll();
	}
};

static void threadClient(Client* c)
{
	c->run();
}

static void threadServer(Server* s)
{
	s->run();
}

int main(int argc, char* argv[])
{
	// Init QT
	QApplication app(argc, argv);
	setlocale(LC_ALL, "C");

	try
	{
		if (argc > 1) // client
		{
			Client *client = new Client(argv[1]);

			thread thread_client(threadClient, client);

			sleep(1); // Wait client thread initialization

			ViewerWidget viewer(client->getWorld(), 0);
			viewer.show();
			app.exec();
		}
		else // server
		{
			World* world = new World(120, 90, Color(0.9, 0.9, 0.9), World::GroundTexture());

			// Add Thymio
			Thymio2 *thymio = new Thymio2;
			thymio->pos = Point(0, 0);

			thymio->setLedColor(Thymio2::TOP, Enki::Color::green);
			thymio->setLedColor(Thymio2::BOTTOM_LEFT, Enki::Color::red);
			thymio->setLedColor(Thymio2::BOTTOM_RIGHT, Enki::Color::blue);

			thymio->setLedIntensity(Thymio2::BUTTON_UP, 1.0);
			thymio->setLedIntensity(Thymio2::BUTTON_DOWN, 1.0);
			thymio->setLedIntensity(Thymio2::BUTTON_LEFT, 1.0);
			thymio->setLedIntensity(Thymio2::BUTTON_RIGHT, 1.0);

			thymio->setLedIntensity(Thymio2::RING_0, 1.0);
			thymio->setLedIntensity(Thymio2::RING_1, 1.0);
			thymio->setLedIntensity(Thymio2::RING_2, 1.0);
			thymio->setLedIntensity(Thymio2::RING_3, 1.0);
			thymio->setLedIntensity(Thymio2::RING_4, 1.0);
			thymio->setLedIntensity(Thymio2::RING_5, 1.0);
			thymio->setLedIntensity(Thymio2::RING_6, 1.0);
			thymio->setLedIntensity(Thymio2::RING_7, 1.0);

			thymio->setLedIntensity(Thymio2::IR_FRONT_0, 1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_1, 1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_2, 1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_3, 1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_4, 1.0);
			thymio->setLedIntensity(Thymio2::IR_FRONT_5, 1.0);

			thymio->setLedIntensity(Thymio2::IR_BACK_0, 1.0);
			thymio->setLedIntensity(Thymio2::IR_BACK_1, 1.0);

			thymio->setLedIntensity(Thymio2::LEFT_RED, 1.0);
			thymio->setLedIntensity(Thymio2::LEFT_BLUE, 1.0);
			thymio->setLedIntensity(Thymio2::RIGHT_BLUE, 1.0);
			thymio->setLedIntensity(Thymio2::RIGHT_RED, 1.0);

			thymio->leftSpeed = 3;
			thymio->rightSpeed = 4;

			world->addObject(thymio);

			Server* server = new Server(world);

			thread thread_server(threadServer, server);

			ViewerServer* viewer = new ViewerServer(world, server);
			viewer->show();
			app.exec();
		}
	}
	catch (DashelException e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
