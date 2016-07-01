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

#ifndef __ENKI_PLAYGROUND_H
#define __ENKI_PLAYGROUND_H

#ifdef USE_SDL
#include <SDL.h>
#include <SDL_joystick.h>
#endif

#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <enki/robots/marxbot/Marxbot.h>
#include <QApplication>
#include <QtGui>
#include <QGLWidget>

#include "Viewer.h"

using namespace Enki;
using namespace std;

class EnkiPlayground : public QMainWindow
{
	Q_OBJECT

public:
	const int timerPeriodMs;


protected:
	#ifdef USE_SDL
	QVector<SDL_Joystick *> joysticks;
	#endif
	bool subjectiveView;
	QVector<EPuck*> epucks;
	QMap<PhysicalObject*, int> bullets;
	World* world;

	QTime time;
	Thymio2* thymio;

	ViewerWidget* viewer;
	QHBoxLayout* toolbar;
	QLabel *frameCounter, *pointedPosition;
	QWidget* centralWidget, *mainwidget;
	QPushButton* resetButton;

public:
	EnkiPlayground(World* world, QWidget *parent = 0);
	~EnkiPlayground();

	void addDefaultsRobots(World*world);
	void timerEvent(QTimerEvent* event);
	void keyPressEvent(QKeyEvent* event);
	void resizeEvent(QResizeEvent* event);

public slots:
	void resetCallback(bool clicked);
};

#endif
