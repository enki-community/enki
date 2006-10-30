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

#include <enki/robots/s-bot/SbotCam.h>
#include <enki/robots/s-bot/Sbot.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <limits>

/*!	\file SbotCam.cpp
	\brief Implementation of the Sbot 1D circular camera
*/
namespace Enki
{
	SbotCam::SbotCam(Sbot *owner, unsigned halfPixelCount) :
		zbuffer(halfPixelCount * 2),
		image(halfPixelCount * 2),
		// This is the clean and logic way to do it
		cam0(owner, Point(0, 0), 0, -M_PI/2, M_PI/2, halfPixelCount),
		cam1(owner, Point(0, 0), 0, M_PI/2, M_PI/2, halfPixelCount)
		// Old version, use only if you need backward compatibility
		/*cam0(owner, Point(0, 0), 0,  M_PI/2, M_PI/2, halfPixelCount),
		cam1(owner, Point(0, 0), 0, -M_PI/2, M_PI/2, halfPixelCount)*/
	{
		this->r = std::numeric_limits<double>::max();
		this->owner = owner;
	}

	void SbotCam::objectStep(double dt, PhysicalObject *po, World *w) 
	{
		cam0.objectStep(dt, po, w);
		cam1.objectStep(dt, po, w);
	};

	void SbotCam::init()
	{
		cam0.init();
		cam1.init();
	}
	
	void SbotCam::wallsStep(World *w)
	{
		cam0.wallsStep(w);
		cam1.wallsStep(w);
	}
	
	void SbotCam::finalize(double dt)
	{
		cam0.finalize(dt);
		cam1.finalize(dt);
		size_t camPixelCount = cam0.zbuffer.size();
		std::copy(&cam0.zbuffer[0], &cam0.zbuffer[camPixelCount], &zbuffer[0]);
		std::copy(&cam1.zbuffer[0], &cam1.zbuffer[camPixelCount], &zbuffer[camPixelCount]);
		std::copy(&cam0.image[0], &cam0.image[camPixelCount], &image[0]);
		std::copy(&cam1.image[0], &cam1.image[camPixelCount], &image[camPixelCount]);
	}
	
	void SbotCam::setFogConditions(bool useFog, double density, Color threshold)
	{
		cam0.useFog = useFog;
		cam0.fogDensity = density;
		cam0.lightThreshold = threshold;
		cam1.useFog = useFog;
		cam1.fogDensity = density;
		cam1.lightThreshold = threshold;
	}
	
	void SbotCam::setRange(double range)
	{
		this->r = range;
		owner->sortLocalInteractions();
	}
	
	void SbotCam::setPixelOperationFunctor(PixelOperationFunctor *pixelOperationFunctor)
	{
		cam0.pixelOperation = pixelOperationFunctor;
		cam1.pixelOperation = pixelOperationFunctor;
	}
}


