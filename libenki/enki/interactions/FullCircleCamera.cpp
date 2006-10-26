/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2005 Stephane Magnenat <nct@ysagoon.com>
    Copyright (C) 2005 Laboratory of Intelligent Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator part of the Teem framework
    http://teem.epfl.ch
    Stephane Magnenat <stephane.magnenat@a3.epfl.ch>,
    Markus Waibel <markus.waibel@epfl.ch>
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

#include <enki/interactions/FullCircleCamera.h>
#include <enki/PhysicalEngine.h>
#include <algorithm>
#include <math.h>
#include <limits>

/*! \file FullCircleCamera.cpp
	\brief Implementation of general purpose 360 degrees vision camera based on the Sbot camera
*/

namespace Enki
{
	using namespace An;
	
	FullCircleCamera::FullCircleCamera(Robot* owner,unsigned halfPixelCount) :
		zbuffer(halfPixelCount * 2),
		image(halfPixelCount * 2),
//		cam0(owner, Point(0,0), 0,  M_PI_2, M_PI_2, halfPixelCount),
//		cam1(owner, Point(0,0), 0, -M_PI_2, M_PI_2, halfPixelCount)
		cam0(owner, Point(0,0), 0,  -M_PI_2 , M_PI_2, halfPixelCount),
		cam1(owner, Point(0,0), 0, M_PI_2, M_PI_2, halfPixelCount)
	{
// 	CircularCam::CircularCam(Robot *owner, Vector pos, double height, double orientation, double fieldOfView, unsigned pixelCount) :
		this->r = std::numeric_limits<double>::max();
		this->owner = owner;
	}

	void FullCircleCamera::objectStep(double dt, PhysicalObject *po, World *w)
	{
		cam0.objectStep(dt,po,w);
		cam1.objectStep(dt,po,w);
	}

	void FullCircleCamera::init()
	{
		cam0.init();
		cam1.init();
	}
	
	void FullCircleCamera::wallsStep(World *w)
	{
		cam0.wallsStep(w);
		cam1.wallsStep(w);
	}
	
	void FullCircleCamera::finalize(double dt)
	{
		cam0.finalize(dt);
		cam1.finalize(dt);
		size_t camPixelCount = cam0.zbuffer.size();
		std::copy(&cam0.zbuffer[0], &cam0.zbuffer[camPixelCount], &zbuffer[0]);
		std::copy(&cam1.zbuffer[0], &cam1.zbuffer[camPixelCount], &zbuffer[camPixelCount]);
		std::copy(&cam0.image[0], &cam0.image[camPixelCount], &image[0]);
		std::copy(&cam1.image[0], &cam1.image[camPixelCount], &image[camPixelCount]);
	}
	
	void FullCircleCamera::setFogConditions(bool useFog, double density, An::Color threshold)
	{
		cam0.useFog = useFog;
		cam0.fogDensity = density;
		cam0.lightThreshold = threshold;
		cam1.useFog = useFog;
		cam1.fogDensity = density;
		cam1.lightThreshold = threshold;
	}
	
	void FullCircleCamera::setRange(double range)
	{
		this->r = range;
		owner->sortLocalInteractions();
	}
	
	void FullCircleCamera::setPixelOperationFunctor(PixelOperationFunctor *pixelOperationFunctor)
	{
		cam0.pixelOperation = pixelOperationFunctor;
		cam1.pixelOperation = pixelOperationFunctor;
	}	

	void FullCircleCamera::setHeight(double height)
	{
		cam0.setHeight(height);
		cam1.setHeight(height);
	}
}
