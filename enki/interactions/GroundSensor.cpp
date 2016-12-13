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

#include "GroundSensor.h"

/*!	\file GroundSensor.cpp
	\brief Implementation of the ground infrared sensor
*/

namespace Enki
{
	using namespace std;
	
	GroundSensor::GroundSensor(Robot *owner, Vector pos, double cFactor, double sFactor, double mFactor, double aFactor, double spatialSd, double noiseSd):
		pos(pos),
		cFactor(cFactor),
		sFactor(sFactor),
		mFactor(mFactor),
		aFactor(aFactor),
		noiseSd(noiseSd)
	{
		assert(owner);
		this->owner = owner;
		// compute kernel up to a constant factor
		const double var(spatialSd * spatialSd);
		double sum(0);
		for (int i = 0; i < 9; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				const double x(double(i-4) / 4.);
				const double y(double(j-4) / 4.);
				filter[i][j] = exp(-(x * x + y * y) / (2. * var));
				sum += filter[i][j];
			}
		}
		// renormalize function
		for (int i = 0; i < 9; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				filter[i][j] /= sum; 
			}
		}
	}
	
	static double _sigm(double x, double s)
	{
		return 1. / (1. + exp(-x * s));
	}
	
	void GroundSensor::init(double dt, World* w)
	{
		// compute absolute position
		const Matrix22 rot(owner->angle);
		absPos = owner->pos + rot * pos;
		
		// compute sensor value on a gaussian filtered ground
		double v(0);
		for (int i = 0; i < 9; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				const double x(double(i-4) / 4.);
				const double y(double(j-4) / 4.);
				const double groundIntensity(w->getGroundColor(Point(absPos.x+x, absPos.y+y)).toGray());
				v += filter[i][j] * groundIntensity;
			}
		}
		
		// changing value to response space and adding Gaussian noise before returning value
		finalValue = gaussianRand(_sigm(v - cFactor, sFactor) * mFactor + aFactor, noiseSd);
	}
}
