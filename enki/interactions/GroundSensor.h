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

#ifndef __ENKI_GROUND_SENSOR_H
#define __ENKI_GROUND_SENSOR_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>

/*!	\file GroundSensor.h
 \brief Header of the ground infrared sensor
 */

namespace Enki
{
	//! A ground infrared sensor
	/*! \ingroup interaction
	
	This sensor scans the intensity of a 2x2 cm square on the ground.
	It does 9x9 measurements, using a Gaussian model with a given spatialSd standard deviation,
	and a noiseSd Gaussian measurement error.
	
	The resulted value v is transformed into a noiseless value finalNoiseless:
	
		finalNoiseless = sigm(v - cFactor, sFactor) * mFactor + aFactor
	
	where sigm(x, s) = 1 / (1 + e^(-x * s))
	
	Which is then transformed into a noise finalValue by applying Gaussian noise with noiseSd standard deviation.
	 
	*/
	class GroundSensor : public LocalInteraction
	{
	protected:
		//! Absolute position in the world, updated on init()
		Vector absPos;
		//! Relative position on the robot
		const Vector pos;
		//! Center of the sigmoid
		const double cFactor;
		//! Multiplication factor for the argument of the sigmoid
		const double sFactor;
		//! Multiplicative factor applied after the sigmoid to compute finalValue
		const double mFactor;
		//! Additive factor applied after the sigmoid to compute finalValue
		const double aFactor;
		
		//! Standard deviation of Gaussian noise in the response space
		const double noiseSd;
		
		//! Pre-computed coefficient to filter ground image on a 2x2 cm square, with a 0.25 cm resolution
		double filter[9][9];
		
		//! Final sensor value
		double finalValue;
		
	public:
		//! Constructor
		/*!
		\param owner robot which embeds this sensor
		\param pos relative position (x,y) on the robot
		\param cFactor center of the sigmoid
		\param sFactor multiplication factor for the argument of the sigmoid
		\param mFactor multiplicative factor to compute finalValue
		\param aFactor additive factor to compute finalValue
		\param spatialSd standard deviation of the reading beam on the sensor on the ground
		\param noiseSd standard deviation of Gaussian noise in the response space
		*/
		GroundSensor(Robot *owner, Vector pos, double cFactor, double sFactor, double mFactor, double aFactor, double spatialSd = 0.4, double noiseSd = 0.);
		//! Compute absolute position
		void init(double dt, World* w);
		
		//! Reset intensity value
		//! Return the final sensor value
		double getValue(void) const { return finalValue; }
		
		//! Return the absolute position of the ground sensor, updated at each time step on init()
		Point getAbsolutePosition(void) const { return absPos; }
	};
}

#endif // __ENKI_GROUND_SENSOR_H
