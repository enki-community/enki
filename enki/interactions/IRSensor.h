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

#ifndef __ENKI_IRSENSOR_H
#define __ENKI_IRSENSOR_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>

#include <valarray>
#undef min

/*!	\file IRSensor.h
	\brief Header of the generic infrared sensor
*/

namespace Enki
{
	/*!	\defgroup responsefunctor Response function classes
	Different response functions for different sensors
	*/
	
	//! Functor for the sensor response
	/*! \ingroup responsefunctor */
	struct SensorResponseFunctor
	{
		//! Virtual destructor, do nothing
		virtual ~SensorResponseFunctor() {}
		//! Return the response for a given distance and object color
		virtual double operator()(double, const Color &) = 0;
	};
	
	//! A generic infrared sensor
	/*! \ingroup interaction */
	class IRSensor : public LocalInteraction
	{
	protected:
		//! Absolute position in the world, updated on init()
		Vector absPos;
		//! Absolute orientation in the world, updated on init()
		double absOrientation;
		//! Relative position on the robot
		Vector pos;
		//! Height above ground, the sensor will not see any object of smaller height
		double height;
		//! Relative orientation on the robot
		double orientation;
		//! Actual detection range
		double range;
		//! Aperture angle
		double aperture;
		//! Number of rays used, each ray has an aperture of aperture/rayCount to the next one. Rays are assembled from right to left (i.e. counterclockwise)
		unsigned rayCount;
		//! Array of size rayCount used to compute the response of different rays using implementations of SensorResponseFunctor functors.
		/*!
		SensorResponseFunctors translate the distances stored in the rayValues[] into actual sensor activations. Distances are in cm. For negative distance values, i.e. a sensor inside an object, wall, etc., the value of the sensor response function at distance 0 will be used. If a ray fails to touch the object, the distance returned will be HUGE_VAL; the sensor response function should typically return a 0 sensor activation for this case. An appropriate noise model (if realistic modelling is desired) should be included in the sensor response function.
		*/
		std::valarray<SensorResponseFunctor *> sensorResponseKernel;
		
		//! Radius for the smallest circle enclosing all rays
		double smartRadius;
		//! Current position of the center of the smartRadius, i.e. center of the smallest circle enclosing all rays in relative (robot) coordinates
		Point smartPos;
		//! Current position of the center of the smartRadius in absolute (world) coordinates, updated on init()
		Vector absSmartPos;
		//! Temporary ray values containing the lowest distance found up to now
		std::valarray<double> rayValues;
		//! Temporary ray values containing the color of the closest segment up to now
		std::valarray<Color> rayColors;
		//! The angle for each ray relative to the sensor orientation in relative (robot) coordinates
		std::valarray<double> rayAngles;
		//! The angle for each ray relative to the sensor orientation in absolute (world) coordinates
		std::valarray<double> absRayAngles;
	
	public:
		//! Final sensor value
		double finalValue;
		
	public:
		//! Constructor
		//! e.g.: "left(this, Vector (0.95, 0.95), 1.2, M_PI/4, 2.5, M_PI/6, 3, AliceIRNormalSensorModel)"
		//! i.e. for left: position on bot is x=0.95, y=0.95 from center of bot, height of sensor is 1.2, looking 45 deg to the left, +/-30 deg opening angle, 3cm max sensor range, a raycount of 3, SensorResponseFunctor in AliceIRNormalSensorModel array
		IRSensor(Robot *owner, Vector pos, double height, double orientation, double range, double aperture, unsigned rayCount, SensorResponseFunctor **sensorResponseKernel);
		//! Reset distance values
		void init(double dt, World* w);
		//! Check for all potential intersections using smartRadius of sensor and calculate and find closest distance for each ray.
		void objectStep(double dt, World *w, PhysicalObject *po);
		//! Separated from objectStep because it is much simpler. 
		void wallsStep(double dt, World* w);
		//! Applies the SensorResponseFunction to each ray and combines all rays using weights defined in the rayCombinationKernel.
		void finalize(double dt, World* w);
		
		//! Return the absolute position of the IR sensor, updated at each time step on init()
		Point getAbsolutePosition(void) { return absPos; }
		//! Return the absolute orientation of the IR sensor, updated at each time step on init()
		double getAbsoluteOrientation(void) { return absOrientation; }
		//! Return the number of rays
		unsigned getRayCount(void) { return rayCount; }
		//! Return the aperture of the sensor
		double getAperture(void) { return aperture; }
		//! Return the range of the sensor
		double getRange(void) { return range; }
		//! Return the radius for the smallest circle enclosing all rays
		double getSmartRadius(void) { return smartRadius; }
		//! Return current position of the center of the smartRadius, i.e. center of the smallest circle enclosing all rays in relative (robot) coordinates
		Point getAbsSmartPos(void) { return absSmartPos; }
		//! Return the distance of the middle ray; this may be usefull for inaccurate simulations
		double getDist(void) { double val = rayValues[rayValues.size() / 2]; return std::min(val, range); }
	
	private:
		//! Returns distance to PhysicalObject po for angle rayAngle.
		//! Note: The polygon MUST be convex and have vertices oriented counterclockwise (ccw). This code does not check for and verify these conditions. Returns distance to shortest intersection point or HUGE_VAL if there is no intersection
		double distanceToPolygon(double rayAngle, const Polygone &p) const;
	};
}

#endif
