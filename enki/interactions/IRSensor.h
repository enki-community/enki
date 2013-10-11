/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2013 Stephane Magnenat <stephane at magnenat dot net>
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
	//! A generic infrared sensor
	/*! \ingroup interaction 
	
	This sensor is based on a inverse square response function and three casted rays.
	
	During objectStep() and wallsStep() it casts the three rays,
	separated by an angle of 15 degrees. Distances are in cm. For negative distance values, i.e. a sensor inside an object, wall, etc., the value of the sensor response function at distance 0 will be used. If a ray fails to touch the object, the distance returned will be HUGE_VAL; the sensor response function will return a 0 sensor response for this case.
	
	Upon finalize(), it computes finalValue and finalDist.
	It does so first using the following equation for each ray:
	
		               m * (c - x0*x0)
		value = F(x) = ----------------
		               x*x - 2*x0*x + c
	
	where x is the distance to the obstacle.
	
	It then combines the three rays with this equation:
	
		finalValue = F(d_center) + F(d_left) + F(d_right) - 2*F(d_center*alpha)
	
	where d_R is the distance of ray R, and alpha is 1/cos(15 degrees).
	
	Finally, it computes the final distance using:
	
		finalDist = F-1(finalValue)
	
	where:
		                                       m
		F-1(v) = x0 + sqrt( (x0*x0-c) * ( 1 - --- ) )
		                                       v
	
	
	TODO
	SensorResponseFunctors translate the distances stored in the rayValues[] into actual sensor activations.  An appropriate noise model (if realistic modelling is desired) should be included in the sensor response function.
	 
	*/
	class IRSensor : public LocalInteraction
	{
	protected:
		//! Absolute position in the world, updated on init()
		Vector absPos;
		//! Absolute orientation in the world, updated on init()
		double absOrientation;
		//! Relative position on the robot
		const Vector pos;
		//! Height above ground, the sensor will not see any object of smaller height
		const double height;
		//! Relative orientation on the robot
		const double orientation;
		//! Actual detection range
		const double range;
		//! Aperture angle
		const double aperture;
		//! 1/cos(aperture)
		const double alpha;
		//! Number of rays used, each ray has an aperture of aperture/rayCount to the next one. Rays are assembled from right to left (i.e. counterclockwise)
		const unsigned rayCount;
		//! Maximum possible response value, might be inside the robot if x0<0, first parameter of response function
		const double m;
		//! Position of the maximum of response (might be negative, inside the robot), second parametere of response function
		const double x0;
		//! Third parameter of response function
		const double c;
		//! Standard deviation of Gaussian noise in the response space
		const double noiseSd;
		
		//! Radius for the smallest circle enclosing all rays
		double smartRadius;
		//! Current position of the center of the smartRadius, i.e. center of the smallest circle enclosing all rays in relative (robot) coordinates
		Point smartPos;
		//! Current position of the center of the smartRadius in absolute (world) coordinates, updated on init()
		Vector absSmartPos;
		//! Temporary ray values containing the lowest distance found up to now
		std::vector<double> rayDists;
		//! Temporary ray values containing the response value of the closest object found up to now
		std::vector<double> rayValues;
		//! The angle for each ray relative to the sensor orientation in relative (robot) coordinates
		std::vector<double> rayAngles;
		//! The angle for each ray relative to the sensor orientation in absolute (world) coordinates
		std::vector<double> absRayAngles;
	
		//! Final sensor value
		double finalValue;
		//! Final computed distance
		double finalDist;
		
	public:
		//! Constructor
		/*!
			\param owner robot which embeds this sensor
			\param pos relative position (x,y) on the robot
			\param height height above ground, the sensor will not see any object of smaller height
			\param orientation relative orientation on the robot
			\param range detection range, objects over this range will not be seen
			\param m maximum possible response value, might be inside the robot if x0<0, first parameter of response function
			\param x0 position of the maximum of response (might be negative, inside the robot), second parametere of response function
			\param c third parameter of response function
			\param noiseSd standard deviation of Gaussian noise in the response space
		*/
		IRSensor(Robot *owner, Vector pos, double height, double orientation, double range, double m, double x0, double c, double noiseSd = 0.);
		//! Reset distance values
		void init(double dt, World* w);
		//! Check for all potential intersections using smartRadius of sensor and calculate and find closest distance for each ray.
		void objectStep(double dt, World *w, PhysicalObject *po);
		//! Separated from objectStep because it is much simpler. 
		void wallsStep(double dt, World* w);
		//! Applies the SensorResponseFunction to each ray and combines all rays using weights defined in the rayCombinationKernel.
		void finalize(double dt, World* w);
		
		//! Return the final sensor value
		double getValue(void) const { return finalValue; }
		//! Return the distance through the inverse response of the final sensor value 
		double getDist(void) const { return finalDist; }
		//! Return the value of a ray
		double getRayValue(unsigned i) const { return rayValues.at(i); }
		//! Return the distance of a ray
		double getRayDist(unsigned i) const { return rayDists.at(i); }
		
		//! Return the absolute position of the IR sensor, updated at each time step on init()
		Point getAbsolutePosition(void) const { return absPos; }
		//! Return the absolute orientation of the IR sensor, updated at each time step on init()
		double getAbsoluteOrientation(void) const { return absOrientation; }
		//! Return the number of rays
		unsigned getRayCount(void) const { return rayCount; }
		//! Return the aperture of the sensor
		double getAperture(void) const { return aperture; }
		//! Return the range of the sensor
		double getRange(void) const { return range; }
		//! Return the radius for the smallest circle enclosing all rays
		double getSmartRadius(void) const { return smartRadius; }
		//! Return current position of the center of the smartRadius, i.e. center of the smallest circle enclosing all rays in relative (robot) coordinates
		Point getAbsSmartPos(void) const { return absSmartPos; }
		
	protected:
		//! If dist is smaller than current ray distance, update distance and response value
		void updateRay(size_t i, double dist);
		//! Return the response for a given distance
		double responseFunction(double x) const;
		//! Return the inverse response for a given distance
		double inverseResponseFunction(double v) const;
		//! Returns distance to PhysicalObject po for angle rayAngle.
		//! Note: The polygon MUST be convex and have vertices oriented counterclockwise (ccw). This code does not check for and verify these conditions. Returns distance to shortest intersection point or HUGE_VAL if there is no intersection
		double distanceToPolygon(double rayAngle, const Polygone &p) const;
	};
}

#endif
