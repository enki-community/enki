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

#include "IRSensor.h"
#include <assert.h>
#include <iostream>
#include <sstream>
#include <limits>

/*!	\file IRSensor.cpp
	\brief Implementation of the generic infrared sensor
*/

namespace Enki
{
	IRSensor::IRSensor(Robot *owner, Vector pos, double height, double orientation, 
					   double range, double aperture, unsigned rayCount, 
					   SensorResponseFunctor **sensorResponseKernel)
	{
		assert(owner);
		assert(sensorResponseKernel);
		
		this->owner = owner;
		this->pos = pos;
		this->height = height;
		this->orientation = orientation;
		this->range = range;
		this->aperture = aperture;
		this->rayCount = rayCount;
		rayValues.resize(rayCount);
		rayColors.resize(rayCount);
		rayAngles.resize(rayCount);
		absRayAngles.resize(rayCount);
		// compute ray orientation
		if (rayCount == 1)
			rayAngles[0] = 0;
		else 
			for (size_t i = 0; i<rayCount; i++)
				rayAngles[i] = - aperture + (i*2.0*aperture)/(rayCount-1.0);
		this->sensorResponseKernel.resize(rayCount);
		std::copy(sensorResponseKernel, sensorResponseKernel+rayCount, 
				  &this->sensorResponseKernel[0]);
		// calculate interaction radius, which is measured from center of bot
		this->r = sqrt(pos.norm2()+range*range-2*pos.norm()*range*cos(M_PI-orientation+pos.angle()));
		// calculate the smartRadius
		if (rayCount == 1)
			this->smartRadius = range/2.0;
		else
			this->smartRadius = range*sqrt(1.25-cos(aperture));
		// calculate relative position for center of central ray
		this->smartPos = Point (range/2*cos(orientation), range/2*sin(orientation));
		// no activation util first loop
		finalValue = 0;
	}

	void IRSensor::init(double dt, World* w)
	{
		// fill initial values with very large value; will be replaced if smaller distance is found
		std::fill(&rayValues[0], &rayValues[rayCount], HUGE_VAL);
		std::fill(&rayColors[0], &rayColors[rayCount], Color::white);
		
		// compute absolute position and orientation
		const Matrix22 rot(owner->angle);
		absPos = owner->pos + rot * pos;
		absOrientation = owner->angle + orientation;
		// compute correct absolute angles
		for (size_t i = 0; i<rayCount; i++)
			absRayAngles[i] = absOrientation + rayAngles[i];
		// calculate current position of center of central ray
		absSmartPos = rot * smartPos + absPos;
	}

	// Detect collision with a physical object's bounding polygon
	// Cyrus & Beck line/polygon intersection algorithm
	//   adapted by yvan.bourquin@epfl.ch from the
	//   code from http://softsurfer.com/ (by Dan Sunday)
	// Input: rayAngle: angle of ray segment to intersect with polygon
	// Input: p: polygon
	//   Note: The polygon MUST be convex and have vertices oriented counterclockwise (ccw).
	// This code does not check for and verify these conditions.
	// Return: distance to shortest intersection point
	//   or HUGE_VAL if there's no intersection
	double IRSensor::distanceToPolygon(double rayAngle, const Polygone &p) const 
	{
		// compute ray segment in global coordinates
		Point absEnd = absPos + Vector(cos(rayAngle), sin(rayAngle)) * range;
		Segment ray(absPos.x, absPos.y, absEnd.x, absEnd.y);

		const int n = p.size();         // number of points in the polygon
		double tE = 0.0;          // the maximum entering segment parameter
		double tL = 1.0;          // the minimum leaving segment parameter
		double t, N, D;           // intersect parameter t = N / D
		Vector dS(ray.b - ray.a); // the segment direction vector

		for (int i = 0; i < n; i++)     			// process polygon edge V[i]V[i+1] 
		{
			Vector e(p[i == n-1 ? 0: i + 1] - p[i]);	// edge vector
			N = e.cross(ray.a - p[i]);			// = -dot(ne, S.P0-V[i])
			D = -e.cross(dS);				// = dot(ne, dS)
			
			// S is nearly parallel to this edge
			if (fabs(D) < 0.00000001)
			{
				if (N < 0)				// P0 is outside this edge, so
					return HUGE_VAL;	// S is outside the polygon
				else					// S cannot cross this edge, so
					continue;			// ignore this edge
			}

			t = N / D;
			// segment S is entering across this edge
			if (D < 0)
			{
				// new max tE
				if (t > tE)
				{
					tE = t;
					// S enters after leaving polygon
					if (tE > tL)
						return HUGE_VAL;
				}
 			}
			// segment S is leaving across this edge
			else
			{
				// new min tL
				if (t < tL)
				{
					tL = t;
					// S leaves before entering polygon
					if (tL < tE)
						return HUGE_VAL;
				}
			}
		}

		// tE <= tL implies that there is a valid intersection subsegment
		// ray.a + tE * dS = point where S enters polygon
		// ray.a + tL * dS = point where S leaves polygon
  		return (dS * tE).norm();
	}


	// robot bounding circle overlaps with po
	// each sensor is composed of n rays
	// modified by yvan.bourquin@epfl.ch to take into account the exact bounding surface
	void IRSensor::objectStep (double dt, World *w, PhysicalObject *po)
	{
		// if we see over the object get out of here
		if (height > po->getHeight())
			return;
		
		const double radius = po->getRadius();
		const double irReflectiveness = po->getInfraredReflectiveness();
		const Color& color = po->getColor();

		// if dist from center point of rays to obj is bigger than sum of obj radii, don't bother
		const Vector v = po->pos-absSmartPos;
		const double radiusSum = radius + smartRadius*irReflectiveness;
		if (v.norm2() > (radiusSum * radiusSum))
			return;

		// Vector from sensor to object bounding circle center
		const Vector v1 = po->pos-absPos;
		// Radius squared of object
		const double r2 = radius * radius;
		// The number of rays
		
		if (po->isCylindric())
		{
			// Calculate distance for each ray...
			for (size_t i = 0; i<rayCount; i++)
			{
				double dist = HUGE_VAL;
				// angle between sensor ray and v1
				const double myAngle = absRayAngles[i] - v1.angle();
				const double sine = sin(myAngle);
				// normal distance of bounding circle center to sensor ray
				const double distsc2 = v1.norm2() * (sine * sine)*(1.0*irReflectiveness);
				
				// if there is an intersection with the object's bounding circle
				if (distsc2 < r2)
				{
					// compute distance of intersection with bounding circle
					dist = (sqrt(v1.norm2()-distsc2) - sqrt(r2-distsc2))*(1.0/irReflectiveness);
					if (dist<0)
						dist = 0;
					
					// if we have a smaller distance than the initial one, replace it
					if (dist < rayValues[i]*(1.0/irReflectiveness))
					{
						rayValues[i] = dist*(1.0/irReflectiveness);
						// FIXME: we do not support textures here
						rayColors[i] = color;
					}
				}
			}
		}
		else
		{
			// Calculate distance for each ray...
			for (size_t i = 0; i<rayCount; i++)
			{
				double dist = HUGE_VAL;
				// angle between sensor ray and v1
				const double myAngle = absRayAngles[i] - v1.angle();
				const double sine = sin(myAngle);
				// normal distance of bounding circle center to sensor ray
				const double distsc2 = v1.norm2() * (sine * sine)*(1.0*irReflectiveness);
				
				// if there is an intersection with the object's bounding circle
				if (distsc2 < r2)
				{
					// iterate over all shapes
					for (PhysicalObject::Hull::const_iterator it = po->getHull().begin(); it != po->getHull().end(); ++it)
					{
						if (height > it->getHeight())
							continue;
						
						// check intersection with polygon
						dist = distanceToPolygon(absRayAngles[i], it->getTransformedShape())*(1.0/irReflectiveness);
						
						// if we have a smaller distance than the initial one, replace it
						if (dist < rayValues[i]*(1.0/irReflectiveness))
						{
							rayValues[i] = dist*(1.0/irReflectiveness);
							// FIXME: we do not support textures here
							rayColors[i] = color;
						}
					}
				}
			}
		}
	}

	void IRSensor::wallsStep (double dt, World* w)
	{
		switch (w->wallsType)
		{
			case World::WALLS_SQUARE:
			{
				// if radius from center point of rays is not touching walls, don't bother
				if ((absSmartPos.x-smartRadius>0) && (absSmartPos.y-smartRadius>0) && (absSmartPos.x+smartRadius<w->w) && (absSmartPos.y+smartRadius<w->h))
					return;
		
				// if sensor is inside a wall distance is 0
				if ((absPos.x<0) || (absPos.x>w->w) || (absPos.y<0) || (absPos.y>w->h))
				{
					std::fill(&rayValues[0], &rayValues[rayCount], 0);
					std::fill(&rayColors[0], &rayColors[rayCount], Color::white);
					return;
				}
		
				for (size_t i = 0; i < rayCount; i++)
				{
					const Vector rayDir(cos(absRayAngles[i]), sin(absRayAngles[i]));
					
					// the absolute position of the sensor ray's end point
					const Point absRayEndPoint = absPos+rayDir*range;
					double candidate0 = HUGE_VAL; //infinity
					double candidate1 = HUGE_VAL;
					double newDist;
					
					// we have a candidate if our sensor sticks out into the left wall
					if (absRayEndPoint.x < 0)
						candidate0 = -absPos.x / (absRayEndPoint.x-absPos.x); 	// idea: a/b = c/d;
					// or the right wall
					else if (absRayEndPoint.x > w->w)
						candidate0 = (w->w-absPos.x) / (absRayEndPoint.x-absPos.x);
					// or the bottom wall
					if (absRayEndPoint.y < 0) 
						candidate1 = -absPos.y / (absRayEndPoint.y-absPos.y);
					// or the top wall
					else if (absRayEndPoint.y > w->h) 
						candidate1 = (w->h-absPos.y) / (absRayEndPoint.y-absPos.y);
		
					newDist = std::min(candidate0, candidate1);
					newDist *= range;
		
					// if we have a smaller distance than the initial one, replace it
					if (newDist < rayValues[i])
					{
						rayValues[i] = newDist;
						rayColors[i] = w->wallsColor;
					}
				}
			}
			break;
			
			case World::WALLS_CIRCULAR:
			{
				// TODO
			}
			break;
			
			default:
			break;
		}
	}
	
	// we combine all the sensor values
	void IRSensor::finalize(double dt, World* w)
	{
		finalValue = 0;
		for (size_t i = 0; i<rayCount; i++)
		{
			// we combine all the rays using the sensorModel
			finalValue += (*sensorResponseKernel[i])(rayValues[i], rayColors[i]);
		}
		// we apply final proportional (+/-7%) noise
		//FIXME: user-specified/controlled noise!	
		finalValue *= (.93 + random.getRange(.14));
	}
}

