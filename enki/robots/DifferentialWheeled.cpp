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

#include "DifferentialWheeled.h"

/*! \file DifferentialWheeled.cpp
	\brief Implementation of the features of differential wheeled robots
*/
namespace Enki
{
	template<typename T>
	T clamp(T v, T min, T max)
	{
		if (v < min)
			return min;
		else if (v > max)
			return max;
		else
			return v;
	}
	
	DifferentialWheeled::DifferentialWheeled(double distBetweenWheels, double maxSpeed, double noiseAmount) :
		distBetweenWheels(distBetweenWheels),
		maxSpeed(maxSpeed),
		noiseAmount(noiseAmount)
	{
		leftSpeed = rightSpeed = 0;
		resetEncoders();
	}
	
	void DifferentialWheeled::resetEncoders()
	{
		leftEncoder = rightEncoder = 0.0;
		leftOdometry = rightOdometry = 0.0;
	}
	
	static double sgn(double v)
	{
		if (v > 0)
			return 1;
		else if (v < 0)
			return -1;
		else
			return 0;
	}
	
	static double clamp(double v, double range)
	{
		if (v < -range)
			return -range;
		else if (v > range)
			return range;
		else
			return v;
	}
	
	void DifferentialWheeled::step(double dt)
	{
		// handle physic
		PhysicalObject::step(dt);
		
		// +/- noiseAmout % of motor noise
		double baseFactor = 1 - noiseAmount;
		double noiseFactor = 2 * noiseAmount;
		double realLeftSpeed = clamp(leftSpeed, -maxSpeed, maxSpeed);
		realLeftSpeed *= (baseFactor + random.getRange(noiseFactor));
		double realRightSpeed = clamp(rightSpeed, -maxSpeed, maxSpeed);
		realRightSpeed  *= (baseFactor + random.getRange(noiseFactor));
		
		/*// NOTE: if we seems to need a better model, have a look at the commented code, but the latter tuning for sure
		Vector force = ((oldSpeed - speed) / dt) * mass;
		if (force.norm2() > staticFrictionThreshold * staticFrictionThreshold)
		{*/
			// non slipping, override speed
			double forwardSpeed =  (realLeftSpeed + realRightSpeed) * 0.5;
			angSpeed = (realRightSpeed - realLeftSpeed) / distBetweenWheels;
			speed = Vector(
							forwardSpeed * cos(angle + angSpeed * dt * 0.5),
							forwardSpeed * sin(angle + angSpeed * dt * 0.5)
					);
		/*}
		else
		{
			// slipping, add speed difference
			
			// speeds of the contact points, we cannot transfer more speed that we do have
			double speedInDir = speed * Vector(cos(angle), sin(angle));
			double leftContactSpeed = clamp(speedInDir - distBetweenWheels * 0.5 * angSpeed - realLeftSpeed, abs(realLeftSpeed));
			double rightContactSpeed = clamp(speedInDir + distBetweenWheels * 0.5 * angSpeed - realRightSpeed, abs(realRightSpeed));
				
			// we use a non sleeping wheel model, and we directly affect pos
			// speeds, according to Prof. Roland Siegwart class material
			double forwardSpeed = - (leftContactSpeed + rightContactSpeed) * 0.5;
			angSpeed += (leftContactSpeed - rightContactSpeed) / distBetweenWheels;
			speed += Vector(
							forwardSpeed * cos(angle + angSpeed * dt * 0.5),
							forwardSpeed * sin(angle + angSpeed * dt * 0.5)
					);
		}
		
		oldSpeed = speed;*/
		
		// NOTE: affecting pos does not work as the speed is not transfered upon collision
		
		/*// We use a model of constant force in opposite direction than movement
		// This is very simple but the full model is way too complex and requires too many experimentation.
		// Note: even this simple model is bad
		double leftThrust= -sgn(leftContactSpeed) * contactPointThrust;
		double rightThrust = -sgn(rightContactSpeed) * contactPointThrust;
		double totalThrust = leftThrust + rightThrust;
		double diffThrust = rightThrust - leftThrust;
		acc += Vector(totalThrust * cos(angle), totalThrust * sin(angle)) / mass;
		angAcc += (diffThrust * distBetweenWheels * 0.5) / momentOfInertia;
		*/
		
		// Compute encoders
		leftEncoder = realLeftSpeed;
		rightEncoder = realRightSpeed;
		leftOdometry += leftEncoder * dt;
		rightOdometry += rightEncoder * dt;
	}
}

