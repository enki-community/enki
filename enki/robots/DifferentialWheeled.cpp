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
		noiseAmount(noiseAmount),
		cmdAngSpeed(0),
		cmdSpeed(0)
	{
		leftSpeed = rightSpeed = 0;
		resetEncoders();
	}
	
	void DifferentialWheeled::resetEncoders()
	{
		leftEncoder = rightEncoder = 0.0;
		leftOdometry = rightOdometry = 0.0;
	}
	
	void DifferentialWheeled::controlStep(double dt)
	{
		// +/- noiseAmout % of motor noise
		const double baseFactor = 1 - noiseAmount;
		const double noiseFactor = 2 * noiseAmount;
		
		const double realLeftSpeed = clamp(
			leftSpeed * (baseFactor + random.getRange(noiseFactor)),
			-maxSpeed,maxSpeed
		);
		const double realRightSpeed = clamp(
			rightSpeed * (baseFactor + random.getRange(noiseFactor)),
			-maxSpeed, maxSpeed
		);
		
		// set non slipping, override speed
		cmdSpeed = (realLeftSpeed + realRightSpeed) * 0.5;
		cmdAngSpeed = (realRightSpeed - realLeftSpeed) / distBetweenWheels;
		
		// Compute encoders
		leftEncoder = realLeftSpeed;
		rightEncoder = realRightSpeed;
		leftOdometry += leftEncoder * dt;
		rightOdometry += rightEncoder * dt;
		
		// Call parent
		Robot::controlStep(dt);
	}
	
	void DifferentialWheeled::applyForces(double dt)
	{
		const Vector cmdVelocity(
			cmdSpeed * cos(angle + angSpeed * dt * 0.5),
			cmdSpeed * sin(angle + angSpeed * dt * 0.5)
		);
		angSpeed = cmdAngSpeed;
		speed = cmdVelocity;
	}
}

