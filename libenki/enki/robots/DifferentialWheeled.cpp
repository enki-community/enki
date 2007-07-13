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

#include "DifferentialWheeled.h"

/*! \file DifferentialWheeled.cpp
	\brief Implementation of the features of differential wheeled robots
*/
namespace Enki
{
	DifferentialWheeled::DifferentialWheeled(double distBetweenWheels, double noiseAmount) :
		distBetweenWheels(distBetweenWheels),
		noiseAmount(noiseAmount)
	{
		leftSpeed = rightSpeed = 0;
		resetEncoders();
	}
	
	void DifferentialWheeled::resetEncoders()
	{
		oldPos = pos;
		oldAngle = angle;
		leftEncoder = rightEncoder = 0.0;
		leftOdometry = rightOdometry = 0.0;
	}
	
	void DifferentialWheeled::step(double dt)
	{
		// +/- noiseAmout % of motor noise
		double baseFactor = 1 - noiseAmount;
		double noiseFactor = 2 * noiseAmount;
		double realRightSpeed = rightSpeed * (baseFactor + random.getRange(noiseFactor));
		double realLeftSpeed = leftSpeed * (baseFactor + random.getRange(noiseFactor));
		
		// speeds, according to Prof. Roland Siegwart class material
		double forwardSpeed = (realRightSpeed + realLeftSpeed) * 0.5;
		angSpeed += (realRightSpeed-realLeftSpeed) / distBetweenWheels;
		speed = Vector(
					forwardSpeed * cos(angle + angSpeed * dt * 0.5),
					forwardSpeed * sin(angle + angSpeed * dt * 0.5)
				);
		
		// PhysicalObject::step will actually move, and in next loop
		// care will be taken regarding collision. So we have to compute
		// the difference here.
		Vector posDiff = pos - oldPos;
		double norm = posDiff.norm();
		double travelAngle = posDiff.angle();
		
		// we let only the component of the norm that is in the direction
		// of the robot (the other component is the wheel sliding in the
		// perpendicular direction). we take the mean angle as direction.
		// angle and oldAngle are normalized, so we dont have problems here.
		double meanAngle = (angle + oldAngle) * 0.5;
		if (fabs(angle-oldAngle) > M_PI)
			meanAngle += M_PI;	// this is not normalized but we dont care
		norm = norm * cos(meanAngle - travelAngle);
		
		// Compute encoders
		double angleDiff = normalizeAngle(angle - oldAngle);
		leftEncoder = (norm - distBetweenWheels * angleDiff * 0.5) / dt;
		rightEncoder = (norm + distBetweenWheels * angleDiff * 0.5) / dt;
		leftOdometry += leftEncoder;
		rightOdometry += rightEncoder;
		
		// Save values for next step.
		oldPos = pos;
		oldAngle = angle;
		
		// handle physic
		PhysicalObject::step(dt);
	}
}

