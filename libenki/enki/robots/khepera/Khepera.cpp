/*
    Copyright (C) 1999-2005 Stephane Magnenat <nct@ysagoon.com>
    Copyright (C) 2004-2005 Antoine Beyeler <antoine.beyeler@epfl.ch>
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

#include "Khepera.h"

/*! \file Khepera.cpp
	\brief Implementation of the Khepera robot
*/
namespace Enki
{
	using namespace An;
	
	//! Calculate the signal strength as a function of the distance.
	/*! The nearer we are, the higher the sensor activation.
		This model is very simple and not very good but sufficient for simple demonstration.
		\ingroup responsefunctor
	*/
	struct KheperaIRSensorModel : public SensorResponseFunctor
	{
		virtual double operator()(double dist, const An::Color &color)
		{
			dist -= 1.0;
			if(dist < 0.0)
				dist = 0.0;
			// add proportional noise; +/-3% and fit
			dist *= (0.97 + random.getRange(0.06));
			return 1100.0 * exp(-dist / 2.5);
		}
	};
	/*	double KheperaIRSensorModel(double dist)
	{
		// add proportional noise; +/-3% and fit
		dist *= (0.97 + random.getRange(0.06));
		return 1100.0 * exp(-dist / 2.5);
	}*/

	
	//! We use only one ray per sensor for the Khepera.
	KheperaIRSensorModel kheperaIRSensorModel;
	//! Pointer to sensor model for Khepera, one value C array.
	SensorResponseFunctor *kheperaIRSensorModelPtr = &kheperaIRSensorModel;
	
	/*
	infraredSensor0(this, Vector(1, 1.5),    1.8, M_PI/2, 10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModelmake),
		infraredSensor1(this, Vector(2, 2),      1.8, M_PI/4, 10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModel),
		infraredSensor2(this, Vector(1.6, 0.6),  1.8, 0,       10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModel),
		infraredSensor3(this, Vector(1.6, -0.6), 1.8, 0,       10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModel),
		infraredSensor4(this, Vector(2, -2),     1.8, -M_PI/4,  10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModel),
		infraredSensor5(this, Vector(1, -1.5),   1.8, -M_PI/2,  10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModel),
		infraredSensor6(this, Vector(-1.5, -1),  1.8, -M_PI,   10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModel),
		infraredSensor7(this, Vector(-1.5, 1),   1.8, -M_PI,   10, 0, 1, kheperaRayCombinationKernel, KheperaIRSensorModel),

	*/
	
	Khepera::Khepera(unsigned capabilities) :
		infraredSensor0(this, Vector(1.0, 1.5),  1.8, M_PI/2, 10, 0, 1, &kheperaIRSensorModelPtr),
		infraredSensor1(this, Vector(1.3, 1.3),  1.8, M_PI/4, 10, 0, 1, &kheperaIRSensorModelPtr),
		infraredSensor2(this, Vector(1.6, 0.6),  1.8, 0,      10, 0, 1, &kheperaIRSensorModelPtr),
		infraredSensor3(this, Vector(1.6, -0.6), 1.8, 0,      10, 0, 1, &kheperaIRSensorModelPtr),
		infraredSensor4(this, Vector(1.3, -1.3), 1.8, -M_PI/4,10, 0, 1, &kheperaIRSensorModelPtr),
		infraredSensor5(this, Vector(1.0, -1.5), 1.8, -M_PI/2,10, 0, 1, &kheperaIRSensorModelPtr),
		infraredSensor6(this, Vector(-1.5, -1.0),1.8, -M_PI,  10, 0, 1, &kheperaIRSensorModelPtr),
		infraredSensor7(this, Vector(-1.5, 1.0), 1.8, -M_PI,  10, 0, 1, &kheperaIRSensorModelPtr),
		camera(this, Vector(0, 0), 0, 0.0, M_PI/4, 50)
	{
		mass = 80;
		height = 3;
		r = 2.6;
		collisionAngularFrictionFactor = 0.7;
		viscousFrictionTau = 0.5;
		viscousMomentFrictionTau = 0;
		
		if (capabilities & CAPABILITIY_BASIC_SENSORS)
		{
			addLocalInteraction(&infraredSensor0);
			addLocalInteraction(&infraredSensor1);
			addLocalInteraction(&infraredSensor2);
			addLocalInteraction(&infraredSensor3);
			addLocalInteraction(&infraredSensor4);
			addLocalInteraction(&infraredSensor5);
			addLocalInteraction(&infraredSensor6);
			addLocalInteraction(&infraredSensor7);
		}
		
		if (capabilities & CAPABILITY_CAMERA)
		{
			addLocalInteraction(&camera);
		}
		
		leftEncoder = rightEncoder = 0;
		leftSpeed = rightSpeed = 0;
	}
	
	void Khepera::step(double dt) 
	{
		// Khepera dist between wheels are 5.2 cm
		double wheelDist = 5.2;

		// +/- 5% motor noise
		double realRightSpeed = rightSpeed * (0.95 + random.getRange(0.1));
		double realLeftSpeed = leftSpeed * (0.95 + random.getRange(0.1));
		
		// speeds, according to Prof. Siegwart class material
		double forwardSpeed = (realRightSpeed + realLeftSpeed) * 0.5;
		angSpeed += (realRightSpeed-realLeftSpeed) / wheelDist;
		speed = Vector(
					forwardSpeed * cos(angle + angSpeed * dt * 0.5),
					forwardSpeed * sin(angle + angSpeed * dt * 0.5));
		
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
		if(fabs(angle-oldAngle) > M_PI)
			meanAngle += M_PI;	// this is not normalized but we dont care
		norm = norm * cos(meanAngle - travelAngle);
		
		// Compute encoders
		double angleDiff = normalizeAngle(angle - oldAngle);
		leftEncoder = (norm - wheelDist * angleDiff * 0.5) / dt;
		rightEncoder = (norm + wheelDist * angleDiff * 0.5) / dt;
		
		// Save values for next step.
		oldPos = pos;
		oldAngle = angle;
		
		// handle physic
		PhysicalObject::step(dt);
	}
	
	void Khepera::resetEncoders()
	{
		oldPos = pos;
		oldAngle = angle;
		leftEncoder = rightEncoder = 0.0;
	}
}

