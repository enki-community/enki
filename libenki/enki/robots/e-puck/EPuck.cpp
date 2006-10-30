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

#include "EPuck.h"
#include <limits.h>

/*! \file EPuck.cpp
\brief Implementation of the E-Puck robot
*/
namespace Enki
{
	//! Calculate the signal strength as a function of the distance.
	/*! The nearer we are, the higher the sensor activation.
	This model is very simple and not very good but sufficient for simple demonstration.
	\ingroup responsefunctor
	*/
	struct EPuckIRSensorModel : public SensorResponseFunctor
	{
		virtual double operator()(double dist, const Color &color)
		{
			if (dist<0.5)
				dist=-440*dist+3000;
			else if (dist>=0.5 && dist<=9)
				dist=4526*exp(-0.9994*dist);
			else
				dist=random.getRange(20.0);
			
			dist*=(0.97+random.getRange(0.06));
			
			
			return dist;
		}
				
	};
	
	
	//! We use only one ray per sensor for the e-puck.
	EPuckIRSensorModel epuckIRSensorModel;
	//! Pointer to sensor model for e-puck, one value C array.
	SensorResponseFunctor *epuckIRSensorModelPtr = &epuckIRSensorModel;
	
	EPuck::EPuck(unsigned capabilities) :
		infraredSensor0(this, Vector(3.0, -0.9),  0.0, -4*M_PI/45.0, 12, 0, 1, &epuckIRSensorModelPtr),   // IR 0 (Front right)    height should be 2.5 but all the other objects have a lower value so they are not seen...
		infraredSensor1(this, Vector(2.6, -2.6),  0.0, -M_PI/4.0, 12, 0, 1, &epuckIRSensorModelPtr),   // IR 1 (Half front right)
		infraredSensor2(this, Vector(0.0, -3.3),  0.0, -M_PI/2.0,      12, 0, 1, &epuckIRSensorModelPtr),     // IR 2 (Right side)
		infraredSensor3(this, Vector(-2.8, -1.7), 0.0, -5*M_PI/6.0,      12, 0, 1, &epuckIRSensorModelPtr),   // IR 3 (Back Right)
		infraredSensor4(this, Vector(-2.8, 1.7), 0.0, 5*M_PI/6.0 ,12, 0, 1, &epuckIRSensorModelPtr),  // IR 4 (Back left)
		infraredSensor5(this, Vector(0, 3.3), 0.0, M_PI/2.0,12, 0, 1, &epuckIRSensorModelPtr),     // IR 5 (Left side)
		infraredSensor6(this, Vector(2.6, 2.6), 0.0, M_PI/4.0,  12, 0, 1, &epuckIRSensorModelPtr),  // IR 6 (Half front left)
		infraredSensor7(this, Vector(3.0, 0.9), 0.0, 4*M_PI/45.0,  12, 0, 1, &epuckIRSensorModelPtr),    // IR 7 (Front left)
		camera(this, Vector(3.7, 0.0), 0.0, 0.0, M_PI/6.0, 60),     // height should be 2.2
		bluetooth(NULL)
	{
			oldAngle=angle;
			reflection=1.0;   // 0.6 would be more realistic
			mass = 152;
			height = 4.7;
			r = 3.7;
			collisionAngularFrictionFactor = 0.7;
			viscousFrictionTau = 0.5;
			viscousMomentFrictionTau = 0;
			
			if (capabilities & CAPABILITY_BASIC_SENSORS)
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
			
			if (capabilities & CAPABILITY_BLUETOOTH)
			{
				bluetooth = new Bluetooth(this,1000,7,100,100,random.get()%UINT_MAX);
				addGlobalInteraction(bluetooth);
			}
			leftSpeed = 0;
			rightSpeed = 0;
			
			leftEncoder = rightEncoder = 0;
			leftSpeed = rightSpeed = 0;
	}
	
	EPuck::~EPuck()
	{
		if (bluetooth)
			delete bluetooth;
	}
	
	void EPuck::step(double dt) 
	{
		// E-Puck dist between wheels are 5.1 cm
		double wheelDist = 5.1;
		
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
	
	void EPuck::resetEncoders()
	{
		oldPos = pos;
		oldAngle = angle;
		leftEncoder = rightEncoder = 0.0;
	}
	
	void EPuck::setLedRing(bool status)
	{
		color = status ? Color::red : Color::black;
	}
}

