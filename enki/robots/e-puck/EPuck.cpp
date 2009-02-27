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

#include "EPuck.h"
#include <algorithm>
#include <functional>
#include <climits>

/*! \file EPuck.cpp
\brief Implementation of the E-Puck robot
*/
namespace Enki
{
	using namespace std;
	
	EPuckScannerTurret::EPuckScannerTurret(Robot *owner, double height, unsigned halfPixelCount) :
		OmniCam(owner, height, halfPixelCount),
		scan(halfPixelCount * 2)
	{
	}
	
	void EPuckScannerTurret::finalize(double dt, World* w)
	{
		OmniCam::finalize(dt, w);
		
		// apply sensor response
		const double a1 =        1116;
		const double b1 =       56.92;
		const double c1 =       26.26;
		const double a2 =       780.9;
		const double b2 =       73.26;
		const double c2 =       76.33;
		const double a3 =  3.915e+016;
		const double b3 = -1.908e+004;
		const double c3 =        3433;
		
		assert(scan.size() == zbuffer.size());
		
		for (size_t i = 0; i < zbuffer.size(); i++)
		{
			// calibration was done in mm, convert to cm
			double x = sqrt(zbuffer[i]) * 10;
			size_t destIndex = ((scan.size()/2) -1 + scan.size() - i) % scan.size();
			scan[destIndex] = a1*exp(-((x-b1)/c1)*((x-b1)/c1)) + a2*exp(-((x-b2)/c2)*((x-b2)/c2)) + a3*exp(-((x-b3)/c3)*((x-b3)/c3));
		}
	}
	
	
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
				dist = -440*dist+2965.98144;
			else
				dist = 4526*exp(-0.9994*dist);
			
			dist *= (0.97 + random.getRange(0.06));
			dist += random.getRange(20.0);
			
			return dist;
		}
	};
	
	
	//! We use only one ray per sensor for the e-puck.
	EPuckIRSensorModel epuckIRSensorModel;
	//! Pointer to sensor model for e-puck, one value C array.
	SensorResponseFunctor *epuckIRSensorModelPtr = &epuckIRSensorModel;
	
	EPuck::EPuck(unsigned capabilities) :
		DifferentialWheeled(5.1, 12.8, 0.05),
		
		infraredSensor0(this, Vector(3.0, -0.9),  2.5, -4*M_PI/45.0, 12, 0, 1, &epuckIRSensorModelPtr), infraredSensor1(this, Vector(2.6, -2.6),  2.5, -M_PI/4.0,    12, 0, 1, &epuckIRSensorModelPtr),   infraredSensor2(this, Vector(0.0, -3.3),  2.5, -M_PI/2.0,    12, 0, 1, &epuckIRSensorModelPtr),
		infraredSensor3(this, Vector(-2.8, -1.7), 2.5, -5*M_PI/6.0,  12, 0, 1, &epuckIRSensorModelPtr),
		infraredSensor4(this, Vector(-2.8, 1.7),  2.5, 5*M_PI/6.0,   12, 0, 1, &epuckIRSensorModelPtr),
		infraredSensor5(this, Vector(0, 3.3),     2.5, M_PI/2.0,     12, 0, 1, &epuckIRSensorModelPtr),
		infraredSensor6(this, Vector(2.6, 2.6),   2.5, M_PI/4.0,     12, 0, 1, &epuckIRSensorModelPtr),
		infraredSensor7(this, Vector(3.0, 0.9),   2.5, 4*M_PI/45.0,  12, 0, 1, &epuckIRSensorModelPtr),
		camera(this, Vector(3.7, 0.0), 2.2, 0.0, M_PI/6.0, 60),
		scannerTurret(this, 7.2, 32),
		bluetooth(NULL)
	{
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
		
		if (capabilities & CAPABILITY_SCANNER_TURRET)
		{
			addLocalInteraction(&scannerTurret);
		}
		
		if (capabilities & CAPABILITY_BLUETOOTH)
		{
			bluetooth = new Bluetooth(this,1000,7,100,100,random.get()%UINT_MAX);
			addGlobalInteraction(bluetooth);
		}
		
		//staticFrictionThreshold = 0.5;
		dryFrictionCoefficient = 0.25;
		dryFrictionCoefficient = 2.5;
		
		setCylindric(3.7, 4.7, 152);
		setColor(Color(0, 0.7, 0));
		// 0.6 of infraredReflectiveness would be more realistic instead of the default 1
	}
	
	EPuck::~EPuck()
	{
		if (bluetooth)
			delete bluetooth;
	}
	
	void EPuck::setLedRing(bool status)
	{
		setColor(status ? Color::red : Color(0, 0.7, 0));
	}
}

