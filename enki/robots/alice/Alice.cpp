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

#include "Alice.h"

/*! \file Alice.cpp
	\brief Implementation of the Alice robot
*/

namespace Enki
{
	 //In our case, the sensormodelfunction is of the form a*exp(-b*x)-c*exp(-d*x)+e, where e is a global offset, sometimes adjusted with respect to sensorOffsetNoise.
	
	//! The bounding surface of an Alice
	struct AliceBoundingSurface
	{
		//! The polygone containing the surface
		Polygone p;

		//! The constructor, create the physical shape of the alice
		AliceBoundingSurface()
		{
			// Alice physical shape. If you have alice with different shapes
			// or what to some particular bias in your simulation, you can change
			// this.
			
			// Basic
			p.push_back(Point(-1.25,-1));
			p.push_back(Point(1.25,-1));
			p.push_back(Point(1.25,1));
			p.push_back(Point(-1.25,1));
			
			/*
			// Snowplough
			p.push_back(Point(-1.25,-1));
			p.push_back(Point(1.05,-1));
			p.push_back(Point(1.26,0));
			p.push_back(Point(1.05,1));
			p.push_back(Point(-1.25,1));
			*//*
			// Double Snowplough
			p.push_back(Point(-1.15,-1));
			p.push_back(Point(1.15,-1));
			p.push_back(Point(1.25,0));
			p.push_back(Point(1.15,1));
			p.push_back(Point(-1.15,1));
			p.push_back(Point(-1.25,0));
			*//*
			// Cut-off Corners
			p.push_back(Point(-1.25,-1));
			p.push_back(Point(1.0,-1));
			p.push_back(Point(1.25,-.75));
			p.push_back(Point(1.25,.75));
			p.push_back(Point(1.0,1));
			p.push_back(Point(-1.25,1));
			*//*
			// Double-quadruple cut-off corners
			p.push_back(Point(-1.0,-1));
			p.push_back(Point(1.0,-1));
 			//p.push_back(Point(1.2,-.7));
			
			//p.push_back(Point(1.25 + (-0.1 + random.getRange(0.2)),-.1 - (random.getRange(0.6))));
			//p.push_back(Point(1.25 + (-0.1 + random.getRange(0.2)),.1 + (random.getRange(0.6))));

			p.push_back(Point(1.25 + (-1 + random.getRange(3)),-.1 - (random.getRange(0.6))));
			p.push_back(Point(1.25 + (-1 + random.getRange(3)),.1 + (random.getRange(0.6))));
			
			//p.push_back(Point(1.2,.7));
			p.push_back(Point(1.0,1));
			p.push_back(Point(-1.0,1));
			p.push_back(Point(-1.2,0));
			*/
 		}
	} aliceBoundingSurface;


	//! Calculate the signal strength as a function of the distance.
	/*! The nearer we are, the higher the sensor activation will be.
		\ingroup responsefunctor
	*/
	struct AliceIRNormalSensorModel : public SensorResponseFunctor
	{
		double mult; //!< multiplier coefficient for response computation
		double shift; //!< shift coefficients for response computation

		//! Constructor, store coefficients for response computation
		AliceIRNormalSensorModel(double mult, double shift)
		{
			this->mult = mult;
			this->shift = shift;
		}

		virtual double operator()(double dist, const Color &color)
		{
			// last term adjusts for half of overall offset so noise is +/-
			return shift + mult * (230.973 * exp (-1.49667*dist) - 108.396 * exp (-5.43925*dist) + 1.0 - 1.5);
		}
	};


	//! Calculate the signal strength as a function of the distance.
	/*! Similar to AliceIRNormalSensorModel, but slight shift of the curve for correct large disk pushing values
		The nearer we are, the higher the sensor activation will be.
		\ingroup responsefunctor
	*/
	struct AliceIRLeftSensorModel : public SensorResponseFunctor
	{
		double mult; //!< multiplier coefficient for response computation
		double shift; //!< shift coefficients for response computation

		//! Constructor, store coefficients for response computation
		AliceIRLeftSensorModel(double mult, double shift)
		{
			this->mult = mult;
			this->shift = shift;
		}

		virtual double operator()(double dist, const Color &color)
		{
			// last term adjusts for half of overall offset so noise is +/-
			// FIXME: shifted dist by a factor -.14 to get interlaced values right (around 50 for large object)
			return shift + mult * (230.973 * exp (-1.49667*(dist-.14)) - 108.396 * exp (-5.43925*(dist-.14)) + 1.0 - 1.5);
		}
	};


	//! Calculate the signal strength as a function of the distance.
	/*! Similar to AliceIRNormalSensorModel, but slight shift of the curve for correct large disk pushing values
		The nearer we are, the higher the sensor activation will be.
		\ingroup responsefunctor
	*/
	struct AliceIRRightSensorModel : public SensorResponseFunctor
	{
		double mult; //!< multiplier coefficient for response computation
		double shift; //!< shift coefficients for response computation

		//! Constructor, store coefficients for response computation
		AliceIRRightSensorModel(double mult, double shift)
		{
			this->mult = mult;
			this->shift = shift;
		}

		virtual double operator()(double dist, const Color &color)
		{
			// last term adjusts for half of overall offset so noise is +/-
			// FIXME: shifted dist by a factor -.14 to get interlaced values right (around 50 for large object)
			return shift + mult * (230.973 * exp (-1.49667*(dist-.14)) - 108.396 * exp (-5.43925*(dist-.14)) + 1.0 - 1.5);
		}
	};


	//! Calculate the signal strength as a function of the distance. 
	/*! Same as AliceIRNormalSensorModel, but works at twice the distance.
		The nearer we are, the higher the sensor activation.
		\ingroup responsefunctor
	*/
	struct AliceIRHighSensorModel : public SensorResponseFunctor
	{
		double mult; //!< multiplier coefficient for response computation
		double shift; //!< shift coefficients for response computation
		
		//! Constructor, store coefficients for response computation
		AliceIRHighSensorModel(double mult, double shift)
		{
			this->mult = mult;
			this->shift = shift;
		}

		virtual double operator()(double dist, const Color &color)
		{
			// high sensor reflection is a function of the object color (independent of the textures seen by the camera!)
			// if the other object is black and does not reflect well (e.g. Alices) we use the first function ...
			// FIXME: added .90 to have overall lower sensor values in simulation than reality for more reactivity in reality
			if (color==Color::black)
				return shift + mult * .90 * (119.695 * exp (-0.248*dist) - 86.276 * exp (-3.5*dist) + 0 - 3.5);
			// ... else we assume its color is white and use the second function (e.g. Walls)
			// we also add a linear (shift) and multiplicative (mult) offset calculated at initialisation of sensormodel	
			else 
				// FIXME: added a .1 to the dist. should get a better function here!
				assert (dist>=0);
				return shift + mult * .90 * (123.9 * exp (-0.39*(dist+.1)) - 123.572 * exp (-3.22*(dist+.1)) - .05 - 3.5);
		}
	};

	//! Response function which returns zero all the time.
	/*! \ingroup responsefunctor */
	struct ZeroSensorModel : public SensorResponseFunctor
	{
		//! Return zero
		virtual double operator()(double dist, const Color &color) { return 0; }
	};

	Alice::SensorModels::SensorModels()
	{
		// Each sensor has some specific noise in response
		frontleft[0] = new AliceIRLeftSensorModel(0.305098, random.getRange(3.0));
		frontleft[1] = new AliceIRLeftSensorModel(0.309804, random.getRange(3.0));
		frontleft[2] = new AliceIRLeftSensorModel(0.340098, random.getRange(3.0));
		frontright[0] = new AliceIRRightSensorModel(0.340098, random.getRange(3.0));
		frontright[1] = new AliceIRRightSensorModel(0.309804, random.getRange(3.0));
		frontright[2] = new AliceIRRightSensorModel(0.305098, random.getRange(3.0));
		normal[0] = new AliceIRNormalSensorModel(0.345098, random.getRange(3.0));
		normal[1] = new AliceIRNormalSensorModel(0.309804, random.getRange(3.0));
		normal[2] = new AliceIRNormalSensorModel(0.345098, random.getRange(3.0));
		high[0] = new AliceIRHighSensorModel(0.3451, random.getRange(4.0));
		high[1] = new AliceIRHighSensorModel(0.309804, random.getRange(4.0));
		high[2] = new AliceIRHighSensorModel(0.3451, random.getRange(4.0));
		
		// If you want all sensor with the same model, use this version
		/*
		frontleft[0] = new AliceIRLeftSensorModel(0.305098, 0);
		frontleft[1] = new AliceIRLeftSensorModel(0.309804, 0);
		frontleft[2] = new AliceIRLeftSensorModel(0.340098, 0);
		frontright[0] = new AliceIRRightSensorModel(0.340098, 0);
		frontright[1] = new AliceIRRightSensorModel(0.309804, 0);
		frontright[2] = new AliceIRRightSensorModel(0.305098, 0);
		normal[0] = new AliceIRNormalSensorModel(0.345098, 0);
		normal[1] = new AliceIRNormalSensorModel(0.309804, 0);
		normal[2] = new AliceIRNormalSensorModel(0.345098, 0);
		high[0] = new AliceIRHighSensorModel(0.3451, 0);
		high[1] = new AliceIRHighSensorModel(0.309804, 0);
		high[2] = new AliceIRHighSensorModel(0.3451, 0);
		*/
	}

	Alice::SensorModels::~SensorModels()
	{
		for (size_t i = 0; i<3; i++)
		{
			delete frontleft[i];
			delete frontright[i];
			delete normal[i];
			delete high[i];
		}
	}

	Alice::Alice(unsigned capabilities) :
		sensorModels(),
	// IRSensor(Robot *owner, Vector pos, double height, double orientation, double range, double aperture, unsigned rayCount, double *rayCombinationKernel, SensorResponseFunction sensorModel);
	// the radius is the max range of the IR Sensor.
		// old values: orientation: 0.73030; aperture: 0.77883
		left(this, Vector (0.788235 + (-0.15 + random.getRange(0.3)), 0.944118 + (-0.05 + random.getRange(0.1))),     1.2, 0.77030 + (-0.1745 + random.getRange(0.3490)),   2.5,  0.73883,  3, sensorModels.frontleft), 
		front(this, Vector (1.25, 0),        		1.2, 0,         2.5,  1.5291,   3, sensorModels.normal),
		right(this, Vector (0.788235 + (-0.15 + random.getRange(0.3)), -0.944118 + (-0.05 + random.getRange(0.1))),   1.2,-0.77030 + (-0.1745 + random.getRange(0.3490)),   2.5,  0.73883,  3, sensorModels.frontright), // old value: 0.77883
 		back(this, Vector (-0.95, 0),        		1.2, M_PI,      2.5,  M_PI/6,   3, sensorModels.normal),
		// IR Sensor on new extension module with camera
		frontHighExt(this, Vector (0.917843 , -.2),    2.4, 0,         9.0, 0.6222,  3, sensorModels.high),
		// back left sensor of the new ext module; global angle of 160 deg.
		backleft(this, Vector (-0.95 + (-0.15 + random.getRange(0.3)), 1.05 + (-0.05 + random.getRange(0.1))), 1.2, 8*M_PI/9 + (-0.1745 + random.getRange(0.3490)), 2.5,  M_PI/6,  3, sensorModels.frontleft),
		backright(this, Vector(-0.95 + (-0.15 + random.getRange(0.3)), -1.05 + (-0.05 + random.getRange(0.1))),1.2, -8*M_PI/9 + (-0.1745 + random.getRange(0.3490)),  2.5,  M_PI/6,  3, sensorModels.frontright),
		// CircularCam(Robot *owner, Vector pos, double height = 0, double orientation, double fieldOfView, unsigned pixelCount);
		circcam(this, Vector (0, 0), 3, 0, M_PI/10, 6),
		comm(4.6, this)
	{
		if (capabilities & CAPABILITIY_BASIC_SENSORS)
		{
			addLocalInteraction(&left);
			addLocalInteraction(&front);
			addLocalInteraction(&right);
			addLocalInteraction(&back);
		}
		if (capabilities & CAPABILITIY_CAMERA)
		{
			addLocalInteraction(&circcam);
		}
		if (capabilities & CAPABILITIY_COMMUNICATION)
		{
			comm.addSensor(Point(-2.1,2.1), 1.6);
			comm.addSensor(Point(0,2.6), 1.6);
			comm.addSensor(Point(2.1,2.1), 1.6);
			comm.addSensor(Point(0,-2.6), 1.6);	
			addLocalInteraction(&comm);
		}
		if (capabilities & CAPABILITIY_EXT_HIGH_SENSOR)
		{
			addLocalInteraction(&frontHighExt);
		}
		if (capabilities & CAPABILITIY_EXT_BACK_LR_SENSORS)
		{
			addLocalInteraction(&backleft);
			addLocalInteraction(&backright);
		}

		leftSpeed = rightSpeed = 0;
		
		/*
		// Create a slightly different bounding surface for each alice
		AP.push_back(Point(-1.0,-1));
		AP.push_back(Point(1.0,-1));
		
		AP.push_back(Point(1.30 + (-0.1 + random.getRange(0.2)),-.2 - (random.getRange(0.5))));
		AP.push_back(Point(1.30 + (-0.1 + random.getRange(0.2)),.2 + (random.getRange(0.5))));

		AP.push_back(Point(1.0,1));
		AP.push_back(Point(-1.0,1));
		AP.push_back(Point(-1.35 + (-0.15 + random.getRange(0.3)),(-0.4 + random.getRange(0.8))));
		setBoundingSurface(&AP);
		*/
		// Use a predefined bounding surface
		mass = 10;
		height = 4;
		setupBoundingSurface(aliceBoundingSurface.p);
		color = Color::blue;

		// set high static friction threshold to exclude Alices pushed by disks and 
		// delay Alice-Alice pushing (factor 2 slowdown; since collisions are symmetric,
		// we don't know which Alice is pushing which one)
		staticFrictionThreshold = 1.0;
		collisionAngularFrictionFactor = 1.7;
		viscousFrictionTau = 0.5;
		viscousMomentFrictionTau = 0.0;
		
		commitPhysicalParameters();
	}

	void Alice::step(double dt) 
	{
		double realLeftSpeed, realRightSpeed;
		
		// rightSpeed/leftSpeed are the speeds as given by the NN output 
		// function,i.e. between -1 and 1. we do a speed transformation,
		// eliminating small Alice motor speeds, since the stepmotors 
		// do not function correctly for small velocities
		// currently, possible speeds are between 2.5 and 4cm/s
		if (fabs(leftSpeed) < 0.625)
			realLeftSpeed = 0;
		else
			// we apply an inversely proportional motor noise to the speeds between 
			// -4 and +4; the noise is of the order of +/- 5% for the maximum speed, 
			// for +/- 3cm/s it is +/- 10%
			realLeftSpeed = 4 * leftSpeed * (0.95 + random.getRange(0.05));

		if (fabs(rightSpeed) < 0.625)
			realRightSpeed = 0;
		else
			// same as above
			realRightSpeed = 4 * rightSpeed * (0.95 + random.getRange(0.05));

		// forward component
		double forwardSpeed = (realLeftSpeed+realRightSpeed) / 2;
		double wheelDist = 1.9;
		// Khepera code:
		speed = Vector(
					forwardSpeed * cos(angle + angSpeed * dt * 0.5),
					forwardSpeed * sin(angle + angSpeed * dt * 0.5));

		// angle
		angSpeed += (-realLeftSpeed+realRightSpeed) / wheelDist;

		// handle physics
		PhysicalObject::step(dt);
	}
}

