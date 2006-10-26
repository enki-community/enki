/*
  Enki - a fast 2D robot simulator
  Copyright (C) 1999-2005 Stephane Magnenat <nct@ysagoon.com>
  Copyright (C) 2004-2005 Markus Waibel <markus.waibel@epfl.ch>
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

#ifndef __MICROPHONE_H
#define __MICROPHONE_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>
#include <enki/robots/s-bot/ActiveSoundObject.h>
//#include <enki/robots/s-bot/Sbot.h>

#include <valarray>

/*!	\file Microphone.h
  \brief Header of the generic infrared sensor
*/

namespace Enki
{
	// A function for manipulating acquired sound, normally to model 
	// saturation, distance decreasing or frequency response
	typedef double (*MicrophoneResponseModel)(double, double);

	/*!	\defgroup responsefunctor Response function classes
	  Different response functions for different sensors
	
	//! Functor for the sensor response
	! \ingroup responsefunctor 
	struct SensorResponseFunctor
	{
		//! Virtual destructor, do nothing
		virtual ~SensorResponseFunctor() {}
		//! Return the response for a given distance and object color
		virtual double operator()(double, const An::Color &) = 0;
	};*/
	
	//! A generic sound sensor/microphone
	/*! \ingroup interaction */
	class Microphone : public LocalInteraction {
	protected:
		//! Robot/object with the microphone
		PhysicalObject *owner;
		//! Absolute position in the world, updated on init()
		An::Vector micAbsPos;
		//! Relative position of mic on object
		An::Vector micRelPos;
		// microphone frequency response model
		MicrophoneResponseModel micModel;
		//! Actual detection range
		double range;
		//! No of frequency channels distinguished in input
		unsigned noOfChannels;
		//! microphone input signal (array of size noOfChannels)
		double* acquiredSound;
		
	public: 
		//! Constructor
		//! e.g.: Microphone(this, An::Vector(0.5, 0.5), 5, micStepModel, 20);
		//! meaning: the mic is (0.5, 0.5) away from robot center, can hear sounds up to
		//! 5 units away, uses a step model to detect sounds and can distinguish 20 frequencies
		Microphone(Robot *owner, An::Vector micRelPos, double range, 
				   MicrophoneResponseModel micModel, unsigned channels);
		//! Destructor
		~Microphone(void);
		//! Reset distance values, called every w->step()
		void init();
		//! Check for local interactions with other physical objects
		virtual void objectStep(double dt, PhysicalObject *po, World *w);
		//! Reset sound buffer to 0 after one time-step in experiment
		void resetSound(void);
		//! Return frequencies of input sound
		double* getAcquiredSound(void);
		//! Find frequency with maximum intensity
		void getMaxChannel(double *intensity, int *channel);
		//! Get absolute position of microphone
		An::Vector getMicAbsPos();
	};

	//! A generic sound sensor/microphone
	/*! \ingroup interaction */
	class FourWayMic : public LocalInteraction {
	protected:
		//! Robot/object with the microphone
		PhysicalObject *owner;
		//! Absolute position in the world, updated on init()
		An::Vector allMicAbsPos[4];
		//! Distance of the mics from centre of object
		double micDist;
		// microphone frequency response model
		MicrophoneResponseModel micModel;
		//! Actual detection range
		double range;
		//! No of frequency channels distinguished in input
		unsigned noOfChannels;
		//! microphone input signal (array of size noOfChannels for 4 mics)
		double* acquiredSound[4];
		
	public: 
		//! Constructor
		//! e.g.: FourWayMic(this, 0.5, 5, micStepModel, 20);
		//! meaning: each of the 4 mics is 0.5 away from robot center, can hear sounds up to
		//! 5 units away, uses a step model to detect sounds and can distinguish 20 frequencies
		FourWayMic(Robot *owner, double micDist, double range, 
				   MicrophoneResponseModel micModel, unsigned channels);
		//! Destructor
		~FourWayMic(void);
		//! Reset distance values, called every w->step()
		void init();
		//! Check for local interactions with other physical objects
		virtual void objectStep(double dt, PhysicalObject *po, World *w);
		//! Reset sound buffer to 0 after one time-step in experiment
		void resetSound(void);
		//! Return frequencies of input sound
		double* getAcquiredSound(unsigned micNo);
		//! Find frequency with maximum intensity
		void getMaxChannel(unsigned micNo, double *intensity, int *channel);
		//! Get absolute position of microphone
		An::Vector getMicAbsPos(unsigned micNo);
	};

	/*****************michele***********************/
	/*
	// A function for manipulating acquired sound, normally to model 
	// saturation, distance decreasing or frequency response
	typedef double (*MicrophoneResponseModel)(double, double);
	
	// Microphone general class
	class Microphone : public LocalInteraction
	{
	protected:
	
	PhysicalObject *owner;
	
	public:
	// microphone frequency response model
	MicrophoneResponseModel micModel;
	// microphone relative position in the robot
	An::Vector micRelPosition;
	// microphone resulting absolute position
	An::Vector micAbsPosition;
	
	// TODO :microphone generic characteristics
	double gain;
	unsigned noOfChannels;
	
	// microphone input signal
	double* acquiredSound;
	
	Microphone(Robot *owner, An::Vector micRelPos, double range, MicrophoneResponseModel micModel, unsigned channels);
	~Microphone(void);
	void init();
	void objectStep(double dt, PhysicalObject *po, World *w);
	void finalize(double dt);
	
	void resetMicrophone(void);
	
	double getAcqSound(unsigned index);
	
	};
	*/	
}

#endif
