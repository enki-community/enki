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

#ifndef __ENKI_MICROPHONE_H
#define __ENKI_MICROPHONE_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>
#include "ActiveSoundSource.h"

#include <valarray>

/*!	\file Microphone.h
  \brief Header of the generic infrared sensor
*/

namespace Enki
{
	//! A function for manipulating acquired sound, normally to model saturation, distance decreasing or frequency response
	typedef double (*MicrophoneResponseModel)(double, double);

	//! A generic sound sensor/microphone
	/*! \ingroup interaction */
	class Microphone : public LocalInteraction
	{
	protected:
		//! Robot/object with the microphone
		PhysicalObject *owner;
		//! Absolute position in the world, updated on init()
		Vector micAbsPos;
		//! Relative position of mic on object
		Vector micRelPos;
		//! Microphone frequency response model
		MicrophoneResponseModel micModel;
		//! Actual detection range
		double range;
		//! No of frequency channels distinguished in input
		unsigned noOfChannels;
		//! microphone input signal (array of size noOfChannels)
		double* acquiredSound;
		
	public: 
		//! Constructor
		//! e.g.: Microphone(this, Vector(0.5, 0.5), 5, micStepModel, 20);
		//! meaning: the mic is (0.5, 0.5) away from robot center, can hear sounds up to
		//! 5 units away, uses a step model to detect sounds and can distinguish 20 frequencies
		Microphone(Robot *owner, Vector micRelPos, double range, 
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
		Vector getMicAbsPos();
	};

	//! A generic sound sensor/microphone
	/*! \ingroup interaction */
	class FourWayMic : public LocalInteraction
	{
	protected:
		//! Robot/object with the microphone
		PhysicalObject *owner;
		//! Absolute position in the world, updated on init()
		Vector allMicAbsPos[4];
		//! Distance of the mics from centre of object
		double micDist;
		//! Microphone frequency response model
		MicrophoneResponseModel micModel;
		//! Actual detection range
		double range;
		//! No of frequency channels distinguished in input
		unsigned noOfChannels;
		//! Microphone input signal (array of size noOfChannels for 4 mics)
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
		Vector getMicAbsPos(unsigned micNo);
	};
}

#endif
