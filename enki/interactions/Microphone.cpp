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

#include "Microphone.h"
#include <assert.h>
#include <iostream>
#include <sstream>
#include <limits>

/*!	\file Microphone.cpp
	\brief Implementation of the generic sound sensor/microphone
*/

namespace Enki
{
	Microphone::Microphone(Robot *owner, Vector micRelPos, double range,
						 MicrophoneResponseModel micModel, unsigned channels)
	{
		this->owner = owner;
 		this->r = range;
		this->micModel = micModel;
		this->micRelPos = micRelPos;
		this->noOfChannels = channels;
		this->acquiredSound = new double[noOfChannels];
			
		for (size_t i=0; i<noOfChannels; i++)
			acquiredSound[i] = 0.0;

		Matrix22 rot(owner->angle);
		micAbsPos = owner->pos + rot*micRelPos;
	}

	Microphone::~Microphone(void)
	{
		delete[] acquiredSound;
	}
	
	void Microphone::init()
	{
		Matrix22 rot(owner->angle);
		micAbsPos = owner->pos + rot*micRelPos;
		resetSound();
	}

	void Microphone::objectStep(double dt, PhysicalObject *po, World *w)
	{
		// Current distance between the interacting physical object and 
		// the sensor (used later in sound filtering)
		double current_dist = (po->pos - micAbsPos).norm();
	
		// Get current object sound
		double *currentSound = new double[noOfChannels];
		assert(currentSound);
		
		
		ActiveSoundObject *so = dynamic_cast<ActiveSoundObject *>(po);
		if (so)
		{
			assert(noOfChannels == so->speaker.noOfChannels);
			for (size_t i=0; i<noOfChannels; i++)
				currentSound[i] = so->speaker.pitch[i];
		}
		else
			for (size_t i=0; i<noOfChannels; i++)
				currentSound[i] = 0.0;
		
		// Apply sensor model to acquisition
		for (size_t i=0; i<noOfChannels; i++)
			currentSound[i] = micModel(currentSound[i], current_dist);
		
		// Acquired sound is always the sum of all contributes after model filtering
		for(size_t i=0; i<noOfChannels; i++) 
			acquiredSound[i] += currentSound[i];
		
		// 3.0 is the saturating value of tanh used in sigmoidal neurons
		/*
		for(size_t i=0; i<noOfChannels; i++) 
			if ((acquiredSound[i]*acquiredSound[i])>9.0)
				acquiredSound[i] = 3.0;
		*/
	 
		delete[] currentSound;

	}

	double* Microphone::getAcquiredSound(void)
	{
		return acquiredSound;
	}

	void Microphone::resetSound()
	{
		for (size_t i=0; i<noOfChannels; i++) acquiredSound[i] = 0.0;
	}

	void Microphone::getMaxChannel(double *intensity, int *channel)
	{
		*intensity = 0;
		*channel = -1;
		for (size_t i = 0; i < noOfChannels; i++ )
		{
			if ( acquiredSound[i] > *intensity )
			{
				*intensity = acquiredSound[i];
				*channel = i;
			}
		}
	}

	Vector Microphone::getMicAbsPos()
	{
		return micAbsPos;
	}
		
	FourWayMic::FourWayMic(Robot *owner, double micDist, double range, 
						   MicrophoneResponseModel micModel, unsigned channels)
	{
		this->owner = owner;
		this->r = range;
		this->micModel = micModel;
		this->micDist = micDist;
		this->noOfChannels = channels;
		for (size_t i=0; i<4; i++)
		{
			this->acquiredSound[i] = new double[noOfChannels];
			for (size_t j=0; j<noOfChannels; j++)
				acquiredSound[i][j] = 0.0;
		}

		Matrix22 rot(owner->angle);
		allMicAbsPos[0] = owner->pos + rot*Vector( micDist, micDist);
		allMicAbsPos[1] = owner->pos + rot*Vector( micDist,-micDist);
		allMicAbsPos[2] = owner->pos + rot*Vector(-micDist, micDist);
		allMicAbsPos[3] = owner->pos + rot*Vector(-micDist,-micDist);
	}

	FourWayMic::~FourWayMic(void)
	{
		for (unsigned i=0; i<4; i++)
			delete[] acquiredSound[i];
	}
		
	void FourWayMic::init()
	{
		Matrix22 rot(owner->angle);
		allMicAbsPos[0] = owner->pos + rot*Vector( micDist, micDist);
		allMicAbsPos[1] = owner->pos + rot*Vector( micDist,-micDist);
		allMicAbsPos[2] = owner->pos + rot*Vector(-micDist, micDist);
		allMicAbsPos[3] = owner->pos + rot*Vector(-micDist,-micDist);
		resetSound();
	}

	void FourWayMic::objectStep(double dt, PhysicalObject *po, World *w)
	{
		// Get current object sound
		double *currentSound = new double[noOfChannels];
		assert(currentSound);
		
		ActiveSoundObject *so = dynamic_cast<ActiveSoundObject *>(po);
		if (so)
		{
			assert(noOfChannels == so->speaker.noOfChannels);
			for (size_t i=0; i<noOfChannels; i++)
				currentSound[i] = so->speaker.pitch[i];
		}
		
		// Current distance between the interacting physical object and 
		// the sensor (used later in sound filtering)
		double current_dist;
		double min_dist = 0xFFFFFFFF;
		unsigned min_dist_micNo = 0;
		for (size_t i=0; i<4; i++)
		{
			current_dist = (po->pos - allMicAbsPos[i]).norm();
			// find mic closest to interacting physical object
			if ( current_dist < min_dist )
			{
				min_dist = current_dist;
				min_dist_micNo = i;
			}
		}

		// Apply sensor model to acquisition
		// Acquired sound is always the sum of all contributes after model filtering
		for (size_t j=0; j<noOfChannels; j++)
			acquiredSound[min_dist_micNo][j] += micModel(currentSound[j], min_dist);
		
		// 3.0 is the saturating value of tanh used in sigmoidal neurons
		/*
		for(size_t i=0; i<noOfChannels; i++) 
			if ((acquiredSound[i]*acquiredSound[i])>9.0)
				acquiredSound[i] = 3.0;
		*/
	 
		delete[] currentSound;

	}

	double* FourWayMic::getAcquiredSound(unsigned micNo)
	{
		return acquiredSound[micNo];
	}

	void FourWayMic::resetSound()
	{
		for (size_t i=0; i<4; i++) 
			for (size_t j=0; j<noOfChannels; j++) 
				acquiredSound[i][j] = 0.0;
	}

	void FourWayMic::getMaxChannel(unsigned micNo, double *intensity, int *channel)
	{
		*intensity = 0;
		*channel = -1;
		for (size_t i = 0; i < noOfChannels; i++ )
		{
			if ( acquiredSound[micNo][i] > *intensity )
			{
				*intensity = acquiredSound[micNo][i];
				*channel = i;
			}
		}
	}

	Vector FourWayMic::getMicAbsPos(unsigned micNo)
	{
		return allMicAbsPos[micNo];
	}
}
