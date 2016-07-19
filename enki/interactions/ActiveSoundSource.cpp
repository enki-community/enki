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

#include "ActiveSoundSource.h"
#include <iostream>
#include <algorithm>
#include <limits.h>
#include <assert.h>

/*!	\file ActiveSoundSource.cpp
	\brief Implementation of sound emitter interaction
*/

namespace Enki
{
	ActiveSoundSource::ActiveSoundSource(Robot *owner, double r, unsigned channels)
	{
		this->r = r;
		this->owner = owner;
	
		noOfChannels = channels;
	
		pitch = new double[channels];
		assert(pitch);
		
		for (size_t i=0; i<channels; i++)
			pitch[i] = 0.0;
	
		enableFlag = false;
		elapsedTime = 0.0;
	
		activityTime = 5.0;
	}
	
	ActiveSoundSource::~ActiveSoundSource()
	{
		delete[] pitch;
	}

	void ActiveSoundSource::setSoundRange(double range)
	{
		this->r = range;
	}
		
	void ActiveSoundSource::setSound(unsigned channel, double signal)
	{
		if (channel < noOfChannels)
			pitch[channel] = signal;
	}
		
	void ActiveSoundSource::realisticSetSound(unsigned channel, double signal)
	{
		double variance = 1;
		//double gaussian;

		if (channel < noOfChannels)
		{
			/*
			pitch[channel] = signal;
			for (int i=1; i<=2; i++) {
				gaussian = exp(-(i*i)/(2*variance*variance));
				if (channel != 0 && (channel != 1 || i != 2)) 
					pitch[channel-i] = gaussian*signal;
				if (channel+i < noOfChannels) pitch[channel+i] = gaussian*signal;
			}
			*/
			int c = (int)channel + round(gaussianRand(0, variance));
			if (c < 0)
				c = 0;
			if (c >= noOfChannels)
				c = noOfChannels-1;
			channel = c;
			pitch[channel] = signal;
		}
	}

	double ActiveSoundSource::getSound(unsigned channel)
	{
		if (channel < noOfChannels)
			return (pitch[channel]);
		else
			return -1;
	}

	
	double ActiveSoundSource::getMaxSound(int* channel)
	{
		double maxPitch = 0;
		
		for (unsigned i=0; i<noOfChannels; i++)
			if (pitch[i] > maxPitch)
			{
				maxPitch = pitch[i];
				*channel = i;
			}

		if (maxPitch)
			return maxPitch; 
		else
			return -1;
	}
	
	ActiveSoundObject::ActiveSoundObject(Robot *owner, double actionRange, unsigned channels) :
		speaker(owner, actionRange, channels)
	{
	
	}
}
