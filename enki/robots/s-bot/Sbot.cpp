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

#include "enki/robots/s-bot/Sbot.h"

/*!	\file Sbot.cpp
	\brief Implementation of the Sbot robot
*/

namespace Enki
{
	//! Response model for sound on s-bot
	double MicrophonePseudoRealResponseModel(double signal, double distance)
	{
		//apply filter to signal
		double Lp;
		double d = distance/10;
		double attenuation = 100;
		if (distance <= 5.2)
			Lp = log(signal);
		else
			Lp = log(signal/exp(d*d/attenuation));

		if (Lp < 0) return 0.0;
		return Lp;
	}
	
	void SbotMicrophone::objectStep(double dt, PhysicalObject *po, World *w)
	{
		// Get current object sound
		double *currentSound = new double[noOfChannels];
		assert(currentSound);
		
		SbotActiveSoundObject *so = dynamic_cast<SbotActiveSoundObject *>(po);
		if (so)
		{
			assert(noOfChannels==so->speaker.noOfChannels);
			for (size_t i=0; i<noOfChannels; i++)
			{
				currentSound[i] = so->speaker.pitch[i];
			}
		}
		else
		{
			SoundSbot *ss = dynamic_cast<SoundSbot *>(po);
			if (ss)
			{
				assert(noOfChannels==ss->speaker.noOfChannels);
				for (size_t i=0; i<noOfChannels; i++)
				{
					currentSound[i] = ss->speaker.pitch[i];
				}
			}
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
		{
			acquiredSound[min_dist_micNo][j] += micModel(currentSound[j], min_dist);
		}
		
		delete[] currentSound;
	}
	
	Sbot::Sbot() :
		DifferentialWheeled(5, 40, 0.02),
		camera(this, 12, 64),
		globalSound(this)
	{
		addLocalInteraction(&camera);
		//addGlobalInteraction(&globalSound);
		
		setCylindric(6, 15, 500);
	}
	
	unsigned SbotGlobalSound::worldFrequenciesState = 0;
	
	unsigned SbotGlobalSound::getWorldFrequenciesState(void)
	{
		return worldFrequenciesState;
	}
		
	void FeedableSbot::controlStep(double dt)
	{
		DifferentialWheeled::controlStep(dt);
		
		// clear dEnergy for next step
		energy += dEnergy * dt;
		lastDEnergy = dEnergy;
		dEnergy = 0;
	}

	SoundSbot::SoundSbot() :
		// microphones can pick up sound reaching up to 1m away
		mic(this, 6.0, 150, MicrophonePseudoRealResponseModel, 25),
		// speaker can produce sounds heard by a microphone 1m + micrange away
		speaker(this, 0, 25)
	{
		addLocalInteraction(&mic);
		addLocalInteraction(&speaker);
	}
}

