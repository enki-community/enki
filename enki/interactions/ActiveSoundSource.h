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

#ifndef __ENKI_ACTIVESOUNDOBJECT_H
#define __ENKI_ACTIVESOUNDOBJECT_H

#include "../Interaction.h"
#include "../PhysicalEngine.h"

/*!	\file ActiveSoundSource.h
	\brief Header of sound emitter interaction
*/
namespace Enki
{
	
	//! Time limited sound emitter
	/*! \ingroup interaction */
	class ActiveSoundSource: public LocalInteraction
	{
	public:
		//! No of channels of this sound source
		unsigned noOfChannels;
		
		//! Produced sound: vector of different pitch as they were channels.
		double *pitch;
		
		//! Sound activity
		bool enableFlag;
		//! Elapsed time since last activation
		double elapsedTime;
		//! Activity time
		double activityTime;
		
		//! Constructor
		ActiveSoundSource(Robot *owner, double r, unsigned channels);
		//! Destructor
		~ActiveSoundSource();
		// Local interaction functions
		virtual void init() {}
		virtual void objectStep(double dt, PhysicalObject *po, World *w) {}
		
		//! Set the range of this sound interraction
		void setSoundRange(double range);
		//! Get the value associated with channel
		double getSound(unsigned channel);
		//! Get the maximum value, set channel to the channel where this maximum lies
		double getMaxSound(int* channel);
		//! Set the value of channel to signal using a simplified model
		void setSound(unsigned channel, double signal);
		//! Set the value of channel to signal using a more realistic model
		void realisticSetSound(unsigned channel, double signal);
	};
	
	//! ActiveSoundObject can be inherited by any robot that want to emit sound
	/*! \ingroup robot */
	class ActiveSoundObject
	{
	public:
		//! Speaker
		ActiveSoundSource speaker;

	public:
		//! Constructor. Owner must point to the object which carries this emitter
		ActiveSoundObject(Robot *owner, double actionRange, unsigned channels);
	};
}

#endif

