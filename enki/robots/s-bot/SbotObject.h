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

#ifndef __ENKI_SBOTOBJECT_H
#define __ENKI_SBOTOBJECT_H

#include <enki/Interaction.h>
#include <enki/PhysicalEngine.h>
#include <enki/interactions/ActiveSoundSource.h>

/*!	\file SbotObject.h
	\brief Header of the Sbot Active Object
*/
namespace Enki
{
	//! Feeding interaction gives or remove energy to nearby Sbots
	/*! \ingroup interaction */
	class SbotFeeding : public LocalInteraction
	{
	public:
		//! The energy in stock
		double actualEnergy;
		//! The actual time
		double actualTime;
		//! The duration of active period
		double activeDuration;
		//! The duration of inactive period
		double inactiveDuration;
		//! The color of the object when active: (actualTime % (activeDuration+inactiveDuration) < activeDuration)
		Color activeColor;
		//! The color of the object when inactive: (actualTime % (activeDuration+inactiveDuration) >= activeDuration)
		Color inactiveColor;
		//! If true, energy given to the Sbots is removed from actualEnergy
		bool consumeEnergy;
		//! The energy difference per second when active
		double dEnergyActive;
		//! The energy difference per second when inactive
		double dEnergyInactive;

	public :
		//! Constructor, r is the radius of the interaction
		SbotFeeding(double r, Robot *owner);
		virtual void objectStep (double dt, PhysicalObject *po, World *w);
		virtual void finalize(double dt);
	};

	//! SbotActiveObject give or remove energy to nearby Sbots through an SbotFeeding interaction
	/*! \ingroup robot */
	class SbotActiveObject : public Robot
	{
	public:
		//! The interaction
		SbotFeeding feeding;
		
	public:
		//! Constructor
		SbotActiveObject(double objectRadius, double actionRange);
	};

	//! SbotActiveSoundObject give or remove energy to nearby Sbots through an SbotFeeding interaction
	/*! \ingroup robot */
	class SbotActiveSoundObject : public SbotActiveObject, public ActiveSoundObject
	{
	public:
		//! Constructor
		SbotActiveSoundObject(double objectRadius, double actionRange);
		//! Set the range of the sound interaction
		void setSoundRange(double soundRange);
	};
}
#endif

