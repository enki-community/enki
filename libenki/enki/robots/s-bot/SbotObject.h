/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2005 Stephane Magnenat <nct@ysagoon.com>
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

#ifndef __SBOTOBJECT_H
#define __SBOTOBJECT_H

#include <enki/Interaction.h>
#include <enki/PhysicalEngine.h>
#include <enki/robots/s-bot/ActiveSoundObject.h>

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
		An::Color activeColor;
		//! The color of the object when inactive: (actualTime % (activeDuration+inactiveDuration) >= activeDuration)
		An::Color inactiveColor;
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
	class SbotActiveSoundObject : public SbotActiveObject {
	public:
		//! Speaker
		ActiveSoundSource speaker;

	public:
		//! Constructor
		SbotActiveSoundObject(double objectRadius, double actionRange);
		void setSoundRange(double soundRange);
	};
}
#endif

