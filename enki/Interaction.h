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

#ifndef __ENKI_INTERACTION_H
#define __ENKI_INTERACTION_H

/*!	\file Interaction.h
	\brief The interfaces for the interactions
*/

/*!	\defgroup interaction Interaction classes
	The locals and globals interactions.
*/

namespace Enki
{
	class PhysicalObject;
	class Robot;
	class World;

	//! Interacts with another object or wall only up to a certain distance
	/*! \ingroup core */
	class LocalInteraction
	{
	protected:
		//! Radius of the local interaction
		double r;

		//! Robots can access protected members me
		friend class Robot;
		//! The physical object that owns the interaction.
		Robot *owner;

	public :
		//! Constructor
		LocalInteraction():r(0) {}
		//! Constructor
		LocalInteraction(double range, Robot* owner) : r(range), owner(owner) {}
		//! Destructor
		virtual ~LocalInteraction() { }
		//! Init at each step
		virtual void init(double dt, World* w) { }
		//! Interact with object
		/*!
			\param dt time step
			\param po object to interact with
			\param w world where the interaction takes place
		*/
		virtual void objectStep(double dt, World* w, PhysicalObject *po) { }
		//! Interact with walls
		/*!
			\param w world to which interact
		*/
		virtual void wallsStep(double dt, World* w) { }
		//! Finalize at each step
		virtual void finalize(double dt, World* w) { }
		//! Return the range of the interaction
		double getRange() const { return r; }
	};

	//! Interacts with the whole world
	/*! \ingroup core */
	class GlobalInteraction
	{
	protected:
		//! The physical object that owns the interaction.
		Robot *owner;

	public :
		//! Constructor
		GlobalInteraction() {}
		//! Constructor
		GlobalInteraction(Robot* owner) : owner(owner) {}
		//! Destructor
		virtual ~GlobalInteraction() { }
		//! Init at each step
		virtual void init(double dt, World *w) { }
		//! Interact with world
		virtual void step(double dt, World *w) { }
		//! Finalize at each step
		virtual void finalize(double dt, World *w) { }
	};
}
#endif

