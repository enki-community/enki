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

#ifndef __INTERACTION_H
#define __INTERACTION_H

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
		//! Destructor
		virtual ~LocalInteraction() { }
		//! Init at each step
		virtual void init() { }
		//! Interact with object
		virtual void objectStep(double dt, PhysicalObject *po, World *w) { }
		//! Interact with walls
		virtual void wallsStep(World *w) { }
		//! Finalize at each step
		virtual void finalize(double dt) { }
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
		//! Destructor
		virtual ~GlobalInteraction() { }
		//! Init at each step
		virtual void init() { }
		//! Interact with world
		virtual void step(double dt, World *w) { }
		//! Finalize at each step
		virtual void finalize() { }
	};
}
#endif

