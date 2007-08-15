/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2006 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://lis.epfl.ch/enki
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

#ifndef __ENKI_PHYSICALENGINE_H
#define __ENKI_PHYSICALENGINE_H

#include <iostream>
#include <set>
#include <vector>
#include <valarray>
#include "Types.h"
#include "Geometry.h"
#include "Random.h"
#include "Interaction.h"
#include "BluetoothBase.h"

/*!	\file PhysicalEngine.h
	\brief The core of Enki.
	
	The world, the objects located in it and the interaction logic are all defined in this file.
*/

/*!	\defgroup core Core Enki classes
	The world, basic physical objects and interaction logic
*/

/*!	\defgroup robot Robots classes
	The robots
*/

/*!	\mainpage Enki Reference documentation

	\section intro Introduction
	This is the reference for the Enki simulator library. Enki is a fast 2D simulator
	designed to support the simulation of colonies of robots hundred times faster
	than real time. To achieve this, simplified 2D physics and optimised hierarchical
	interaction models are used.
	
	\section installation Installation
	The installation procedure is explained step by step in the \ref InstallGuide.
	
	\section usage Usage
	A quick start to most important classes is given in the \ref Cookbook.
	
	If you want to extend Enki, do not forget to read and follow the \ref CodingConventions.
	
	\section designChoices Design choices
	The basic datatype is double. It is used everywhere excepted if another datatype
	specifically makes sense.
	
	The core concept in Enki is the interaction. An interaction can be local, i.e. apply only up to a
	certain range, or global, i.e. apply to the whole world.
	
	Local interactions are object <-> object
	and called by the inner simulation loop only when objects are below the interaction range.
	In objects, local interactions are sorted from long to short range so that once one is out
	of range, the following will be too. This is the main optimization in Enki that permits large
	colonies of robots. The complexity is still O(n2) so if very large colonies are required, a larger
	scale, grid based optimization should be used. It is currently not implemented in Enki.
	Physical dynamics between objects are the shortest ranged local interactions.
	Local interactions can also interact with walls. Physical dynamics between objects and walls are
	similar to local interactions with other objects, but use a different method of calculation.
	
	Global interactions are object <-> world.
	
	\section state Development state
	
	The core, the IRSensor, and the basic Khepera, EPuck, Alice and Sbot features reflect real hardware and thus won't change much.
	
	The sound sources and Sbot camera don't reflect directly real hardware and thus are subject to change.
	
	\section feedback Feedback
	If you have any comments or suggestions, do not hesitate to send them to
	Stephane Magnenat (stephane at magnenat dot net).
	
	\section license License
	This program is free software released under the GNU General Public License version 2.
	The authors of any publication arising from research using this software are asked to add the following reference:
\verbatim
Enki - an open source fast 2D robot simulator
http://lis.epfl.ch/enki
Stephane Magnenat <stephane at magnenat dot net>,
Markus Waibel <markus dot waibel at epfl dot ch>,
Antoine Beyeler <abeyeler at ab-ware dot com>
Laboratory of Intelligent Systems, EPFL, Lausanne
\endverbatim
*/

//! Enki is the namespace of the Enki simulator. All Enki functions and classes excepted the math one are inside this namespace
namespace Enki
{
	class World;

	//! A situated object in the world with mass, geometry properties, physical properties, ...
	/*! \ingroup core */
	class PhysicalObject
	{
	public:
		//! User specific data that can be attached to any object in the world.
		class UserData
		{
		public:
			virtual ~UserData() {}
		};
		
		//! Data attached by the user to this physical object. If non-null, will be destroyed with the object.
		UserData *userData;
	
		//! The position of the object.
		Point pos;
		//! The height of the object, used for interaction with robot's sensors.
		double height;
		//! The orientation of the object in the world, standard trigonometric orientation.
		double angle;
		//! The speed of the object.
		Vector speed;
		//! The rotation speed of the object, standard trigonometric orientation.
		double angSpeed;
		//! The mass of the object. If below zero, the object can't move (infinite mass).
		double mass;
		//! Indicates if the object collided with a wall..
		bool collisionWithWalls;

		//! The static friction threshold of the object. If a force is smaller than it, the object will not move.
		double staticFrictionThreshold;
		//! The viscous friction time constant. Half-life of speed when object is free. If lower than timestep, speed is forced to zero.
		double viscousFrictionTau;
		//! The viscous friction moment time constant. Half-life of angular speed when object is free. If lower than timestep, angular speed is forced to zero.
		double viscousMomentFrictionTau;
		//! Upon collision with static objects. The amount of rotation transmitted to the moving object. If zero, moving object slides over static one. If one, moving object is fully rotated.
		double collisionAngularFrictionFactor;
		//! The shape of the object in object coordinates. If NULL, the object is circular and its radius is given by r .
		const Polygone *boundingSurface;
		//! The shape of the object in world coordinates, updated on initLocalInteractions(). Invalid if boundingSurface is NULL.
		Polygone absBoundingSurface;
		//! The radius of circular objects. If boundingSurface is not NULL, it is automatically computed.
		double r;
		//! The reflection factor of the object. It acts only on proximity sensors. If one, the object is seen normally. If less than one, the range of vision of the object dimishes. If zero, the object is invisible to proximity sensors
		double reflection;
		//! The color of the object.
		Color color;
		//! Texture for several faces of this object.
		std::valarray<Texture> textures;
		
	protected:
		//! Vector used for object collisions. If its norm is greater than staticFrictionThreshold, the object is moved.
		Vector deinterlaceVector;

		//! Do the real rotation due to collision.
		void collideWithStaticObject(const Vector &n);
		//! Compute the shape of its object in world coordinates.
		void computeAbsBoundingSurface(void);

	public:
		//! Constructor
		PhysicalObject();
		//! Destructor
		virtual ~PhysicalObject();
		
		//! Set the shape of the object to bs, recompute r, assign color to faces. bs must exists during all object's life.
		void setBoundingSurface(const Polygone *bs);
		//! Return the shape of the object in object coordinates.
		const Polygone &getTrueBoundingSurface(void) const { return absBoundingSurface; }

		// Physical Actions
		//! A simulation step for this object. It is considered as deinterlaced. The position and orientation are updated, and speed is reduced according to global dynamic friction coefficient.
		virtual void step(double dt);

		//! Initialize the collision logic
		virtual void initLocalInteractions();
		//! Do the collision with the other PhysicalObject. firstInteraction controls which object collides with which, dt is not used.
		virtual void doLocalInteractions(World *w, PhysicalObject *o, double dt, bool firstInteraction);
		//! Do the collisions with the walls of world w.
		virtual void doLocalWallsInteraction(World *w);
		//! All collisions are finished, deinterlace the object.
		virtual void finalizeLocalInteractions(double dt);

		//! Initialize the global interactions, do nothing for PhysicalObject.
		virtual void initGlobalInteractions() { }
		//! Do the global interactions with the world, do nothing for PhysicalObject.
		virtual void doGlobalInteractions(World *w, double dt) { }
		//! All global interactions are finished, do nothing for PhysicalObject.
		virtual void finalizeGlobalInteractions() { }

		//! Dynamics for collision with a static object at points cp1 and cp2 with normals vectors n1 and n2 and a penetrated distance of dist.
		void collideWithStaticObject(const Point &cp1, const Point &cp2, const Vector &n1, const Vector &n2, const Vector &dist);
		//! Dynamics for collision with object at point cp with a penetrated distance of dist.
		void collideWithObject(PhysicalObject &object, const Point &cp, const Vector &dist);
		//! Returns if the object collided with a wall during the last time step.
		bool getCollideWithWalls() { return collisionWithWalls; }
	};

	//! A robot is a PhysicalObject that has additional interactions and a controller.
	/*! \ingroup core */
	class Robot: public PhysicalObject
	{
	protected:
		//! Vector of local interactions
		std::vector<LocalInteraction *> localInteractions;
		//! Vector of global interactions
		std::vector<GlobalInteraction *> globalInteractions;
		
	public:
		//! Constructor
		Robot();
		
		//! Add a new local interaction, re-sort interaction vector from long ranged to short ranged.
		void addLocalInteraction(LocalInteraction *li);
		//! Add a global interaction, just add it at the end of the vector.
		void addGlobalInteraction(GlobalInteraction *gi) {globalInteractions.push_back(gi);}
		//! Initialize the local interactions, call init on each one, then call PhysicalObject::initLocalInteractions.
		virtual void initLocalInteractions();
		//! Do the local interactions with other objects, call objectStep on each one, then call PhysicalObject::doLocalInteractions. firstInteraction controls which object interacts with which.
		virtual void doLocalInteractions(World *w, PhysicalObject *po, double dt, bool firstInteraction);
		//! Do the local interactions with walls, call wallsStep on each one, then call PhysicalObject::doLocalWallsInteraction.
		virtual void doLocalWallsInteraction(World *w);
		//! All the local interactions are finished, call finalize on each one, then call PhysicalObject::finalizeLocalInteractions.
		virtual void finalizeLocalInteractions(double dt);
		
		//! Do the global interactions, call step on each one.
		virtual void doGlobalInteractions(World *w, double dt);
		//! Sort local interactions. Called by addLocalInteraction ; can be called by subclasses in case of interaction radius change.
		void sortLocalInteractions(void);
	};

	//! The world is the container of all objects and robots.
	/*! It is a rectangular arena with walls at all sides.
		\ingroup core
	*/
	class World
	{
	protected:
		//! At each step objects are collided in a different order (first A-B, then B-A) to prevent side effects, collideEven contains the actual order.
		bool collideEven;

	public:
		typedef std::set<PhysicalObject *> Objects;
		typedef Objects::iterator ObjectsIterator;
		//! All the objects in the world
		Objects objects;
		//! If true, use walls
		bool useWalls;
		//! The width of the world
		const double w;
		//! The height of the world
		const double h;
		//! Texture of walls.
		Texture wallTextures[4];
		//! Base for the Bluetooth connections between robots
		class BluetoothBase* bluetoothBase;

		//! Return collideEven
		bool getCollideEven() {return collideEven;}
		//! Do the collision of a circular object with one with a different shape (convex boundingsurface)
		void collideCircleWithBS(PhysicalObject *circle, PhysicalObject *objectBS, const Polygone &bs);
		//! Collide two objects. Correct functions will be called depending on type of object (circular or other shape).
		void collideObjects(PhysicalObject *object1, PhysicalObject *object2);
		//! Collide the object with walls.
		void collideWithWalls(PhysicalObject *object);
		//! Return true if point p of object of center c is inside polygone bs and return deinterlacement distVector.
		bool isPointInside(const Point &p, const Point &c, const Polygone &bs, Vector *distVector);

	public:
		//! Constructor, takes width and height of the world arena in cm.
		World(double width, double height);
		//! Destructor, destroy all objects
		~World();
		//! Simulate a timestep of dt. dt should be below 1 (typically .02-.1)
		void step(double dt);
		//! Add an object to the world, simply add it to the vector. Object will be automatically deleted when world will be destroyed.
		//! If the object is already in the world, do nothing
		void addObject(PhysicalObject *o);
		//! Remove an object from the world and destroy it. If object is not in the world, do nothing
		void removeObject(PhysicalObject *o);
		//! Set the seed of the random generator.
		void setRandomSeed(unsigned long seed);
		//! Initialise and activate the Bluetooth base
		void initBluetoothBase();
		//! Return the address of the Bluetooth base
		BluetoothBase* getBluetoothBase();
	};
	
	//! Fast random for use by Enki
	extern FastRandom random;
}

#endif
