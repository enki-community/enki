/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2008 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
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

#include "Geometry.h"
#include "Types.h"
#include "Random.h"
#include "Interaction.h"
#include "BluetoothBase.h"
#include <iostream>
#include <set>
#include <vector>
#include <valarray>


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
	
	The sound sources of Sbot does not reflect directly hardware and thus is subject to change.
	
	\section feedback Feedback
	If you have any comments or suggestions, do not hesitate to send them to
	Stephane Magnenat (stephane at magnenat dot net).
	
	\section license License
	This program is free software released under the GNU General Public License version 2.
	We ask the authors of any publication arising from research using this software to add the following reference:
\verbatim
Enki - an open source fast 2D robot simulator
http://home.gna.org/enki/

Stephane Magnenat <stephane at magnenat dot net>,
Markus Waibel <markus dot waibel at epfl dot ch>,
Antoine Beyeler <abeyeler at ab-ware dot com>

Laboratory of Intelligent Systems,
Mobots group - Laboratoire de Systèmes Robotiques,
École Polytechnique Fédérale de Lausanne - Switzerland
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
		friend class World;
	public:
		
		//! User specific data that can be attached to any object in the world.
		class UserData
		{
		public:
			bool deletedWithObject; //!< if true, deleted along with the physical object.
			
		public:
			//! Virtual destructor, call destructor of child classes
			virtual ~UserData() {}
		};
		
		//! Data attached by the user to this physical object. If non-null, will be destroyed with the object.
		UserData *userData;
		
		// physical constant
		static const double g;
		
		// physical parameters constants
		
		//! Elasticity of collisions of this object. If 0, soft collision, 100% energy dissipation; if 1, elastic collision, 0% energy dissipation. Actual elasticity is the product of the elasticity of the two colliding objects. Walls are fully elastics
		double collisionElasticity;
		//! The dry friction coefficient mu.
		double dryFrictionCoefficient;
		//! The viscous friction coefficient. Premultiplied by mass. A value of k applies a force of -k * speed * mass
		double viscousFrictionCoefficient;
		//! The viscous friction moment coefficient. Premultiplied by momentOfInertia. A value of k applies a force of -k * speed * momentOfInertia
		double viscousMomentFrictionCoefficient;
		
		// physics state variables
		
		// space coordinates
		
		//! The position of the object.
		Point pos;
		//! The orientation of the object in the world, standard trigonometric orientation.
		double angle;
		
		// space coordinates derivatives
		
		//! The speed of the object.
		Vector speed;
		//! The rotation speed of the object, standard trigonometric orientation.
		double angSpeed;
		
		// space coordinates double-derivatives
		
		/*
		The only accelerations we have for now are friction and are computed inside PhysicalObject::step()
		Both collisions and forces due to wheels of differential wheeled robots are computed instantly
		
		//! The acceleration of the object
		Vector acc;
		//! The angular acceleration of the object
		double angAcc;
		
		//! The static friction threshold of the object. If the force resulting from the interaction between non-infinite mass objects is smaller than this, this object will not move.
		double staticFrictionThreshold;
		*/
	protected:
		// mass and inertia tensor
		
		//! The mass of the object. If below zero, the object can't move (infinite mass).
		double mass;
		///! The moment of inertia tensor
		double momentOfInertia;
		
		// geometry properties
		
		//! The radius of circular objects. If boundingSurface is not empty, it is automatically computed upon setShape
		double r;
		//! The height of the object, used for interaction with robot's sensors.
		double height;
		//! The shape of the object in object coordinates. If empty, the object is circular and its radius is given by r .
		Polygone boundingSurface;
		//! The shape of the object in world coordinates, updated on initLocalInteractions(). Invalid if boundingSurface is NULL.
		Polygone absBoundingSurface;
		
		// visual properties
		
		//! The infrared reflection factor of the object. It acts only on proximity sensors. If one, the object is seen normally. If less than one, the range on which the object is visible dimishes. If zero, the object is invisible to proximity sensors
		double infraredReflectiveness;
		//! The color of the object. Must be set using setColor
		Color color;
		//! Texture for several faces of this object.
		Textures textures;
		
		// temporary variables
		//! Vector used for object collisions. If its norm is greater than staticFrictionThreshold, the object is moved.
		Vector deinterlaceVector;

		// internal functions
		
		//! Compute the moment of inertia tensor depending on radius, mass, height, and bounding surface
		void computeMomentOfInertia();
		//! Compute the shape of its object in world coordinates.
		void computeAbsBoundingSurface(void);
		//! Setup the bounding surface. Does not undate the moment of inertia tensor, for use by subclasses in setupPhysicalParameters() only. Outer class must call setShape() instead.
		void setupBoundingSurface(const Polygone& boundingSurface);
		
	public:
		//! Constructor
		PhysicalObject();
		//! Destructor
		virtual ~PhysicalObject();
		//! Call this function after you have setup all your parameters. Base class does not call this from its constructor, but robots will
		void commitPhysicalParameters();
		
		// getters
		inline double _radius() const { return r; }
		inline double _height() const { return height; }
		inline const Polygone& _boundingSurface() const { return boundingSurface; }
		inline double _infraredReflectiveness() const { return infraredReflectiveness; }
		inline const Color& _color() const { return color; }
		inline const Textures& _textures() const { return textures; }
		//! Return the shape of the object in object coordinates. If shape is not defined, return an empty polygone.
		inline const Polygone &getTrueBoundingSurface(void) const { return absBoundingSurface; }
		
		// setters
		
		//! Set the mass of the object
		void setMass(double mass);
		//! Make the object cylindric
		void setCylindric(double radius, double height);
		//! Make the shape of the object. The color is uniformely set to the current color.
		void setShape(const Polygone& boundingSurface, double height);
		//! Set a uniform color all around the object.
		void setColor(const Color &color);
		//! Set textures around an object with a bounding box; textures must be of the same size as the amount of face to the bounding box.
		void setTextures(const Texture* textures);
		//! Set The infrared reflection factor of the object. It acts only on proximity sensors. If one, the object is seen normally. If less than one, the range on which the object is visible dimishes. If zero, the object is invisible to proximity sensors
		void setInfraredReflectiveness(double infraredReflectiveness = 1.0);

	protected:
		// Physical Actions
		
		//! A physics simulation step for this object. It is considered as deinterlaced. The position and orientation are updated, and speed is reduced according to friction.
		virtual void step(double dt);

		//! Initialize the collision logic
		void initPhysicsInteractions();
		//! Do the collision with the other PhysicalObject. firstInteraction controls which object collides with which, dt is not used.
		void doPhysicsInteractions(World *w, PhysicalObject *o, double dt, bool firstInteraction);
		//! Do the collisions with the walls of world w.
		void doPhysicsWallsInteraction(World *w);
		//! All collisions are finished, deinterlace the object.
		void finalizePhysicsInteractions(double dt);
		
		//! Initialize the object specific interactions, do nothing for PhysicalObject.
		virtual void initLocalInteractions() { }
		//! Do the interactions with the other PhysicalObject, do nothing for PhysicalObject.
		virtual void doLocalInteractions(World *w, PhysicalObject *o, double dt) { }
		//! Do the interactions with the walls of world w, do nothing for PhysicalObject.
		virtual void doLocalWallsInteraction(World *w) { }
		//! All interactions are finished, do nothing for PhysicalObject.
		virtual void finalizeLocalInteractions(double dt) { }

		//! Initialize the global interactions, do nothing for PhysicalObject.
		virtual void initGlobalInteractions() { }
		//! Do the global interactions with the world, do nothing for PhysicalObject.
		virtual void doGlobalInteractions(World *w, double dt) { }
		//! All global interactions are finished, do nothing for PhysicalObject.
		virtual void finalizeGlobalInteractions() { }

		//! Dynamics for collision with a static object at points cp1 and cp2 with normals vectors n1 and n2 and a penetrated distance of dist.
		void collideWithStaticObject(const Point &cp1, const Point &cp2, const Vector &n1, const Vector &n2, const Vector &dist);
		//! Dynamics for collision with a static object at points cp with normal vector n
		void collideWithStaticObject(const Vector &n, const Point &cp);
		//! Dynamics for collision with that at point cp with a penetrated distance of dist.
		void collideWithObject(PhysicalObject &that, const Point &cp, const Vector &dist);
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
		//! Add a new local interaction, re-sort interaction vector from long ranged to short ranged.
		void addLocalInteraction(LocalInteraction *li);
		//! Add a global interaction, just add it at the end of the vector.
		void addGlobalInteraction(GlobalInteraction *gi) {globalInteractions.push_back(gi);}
		//! Initialize the local interactions, call init on each one.
		virtual void initLocalInteractions();
		//! Do the local interactions with other objects, call objectStep on each one.
		virtual void doLocalInteractions(World *w, PhysicalObject *po, double dt);
		//! Do the local interactions with walls, call wallsStep on each one.
		virtual void doLocalWallsInteraction(World *w);
		//! All the local interactions are finished, call finalize on each one.
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
		//! Simulate a timestep of dt. dt should be below 1 (typically .02-.1); physicsOversampling is the amount of time the physics is run per step, as usual collisions require a more precise simulation than the sensor-motor loop frequency.
		void step(double dt, unsigned physicsOversampling = 1);
		//! Add an object to the world, simply add it to the vector. Object will be automatically deleted when world will be destroyed.
		//! If the object is already in the world, do nothing
		void addObject(PhysicalObject *o);
		//! Remove an object from the world and destroy it. If object is not in the world, do nothing
		void removeObject(PhysicalObject *o);
		//! Set to 0 the userData member of all object whose value userData->deletedWithObject are false; call this before the creator of user data is destroyed, this method is typically called from a viewer just before its destruction.
		void disconnectExternalObjectsUserData();
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
