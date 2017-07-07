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
		
	public:			// inner classes
		
		// User data
		
		//! User specific data that can be attached to any object in the world.
		class UserData
		{
		public:
			bool deletedWithObject; //!< if true, deleted along with the physical object.
			
		public:
			//! Ask the user data to garbage collect itself if appropriate (i.e. not a singleton)
			void deleteIfRequired() { if (deletedWithObject) delete this; }
			//! Virtual destructor, call destructor of child classes
			virtual ~UserData() {}
		};
		
		//! Data attached by the user to this physical object. If non-null, will be destroyed with the object.
		UserData *userData;
		
		// Physics
		
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
		
		// Geometry
		
		//! A part is one of the convex geometrical element that composes the physical object
		class Part
		{
		public:
			//! Constructor, builds a shaped part without any texture; shape must be closed and convex.
			Part(const Polygon& shape, double height);
			//! Constructor, builds a shaped part with a textured shape; shape must be closed and convex.
			Part(const Polygon& shape, double height, const Textures& textures);
			//! Constructor, builds a rectangular part of size l1xl2, with a given height and color, and update radius
			Part(double l1, double l2, double height);
			
			//! Compute the shape of this part wrt a particular rotation and translation
			void applyTransformation(const Matrix22& rot, const Point& trans, double* radius);
			
			// getters
			inline double getHeight() const { return height; }
			inline double getArea() const { return area; }
			inline const Polygon& getShape() const { return shape; }
			inline const Polygon& getTransformedShape() const { return transformedShape; }
			inline const Point& getCentroid() const { return centroid; }
			inline const Point& getTransformedCentroid() const { return transformedCentroid; }
			inline const Textures& getTextures() const { return textures; }
			inline bool isTextured() const { return !textures.empty(); }
			
		private:
			friend class PhysicalObject;
			// geometrical properties
			
			//! The height of the part, used for interaction with the sensors of other robots.
			double height;
			//! The area of this part
			double area;
			//! The shape of the part in object coordinates.
			Polygon shape;
			//! The shape of the part in world coordinates, updated on initPhysicsInteractions().
			Polygon transformedShape;
			//! The centroid (barycenter) of the part in object coordinates.
			Point centroid;
			//! The centroid (barycenter) of the part in world coordinates, updated on initPhysicsInteractions().
			Point transformedCentroid;
			
			// visual properties
			
			//! Texture for several faces of this object.
			Textures textures;
		
		private:
			//! Compute the area and the centroid (barycenter) of this shape in object coordinates.
			void computeAreaAndCentroid();
			//! Compute the shape of this part in world coordinates with respect to object
			void computeTransformedShape(const Matrix22& rot, const Point& trans);
		};
		
		//! A hull is a vector of Hull
		struct Hull:std::vector<Part>
		{
			//! Construct an empty hull
			Hull() {}
			//! Construct a hull with a single part
			Hull(const Part& part) : std::vector<Part>(1, part) {}
			//! Return the convex hull of this hull, using a simple Jarvis march/gift wrapping algorithm
			Polygon getConvexHull() const;
			//! Add this hull to another one
			Hull operator+(const Hull& that) const;
			//! Add this hull to another one
			Hull& operator+=(const Hull& that);
			//! Compute the shape of this hull wrt a particular rotation and translation, update the radius if provided
			void applyTransformation(const Matrix22& rot, const Point& trans, double* radius = 0);
		};
		
	private:		// variables
		
		// Physics
		
		//! position before collision, used to compute interlacedDistance
		Vector posBeforeCollision;
		
		//! How much this object did penetrate other objects in the course of physics steps since last control step
		double interlacedDistance;
		
		// mass and inertia tensor
		
		//! The mass of the object. If below zero, the object can't move (infinite mass).
		double mass;
		///! The moment of inertia tensor
		double momentOfInertia;
		
		// Geometry
		
		//! The hull of this object, which can be composed of several Hull
		Hull hull;
		//! The radius of circular objects or, if hull is not empty, the bounding circle
		double r;
		//! The height of circular object or, if hull is not empty, the maximum height
		double height;
		//! The overall color of this object, if hull is empty or if it does not contain any texture
		Color color;
		
	public:			// methods
		
		//! Constructor
		PhysicalObject();
		//! Destructor
		virtual ~PhysicalObject();
		
		// getters
		
		inline double getRadius() const { return r; }
		inline double getHeight() const { return height; }
		inline bool isCylindric() const { return hull.empty(); }
		inline const Hull& getHull() const { return hull; }
		inline const Color& getColor() const { return color; }
		inline double getMass() const { return mass; }
		inline double getMomentOfInertia() const { return momentOfInertia; }
		inline double getInterlacedDistance() const { return interlacedDistance; }
		
		// setters
		
		//! Make the object cylindric with a given mass
		void setCylindric(double radius, double height, double mass);
		//! Make the object rectangular of size l1 x l2 with a given mass
		void setRectangular(double l1, double l2, double height, double mass);
		//! Set a custom shape and mass to the object
		void setCustomHull(const Hull& hull, double mass);
		//! Set the overall color of this object, if hull is empty or if it does not contain any texture
		void setColor(const Color &color);

		//! A struct with bitfields for buttons
		enum MouseButtonCode
		{
			MOUSE_BUTTON_LEFT = 0,
			MOUSE_BUTTON_RIGHT = 1,
			MOUSE_BUTTON_MIDDLE = 2
		};
		//! Called for robot if a mouse button is pressed while pointing to it, point is given in relative coordinates
		virtual void mousePressEvent(unsigned button, double pointX, double pointY, double pointZ) {};
		//! Called for a robot if a previously mouse button was pressed and is now released
		virtual void mouseReleaseEvent(unsigned button) {};
		
	private:		// setup methods
		
		//! When a physical parameter (color, shape, ...) has been changed, the user data must be updated.
		void dirtyUserData();
		//! Compute the moment of inertia tensor depending on radius, mass, height, and hull, assuming that the hull is centered around the center of mass, which is done by setupCentorOfMass()
		void computeMomentOfInertia();
		//! Compute the center of mass and move bounding surfaces accordingly. Does not update the moment of inertia tensor.
		void setupCenterOfMass();
		//! Compute the hull of this object in world coordinates.
		void computeTransformedShape();
	
	protected:		// physical actions
		
		/*//! A physics simulation step for this object. It is considered as deinterlaced. The position and orientation are updated.
		virtual void physicsStep(double dt);*/
		//! Control step, not oversampled
		virtual void controlStep(double dt);
		//! Apply forces, typically friction to reduce speed, but one can override to change behaviour.
		virtual void applyForces(double dt);
		
		//! The object collided with o during the current physical step, if o is null, it collided with walls. Called just before the object is de-interlaced
		virtual void collisionEvent(PhysicalObject *o) {}
		
		//! Initialize the object specific interactions, do nothing for PhysicalObject.
		virtual void initLocalInteractions(double dt, World* w) { }
		//! Do the interactions with the other PhysicalObject, do nothing for PhysicalObject.
		virtual void doLocalInteractions(double dt, World *w, PhysicalObject *o) { }
		//! Do the interactions with the walls of world w, do nothing for PhysicalObject.
		virtual void doLocalWallsInteraction(double dt, World* w) { }
		//! All interactions are finished, do nothing for PhysicalObject.
		virtual void finalizeLocalInteractions(double dt, World* w) { }

		//! Initialize the global interactions, do nothing for PhysicalObject.
		virtual void initGlobalInteractions(double dt, World* w) { }
		//! Do the global interactions with the world, do nothing for PhysicalObject.
		virtual void doGlobalInteractions(double dt, World* w) { }
		//! All global interactions are finished, do nothing for PhysicalObject.
		virtual void finalizeGlobalInteractions(double dt, World* w) { }

	private:		// physical actions
		
		//! Initialize the collision logic
		void initPhysicsInteractions(double dt);
		//! All collisions are finished, deinterlace the object.
		void finalizePhysicsInteractions(double dt);
		
		//! Dynamics for collision with a static object at points cp with normal vector n
		void collideWithStaticObject(const Vector &n, const Point &cp);
		//! Dynamics for collision with that at point cp (on that) with a penetrated distance of dist,
		void collideWithObject(PhysicalObject &that, Point cp, const Vector &dist);

	public:
		//! ID is used when sharing a world over the network where it should be
		//! possible to associate client objects with their corresponding ones on
		//! the server even if they don't have the same pointer address.
		unsigned int uid;
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
		virtual void initLocalInteractions(double dt, World* w);
		//! Do the local interactions with other objects, call objectStep on each one.
		virtual void doLocalInteractions(double dt, World *w, PhysicalObject *po);
		//! Do the local interactions with walls, call wallsStep on each one.
		virtual void doLocalWallsInteraction(double dt, World* w);
		//! All the local interactions are finished, call finalize on each one.
		virtual void finalizeLocalInteractions(double dt, World* w);
		
		//! Do the global interactions, call step on each one.
		virtual void doGlobalInteractions(double dt, World* w);
		//! Sort local interactions. Called by addLocalInteraction ; can be called by subclasses in case of interaction radius change.
		void sortLocalInteractions(void);
	};

	//! The world is the container of all objects and robots.
	/*! It is either a rectangular arena with walls at all sides, a circular area with walls, or an infinite surface.
		\ingroup core
	*/
	class World
	{
	public:
		//! Type of walls around the world
		enum WallsType
		{
			WALLS_SQUARE = 0,	//!< square walls, use w and h for size
			WALLS_CIRCULAR,		//!< circle walls, use r for radius
			WALLS_NONE			//!< no walls
		};
		
		//! type of walls this world is using
		const WallsType wallsType;
		//! The width of the world, if wallsType is WALLS_SQUARE
		const double w;
		//! The height of the world, if wallsType is WALLS_SQUARE
		const double h;
		//! The radius of the world, if wallsType is WALLS_CIRCLE
		const double r;
		/* Texture of world walls is disabled now, re-enable a proper support if required
		//! Texture of walls.
		Texture wallTextures[4];*/
		//! The color of the world walls and ground
		const Color color;
		
		//! 2-D Texture for ground
		struct GroundTexture
		{
			//! the width of the ground texture, if any
			unsigned width;
			//! the height of the ground texture, if any
			unsigned height;
			//! the date of the ground texture, organised as scanlines of pixels in ARGB (0xAARRGGBB in little endian); empty if there is no ground texture
			std::vector<uint32_t> data;
			
			//! build an empty texture
			GroundTexture();
			//! build a texture from an existing pointer
			GroundTexture(unsigned width, unsigned height, const uint32_t* data);
		};
		
		//! Current ground texture
		const GroundTexture groundTexture;
		
		typedef std::set<PhysicalObject *> Objects;
		typedef Objects::iterator ObjectsIterator;
		
		//! Whether the world should delete the objects upon destruction, true by default
		bool takeObjectOwnership;
		
		//! All the objects in the world
		Objects objects;
		//! Base for the Bluetooth connections between robots
		BluetoothBase* bluetoothBase;

	protected:
		//! Collide two objects. Correct functions will be called depending on type of object (circular or other shape).
		void collideObjects(PhysicalObject *object1, PhysicalObject *object2);
		//! Collide the object with square walls.
		void collideWithSquareWalls(PhysicalObject *object);
		//! Collide the object with circular walls.
		void collideWithCircularWalls(PhysicalObject *object);

	public:
		//! Construct a world with square walls, takes width and height of the world arena in cm.
		World(double width, double height, const Color& wallsColor = Color::gray, const GroundTexture& groundTexture = GroundTexture());
		//! Construct a world with circle walls, takes radius of the world arena in cm.
		World(double r, const Color& wallsColor = Color::gray, const GroundTexture& groundTexture = GroundTexture());
		//! Construct a world with no walls
		World();
		//! Destructor, destroy all objects
		virtual ~World();
		
		//! Return whether the ground has a texture
		bool hasGroundTexture() const;
		//! Return the color of the ground at a given point, or white.
		Color getGroundColor(const Point& p) const;
		
		//! Simulate a timestep of dt. dt should be below 1 (typically .02-.1); physicsOversampling is the amount of time the physics is run per step, as usual collisions require a more precise simulation than the sensor-motor loop frequency.
		virtual void step(double dt, unsigned physicsOversampling = 1);
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
	
	protected:
		//! Can implement world specific control. By default do nothing
		virtual void controlStep(double dt) { }
	};
	
	//! Fast random for use by Enki
	extern FastRandom random;
}

#endif
