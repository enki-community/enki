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

#ifndef __ENKI_COLLIDERS_H
#define __ENKI_COLLIDERS_H

#include "System.h"
#include "Types.h"

namespace Enki
{
    struct Entity;
    struct GroundAttachedBody;
    struct RigidBody;
    struct CylinderCollider;
    struct HullCollider;
    
    struct Collider: LocalComponent<RigidBodyPhysics, Collider>
    {
    protected:
        //! The maximum height of the attached entity
		double maximumHeight = 0;
        //! The overall color of the attached entity
		Color color;
        //! Whether we know if we have an attached body
        bool isBodyKnown = false;
        //! A pointer to the RigidBody in this Entity, if any
        RigidBody* rigidBody = nullptr;
        //! A pointer to the GroundAttachedBody in this Entity, if any
        GroundAttachedBody* groundAttachedBody = nullptr;

    public:
        Collider(Entity* owner):
            LocalComponent<RigidBodyPhysics, Collider>(owner)
        {}

        inline double getMaximumHeight() const { return maximumHeight; }
        inline const Color& getColor() const { return color; }

        Simple::Signal<void (const Color&)> onColorChanged;
        
        virtual void init(double dt, RigidBodyPhysics* system);
        
    protected:
        friend class RigidBody;
        friend class CylinderCollider;
        virtual void setupCenterOfMass() = 0;
        virtual double computeMomentOfInertia() = 0;
        
        void collideWithObject(const Collider &that, const Point &cp, const Vector &dist) const;
    };

    struct CylinderCollider: Collider
    {
        virtual void step(double dt, RigidBodyPhysics* system, Collider* that, unsigned thisIndex, unsigned thatIndex);
        
    protected:
        virtual void setupCenterOfMass();
        virtual double computeMomentOfInertia();
        
        void collideWithShape(const HullCollider& that, const Polygone &shape) const;
        void collideWithHull(const HullCollider& that) const;
        void collideWithCylinder(const CylinderCollider& that) const;
    };

    struct HullCollider: Collider
    {
        //! A part is one of the convex geometrical element that composes the physical object
		class Part
		{
		public:
			//! Constructor, builds a shaped part without any texture; shape must be closed and convex.
			Part(const Polygone& shape, double height);
			//! Constructor, builds a shaped part with a textured shape; shape must be closed and convex.
			Part(const Polygone& shape, double height, const Textures& textures);
			//! Constructor, builds a rectangular part of size l1xl2, with a given height and color, and update radius
			Part(double l1, double l2, double height);

			//! Compute the shape of this part wrt a particular rotation and translation
			void applyTransformation(const Matrix22& rot, const Point& trans, double* radius);

			// getters
			inline double getHeight() const { return height; }
			inline double getArea() const { return area; }
			inline const Polygone& getShape() const { return shape; }
			inline const Polygone& getTransformedShape() const { return transformedShape; }
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
			Polygone shape;
			//! The shape of the part in world coordinates, updated on initPhysicsInteractions().
			Polygone transformedShape;
			//! The centroid (barycenter) of the part in object coordinates.
			Point centroid;
			//! The centroid (barycenter) of the part in world coordinates, updated on initPhysicsInteractions().
			Point transformedCentroid;

			// visual properties

			//! Texture for several faces of this object.
			Textures textures;

		private:
            friend class HullCollider;
			//! Compute the area and the centroid (barycenter) of this shape in object coordinates.
			void computeAreaAndCentroid();
			//! Compute the shape of this part in world coordinates with respect to object
			void computeTransformedShape(const Matrix22& rot, const Point& trans);
		};

		//! A hull is a vector of Hull
		struct Hull: std::vector<Part>
		{
			//! Construct an empty hull
			Hull() {}
			//! Construct a hull with a single part
			Hull(const Part& part) : std::vector<Part>(1, part) {}
			//! Return the convex hull of this hull, using a simple Jarvis march/gift wrapping algorithm
			Polygone getConvexHull() const;
			//! Add this hull to another one
			Hull operator+(const Hull& that) const;
			//! Add this hull to another one
			Hull& operator+=(const Hull& that);
			//! Compute the shape of this hull wrt a particular rotation and translation, update the radius if provided
			void applyTransformation(const Matrix22& rot, const Point& trans, double* radius = 0);
		};

    protected:
        //! The hull of this object, which can be composed of several Hull
		Hull hull;

    public:
        HullCollider(Entity* owner, const Hull& hull);
        
        inline const Hull& getHull() const { return hull; }

        //! Make the object rectangular of size l1 x l2 with a given mass
		void setRectangular(double l1, double l2, double height, double mass);
		//! Set a custom shape and mass to the object
		void setCustomHull(const Hull& hull, double mass);

    protected:
        void computeTransformedShape(const Point& absPos, double absYaw);
        virtual void setupCenterOfMass();
        virtual double computeMomentOfInertia();
    };

    struct HollowCylinderCollider: Collider
    {
        // FIXME: TODO
    };

    // Note: this should be generalized for polygon
    struct HollowRectangleCollider: Collider
    {
        // FIXME: TODO
    };
    // struct HollowPolygonCollider: Collider

} // namespace Enki

#endif // __ENKI_COLLIDERS_H
