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

#ifndef __ENKI_RIGID_BODY_PHYSICS_H
#define __ENKI_RIGID_BODY_PHYSICS_H

#include "System.h"
#include "Types.h"
#include "Colliders.h"

namespace Enki
{
    class RigidBodyPhysics;
    class GroundAttachedBody;
    
    struct KinematicBody: GlobalComponent<RigidBodyPhysics>
    {
        // space coordinates derivatives

        //! The speed of the object.
        Vector speed;
        //! The rotation speed of the object, standard trigonometric orientation.
        double angSpeed;
        
        //! Initialize the collision logic
        virtual void initPhysics(double dt, RigidBodyPhysics* system);
        //! All collisions are finished, deinterlace the object.
        virtual void finalizePhysics(double dt, RigidBodyPhysics* system);
    
    protected:
        friend class Collider;
        virtual void collideWithStaticObject(const Vector &n, const Point &cp) = 0;
        virtual void collideWithGroundAttachedBody(GroundAttachedBody* that, const Point &cp, const Vector &dist) = 0;
        virtual void collideWithRigidBody(RigidBody* that, const Point &cp, const Vector &dist) = 0;
    };
    
    struct MassiveBody: KinematicBody
    {
    protected:
        // mass
        
        //! The mass of the object, must be positive
        double mass;
        
    public:
        inline double getMass() const { return mass; }
    };
    
    struct GroundAttachedBody: MassiveBody
    {
        
    };

    struct RigidBody: MassiveBody
    {
        // physical constant
		static const double g;

        //! Elasticity of collisions of this object. If 0, soft collision, 100% energy dissipation; if 1, elastic collision, 0% energy dissipation. Actual elasticity is the product of the elasticity of the two colliding objects. Walls are fully elastics
	    double collisionElasticity;
	    //! The dry friction coefficient mu.
	    double dryFrictionCoefficient;
	    //! The viscous friction coefficient. Premultiplied by mass. A value of k applies a force of -k * speed * mass
	    double viscousFrictionCoefficient;
	    //! The viscous friction moment coefficient. Premultiplied by momentOfInertia. A value of k applies a force of -k * speed * momentOfInertia
	    double viscousMomentFrictionCoefficient;

        //! How much this object was forcely moved because of penetration
		double accumulatedInterlacedDistance;

    protected:
        // inertia tensor

        //! Whether the moment of inertia was computed, if not, do it at next time step
        bool isMomentOfInertiaComputed = false;
        
        //! The moment of inertia tensor
        double momentOfInertia = 1.0;

        //! position before collision, used to compute accumulatedInterlacedDistance
		Point posBeforeCollision;

    public:
        // getters
        inline double getMomentOfInertia() const { return momentOfInertia; }
		inline double getAccumulatedInterlacedDistance() const { return accumulatedInterlacedDistance; }

        //! Initialize the collision logic
        virtual void initPhysics(double dt, RigidBodyPhysics* system);
        //! All collisions are finished, deinterlace the object.
        virtual void finalizePhysics(double dt, RigidBodyPhysics* system);
    
    protected:
        //! Apply friction forces to reduce speed
		void applyForces(double dt);
        //! Compute the moment of inertia using colliders
        void computeMomentOfInertia();
    };


    struct RigidBodyPhysics: System
    {
        /**
            This phase updates the rigid bodies:
            - apply forces
            - update pose from speed
            - store pose before collision
            - init temporary pose
        */
        typedef GlobalPhase<RigidBodyPhysics, KinematicBody> InitPhaseBase;
        struct InitPhase: InitPhaseBase
        {
            using InitPhaseBase::InitPhaseBase;

            virtual void step(double dt);
        };

        typedef LocalPhase<RigidBodyPhysics, Collider, Collider> CollisionPhaseBase;
        /**
            This phase performs object to object collisions using the collider components:
            - update shapes of rigid colliders
            - detect collisions
            - perform rigid body physics
            - update temporary pose in Rigid bodies
        */
        struct CollisionPhase: CollisionPhaseBase
        {
            using CollisionPhaseBase::CollisionPhaseBase;
        };

        typedef GlobalPhase<RigidBodyPhysics, KinematicBody> FinalizePhaseBase;
        /**
            This phase finalizes the physics
            - compute deinterlace distance
            - update pose from temporary pose
        */
        struct FinalizePhase: FinalizePhaseBase
        {
            using FinalizePhaseBase::FinalizePhaseBase;

            virtual void step(double dt);
        };

        RigidBodyPhysics(World* world);
    };
} // namespace Enki

#endif // __ENKI_RIGID_BODY_PHYSICS_H
