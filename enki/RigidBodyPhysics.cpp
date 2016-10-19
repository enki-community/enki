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

#include <iostream>
#include <cassert>
#include "RigidBodyPhysics.h"

namespace Enki
{
    using namespace std;
    
    //
    
    void KinematicBody::initPhysics(double dt, RigidBodyPhysics* system)
    {
        owner->setPose(getPos() + speed * dt, getYaw() + angSpeed * dt);
    }

    void KinematicBody::finalizePhysics(double dt, RigidBodyPhysics* system)
    {
        // do nothing for purely kinematic bodies
    }
    
    void RigidBody::initPhysics(double dt, RigidBodyPhysics* system)
    {
        if (!isMomentOfInertiaComputed)
            computeMomentOfInertia();
        applyForces(dt);
        MassiveBody::initPhysics(dt, system);
        posBeforeCollision  = getPos();
    }

    void RigidBody::finalizePhysics(double dt, RigidBodyPhysics* system)
    {
        MassiveBody::finalizePhysics(dt, system);
        
        // TODO: work on temporary pose instead of getPos()
        accumulatedInterlacedDistance += (posBeforeCollision - getPos()).norm();
    }
    
    void RigidBody::applyForces(double dt)
	{
		Vector acc = 0.;
		double angAcc = 0.;
		
		// dry friction, set speed to zero if bigger
		const Vector dryFriction = - speed.unitary() * g * dryFrictionCoefficient;
		if ((dryFriction * dt).norm2() > speed.norm2())
			speed = 0.;
		else
			acc += dryFriction;
		
		// dry rotation friction, set angSpeed to zero if bigger
		const double dryAngFriction = copysign(g * dryFrictionCoefficient, -angSpeed);
		if ((fabs(dryAngFriction) * dt) > fabs(angSpeed))
			angSpeed = 0.;
		else
			angAcc += dryAngFriction;
		
		// viscous friction
		acc += - speed * viscousFrictionCoefficient;
		angAcc += - angSpeed * viscousMomentFrictionCoefficient;
		
		// el cheapos integration
		speed += acc * dt;
		angSpeed += angAcc * dt;
	}
    
    void RigidBody::computeMomentOfInertia()
    {
        try
        {
            Collider& collider(getSiblingComponent<Collider>());
            collider.setupCenterOfMass();
            momentOfInertia = mass * collider.computeMomentOfInertia();
        }
        catch (Entity::ComponentNotFound& e)
        {
            cerr << "The entity owning this RigidBody has no collider!" << endl;
        }
        isMomentOfInertiaComputed = true;
    }

    //

    void RigidBodyPhysics::InitPhase::step(double dt)
    {
        for (auto rigidBody: components)
            rigidBody->initPhysics(dt, system);
    }

    void RigidBodyPhysics::FinalizePhase::step(double dt)
    {
        for (auto rigidBody: components)
            rigidBody->finalizePhysics(dt, system);
    }

    RigidBodyPhysics::RigidBodyPhysics(World* world):
        System(world)
    {
        phases.emplace_back(new InitPhase(world, this));
        phases.emplace_back(new CollisionPhase(world, this));
        phases.emplace_back(new FinalizePhase(world, this));
    }

} // namespace Enki
