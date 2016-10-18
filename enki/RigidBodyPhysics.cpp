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

#include <cassert>
#include "RigidBodyPhysics.h"

namespace Enki
{
    void RigidBody::initPhysics(double dt, RigidBodyPhysics* system)
    {
        // FIXME: implement
        //applyForces(dt);

        owner->setPose(owner->getPos() + speed * dt, owner->getYaw() + angSpeed * dt);

        posBeforeCollision  = owner->getPos();
    }

    void RigidBody::finalizePhysics(double dt, RigidBodyPhysics* system)
    {
        // TODO: work on temporary pose instead of owner->getPos()
        accumulatedInterlacedDistance += (posBeforeCollision - owner->getPos()).norm();
    }

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
