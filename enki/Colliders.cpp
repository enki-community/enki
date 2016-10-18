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

#include "Colliders.h"

namespace Enki
{
    void Collider::init(double dt, RigidBodyPhysics* system)
    {
    }

    void Collider::step(double dt, RigidBodyPhysics* system, Collider* that)
    {

    }

    void Collider::finalize(double dt, RigidBodyPhysics* system)
    {

    }

    void HullCollider::Part::computeTransformedShape(const Matrix22& rot, const Point& trans)
	{
		assert(!shape.empty());
		assert(transformedShape.size() == shape.size());
		for (size_t i = 0; i < shape.size(); ++i)
			transformedShape[i] = rot * (shape)[i] + trans;
		transformedCentroid = rot * centroid + trans;
	}

    HullCollider::HullCollider(Entity* owner):
        Collider(owner)
    {
        owner->onAbsPoseChanged.connect(Simple::slot(this, &HullCollider::computeTransformedShape));
    }

    void HullCollider::computeTransformedShape(const Point& absPos, double absYaw)
    {
        Matrix22 absRotMat(absYaw);
        for (Hull::iterator it = hull.begin(); it != hull.end(); ++it)
            it->computeTransformedShape(absRotMat, absPos);
    }

} // namespace Enki
