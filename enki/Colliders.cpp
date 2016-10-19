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
#include "RigidBodyPhysics.h"

namespace Enki
{
    void Collider::init(double dt, RigidBodyPhysics* system)
    {
        if (!isBodyKnown)
        {
            groundAttachedBody = getSiblingComponentPtr<GroundAttachedBody>();
            rigidBody = getSiblingComponentPtr<RigidBody>();
            isBodyKnown = true;
        }
    }
    
    void Collider::collideWithObject(const Collider &that, const Point &cp, const Vector &dist) const
    {
        // dispatch collision physics of different types
        if (rigidBody)
        {
            if (that.rigidBody)
                rigidBody->collideWithRigidBody(that.rigidBody, cp, dist);
            else if (that.groundAttachedBody)
                rigidBody->collideWithGroundAttachedBody(that.groundAttachedBody, cp, dist);
            else
                rigidBody->collideWithStaticObject(cp, dist);
        }
        else if (groundAttachedBody)
        {
            if (that.rigidBody)
                groundAttachedBody->collideWithRigidBody(that.rigidBody, cp, dist);
            else if (that.groundAttachedBody)
                groundAttachedBody->collideWithGroundAttachedBody(that.groundAttachedBody, cp, dist);
            else
                groundAttachedBody->collideWithStaticObject(cp, dist);
        }
        else
        {
            if (that.rigidBody)
                that.rigidBody->collideWithStaticObject(cp, -dist);
            else if (that.groundAttachedBody)
                that.groundAttachedBody->collideWithStaticObject(cp, -dist);
            else
                ; // do nothing
        }
    }
    
    //
    
    void CylinderCollider::step(double dt, RigidBodyPhysics* system, Collider* that, unsigned thisIndex, unsigned thatIndex)
    {
        if (thisIndex >= thatIndex)
            return;
            
        const HullCollider* thatHullCollider(dynamic_cast<const HullCollider*>(that));
        if (thatHullCollider)
        {
            collideWithHull(*thatHullCollider);
            return;
        }
        
        const CylinderCollider* thatCylinderCollider(dynamic_cast<const CylinderCollider*>(that));
        if (thatCylinderCollider)
        {
            collideWithCylinder(*thatCylinderCollider);
            return;
        }
        
        assert(false);
    }
    
    void CylinderCollider::setupCenterOfMass()
    {
        // do nothing for cylinder
    }
    
    double CylinderCollider::computeMomentOfInertia()
    {
        return 0.5 * r * r;
    }
    
    void CylinderCollider::collideWithShape(const HullCollider& that, const Polygone &shape) const
    {
        const Vector pos(getPos());
        // test if circularObject is inside a shape
		for (unsigned i=0; i<shape.size(); i++)
		{
			const size_t next=(i+1)%shape.size();
			const Segment s(shape[i].x, shape[i].y, shape[next].x, shape[next].y);

			const Vector nn(s.a.y-s.b.y, s.b.x-s.a.x);	//orthog. vector
			const Vector u = nn.unitary();

			const double d = (pos-s.a)*u;
			// if we are inside the circularObject
			if ((d<0) && (-d<r))
			{
				const Point proj = pos - u*d;

				if ((((proj-s.a)*(s.b-s.a))>0) && (((proj-s.b)*(s.a-s.b))>0))
				{
					// if there is a segment which is inside this, and the projection of the center lies within this segment, this projection is the nearest point. So we return. This is a consequence of having convexe polygones.
					const Vector dist = u*-(r+d);
					const Point collisionPoint = pos - u*(d);
					collideWithObject(that, collisionPoint, dist);
					return;
				}
			}
		}

		const double r2 = r * r;
		double pointInsideD2 = r2;
		Point pointInside;
		Vector centerToPointInside;
		
		// test if there is vertex of shape is inside the circularObject. If so, take the closest to the center
		for (unsigned i=0; i<shape.size(); i++)
		{
			const Point &candidate = shape[i];
			const Vector centerToPoint = candidate - pos;
			const double d2 = centerToPoint.norm2();
			if (d2 < pointInsideD2)
			{
				pointInsideD2 = d2;
				pointInside = candidate;
				centerToPointInside = centerToPoint;
			}
		}

		// we get a collision, one point of shape is inside this
		if (pointInsideD2 < r2)
		{
			const double pointInsideDist = sqrt(pointInsideD2);
			const Vector dist = (centerToPointInside / pointInsideDist) * (r - pointInsideDist);
			const Point collisionPoint = pointInside + dist;
			that.collideWithObject(*this, collisionPoint, dist);
		}
    }
    
    void CylinderCollider::collideWithHull(const HullCollider& that) const
    {
        // collide this circle on that hull, iterate over parts
        for (auto part: that.getHull())
            collideWithShape(that, part.getTransformedShape());
    }
    
    void CylinderCollider::collideWithCylinder(const CylinderCollider& that) const
    {
        // collide this and that cylinders together
        const Vector distOCtoOC = this->getPos() - that.getPos();
        const Vector ud = distOCtoOC.unitary();
        const double dLength = distOCtoOC.norm();
        const double addedRadius = this->r + that.r;
        const Vector dist = ud * (addedRadius - dLength);
        const Vector collisionPoint = that.getPos() + ud * that.r;
        collideWithObject(that, collisionPoint, dist);
    }

    void HullCollider::Part::computeTransformedShape(const Matrix22& rot, const Point& trans)
	{
		assert(!shape.empty());
		assert(transformedShape.size() == shape.size());
		for (size_t i = 0; i < shape.size(); ++i)
			transformedShape[i] = rot * (shape)[i] + trans;
		transformedCentroid = rot * centroid + trans;
	}

    HullCollider::HullCollider(Entity* owner, const Hull& hull):
        Collider(owner),
        hull(hull)
    {
        // recompute the maximal height
		for (Hull::const_iterator it = hull.begin(); it != hull.end(); ++it)
			maximumHeight = std::max(maximumHeight, it->getHeight());
        
        owner->onAbsPoseChanged.connect(Simple::slot(this, &HullCollider::computeTransformedShape));
    }

    void HullCollider::computeTransformedShape(const Point& absPos, double absYaw)
    {
        Matrix22 absRotMat(absYaw);
        for (Hull::iterator it = hull.begin(); it != hull.end(); ++it)
            it->computeTransformedShape(absRotMat, absPos);
    }
    
    void HullCollider::setupCenterOfMass()
    {
        // Exact method using the centroid of parts
		Point cm;
		double area = 0;
		for (Hull::iterator it = hull.begin(); it != hull.end(); ++it)
		{
			const Part& part = *it;
			const double partArea = part.getArea();
			cm += part.getCentroid() * partArea;
			area += partArea;
		}
		cm /= area;
		
		// shift all shapes to the CM
		hull.applyTransformation(Matrix22::identity(), -cm, &r);
        // adjust pos of object
        owner->setPos(getPos() +  getYawMatrix() * cm);
    }
    
    double HullCollider::computeMomentOfInertia()
    {
        // Numerical method:
        // arbitrary shaped object, numerically compute moment of inertia
        double momentOfInertia = 0;
        double numericalArea = 0;
        const double dr = r / 50.;
        for (double ix = -r; ix < r; ix += dr)
            for (double iy = -r; iy < r; iy += dr)
                for (Hull::const_iterator it = hull.begin(); it != hull.end(); ++it)
                    if (it->shape.isPointInside(Point(ix, iy)))
                    {
                        momentOfInertia += ix * ix + iy * iy;
                        numericalArea++;
                    }
        // Note: we assume that the total mass is split between parts,
        // and that if parts overlap, their partial mass add
        
        return momentOfInertia / numericalArea;
        
        // Exact method:
        /*
        TODO: check this and implement, probably this can use the centroids of the parts
        http://lab.polygonal.de/2006/08/17/calculating-the-moment-of-inertia-of-a-convex-polygon/
        */
    }

} // namespace Enki
