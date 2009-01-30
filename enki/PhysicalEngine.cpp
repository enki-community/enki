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

#include "PhysicalEngine.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <algorithm>
#include <limits>

// _________________________________
//
// Implementation of PhysicalObject
// _________________________________
// All objects in the world are derived from a PhysicalObject

namespace Enki
{
	FastRandom random;
	
	// PhysicalObject::Part
	
	PhysicalObject::Part::Part(const Polygone& shape, double height) :
		height(height),
		shape(shape)
	{
		transformedShape.resize(shape.size());
	}
	
	PhysicalObject::Part::Part(const Polygone& shape, double height, const Textures& textures) :
		height(height),
		shape(shape),
		textures(textures)
	{
		transformedShape.resize(shape.size());
		
		if (textures.size() != shape.size())
		{
			std::cerr << "Error: PhysicalObject::Part::Part: texture sides count " << textures.size() << " missmatch shape sides count " << shape.size() << std::endl;
			std::cerr << "\tignoring textures for this object" << std::endl;
			this->textures.clear();
			return;
		}
		
		for (size_t i = 0; i < textures.size(); ++i)
		{
			if (textures[i].size() == 0)
			{
				std::cerr << "Error: PhysicalObject::Part::Part: texture for side " << i << " contains no data" << std::endl;
				std::cerr << "\tignoring textures for this object" << std::endl;
				this->textures.clear();
				return;
			}
		}
	}
	
	PhysicalObject::Part::Part(double l1, double l2, double height) :
		height(height)
	{
		double hl1 = l1 / 2;
		double hl2 = l2 / 2;
		shape << Point(-hl1, -hl2) << Point(hl1, -hl2) << Point(hl1, hl2) << Point(-hl1, hl2);
		transformedShape.resize(shape.size());
	}
	
	void PhysicalObject::Part::updateRadius(double& radius)
	{
		for (size_t i = 0; i < 4; i++)
			radius = std::max(radius, shape[i].norm());
	}
		
	void PhysicalObject::Part::computeTransformedShape(const Matrix22& rot, const Point& trans)
	{
		assert(!shape.empty());
		assert(transformedShape.size() == shape.size());
		for (size_t i = 0; i < shape.size(); ++i)
			transformedShape[i] = rot * (shape)[i] + trans;
	}
	
	
	// PhysicalObject
	
	const double PhysicalObject::g = 9.81;
	
	PhysicalObject::PhysicalObject(void) :
		userData(NULL),
		// default physical parameters
		collisionElasticity(0.9),
		dryFrictionCoefficient(0.25),
		viscousFrictionCoefficient(0.01),
		viscousMomentFrictionCoefficient(0.01),
		angle(0),
		angSpeed(0),
		infraredReflectiveness(1)
	{
		setCylindric(1, 1, 1);
	}
	
	PhysicalObject::~PhysicalObject(void)
	{
		if (userData && (userData->deletedWithObject))
		{
			delete userData;
		}
	}
	
	void PhysicalObject::setCylindric(double radius, double height, double mass)
	{
		// remove any hull
		hull.clear();
		this->height = height;
		
		// update the physical and interaction radius
		r = radius;
		
		// set the mass
		this->mass = mass;
		
		// update the moment of inertia
		computeMomentOfInertia();
	}
	
	void PhysicalObject::setRectangular(double l1, double l2, double height, double mass)
	{
		// assign a new hull
		hull.resize(1, Part(l1, l2, height));
		this->height = height;
		
		// compute the center of mass
		setupCenterOfMass();
		
		// set the mass
		this->mass = mass;
		
		// update the moment of inertia
		computeMomentOfInertia();
	}
	
	void PhysicalObject::setCustomHull(const Parts& hull, double mass)
	{
		// assign the new hull
		this->hull = hull;
		height = 0;
		for (Parts::const_iterator it = hull.begin(); it != hull.end(); ++it)
			height = std::max(height, it->getHeight());
		
		// compute the center of mass
		setupCenterOfMass();
		
		// set the mass
		this->mass = mass;
		
		// update the moment of inertia
		computeMomentOfInertia();
	}
	
	void PhysicalObject::setColor(const Color &color)
	{
		this->color = color;
	}
	
	void PhysicalObject::setInfraredReflectiveness(double value)
	{
		this->infraredReflectiveness = value;
	}
	
	void PhysicalObject::computeMomentOfInertia()
	{
		if (hull.empty())
		{
			momentOfInertia = 0.5 * mass * r * r;
		}
		else
		{
			// arbitrary shaped object, numerically compute moment of inertia
			momentOfInertia = 0;
			double area = 0;
			double dr = r / 50.;
			for (double ix = -r; ix < r; ix += dr)
				for (double iy = -r; iy < r; iy += dr)
					for (Parts::const_iterator it = hull.begin(); it != hull.end(); ++it)
						if (it->shape.isPointInside(Point(ix, iy)))
						{
							momentOfInertia += ix * ix + iy * iy;
							area++;
						}
			
			momentOfInertia *= mass / area;
		}
	}
	
	void PhysicalObject::setupCenterOfMass()
	{
		if (hull.empty())
			return;
		
		// get bounding box of the whole hull
		Point bottomLeft, topRight;
		Parts::iterator it = hull.begin();
		bool validBB = it->shape.getAxisAlignedBoundingBox(bottomLeft, topRight);
		assert(validBB);
		++it;
		for (;it != hull.end(); ++it)
			it->shape.extendAxisAlignedBoundingBox(bottomLeft, topRight);
		
		// numerically compute the center of mass of the shape
		Point cm;
		double area = 0;
		double dx = (topRight-bottomLeft).x / 100;
		double dy = (topRight-bottomLeft).y / 100;
		for (double ix = bottomLeft.x; ix < topRight.x; ix += dx)
			for (double iy = bottomLeft.y; iy < topRight.y; iy += dy)
				for (it = hull.begin(); it != hull.end(); ++it)
					if (it->shape.isPointInside(Point(ix, iy)))
					{
						cm.x += ix;
						cm.y += iy;
						area++;
					}
		assert(area != 0);
		cm /= area;
		area = area / (dx * dy);
		
		// shift all shapes to the CM
		pos += cm;
		r = 0;
		for (it = hull.begin(); it != hull.end(); ++it)
		{
			it->shape.translate(-cm);
			it->updateRadius(r);
		}
	}
	
	void PhysicalObject::computeTransformedShape()
	{
		if (!hull.empty())
		{
			Matrix22 rotMat(angle);
			for (Parts::iterator it = hull.begin(); it != hull.end(); ++it)
				it->computeTransformedShape(rotMat, pos);
		}
	}
	
	
	
	static double sgn(double v)
	{
		if (v > 0)
			return 1;
		else if (v < 0)
			return -1;
		else
			return 0;
	}

	void PhysicalObject::physicsStep(double dt)
	{
		applyForces(dt);
		
		pos += speed * dt;
		angle += angSpeed * dt;
		
		/* TODO: Runge-Kutta
			but this needs a refactoring in order to harvest equations up to now.
			furthermore, we have a so simple model that it is seldom useful for now.
		xn+1 = xn + h⁄6 (a + 2 b + 2 c + d)  where 
		a = f (tn, xn)
		b = f (tn + h⁄2, xn + h⁄2 a)
		c = f (tn + h⁄2, xn + h⁄2 b)
		d = f (tn + h, xn + h c)
		*/
		
		angle = normalizeAngle(angle);
	}
	
	void PhysicalObject::applyForces(double dt)
	{
		/*
		Temporary not used as there is no intrinsic force for now
		The only force available are the friction ones below
		// static friction
		const double minSpeedForMovement = 0.001;
		if ((speed.norm2() < minSpeedForMovement * minSpeedForMovement) && 
			(abs(angSpeed) < minSpeedForMovement) &&
			(acc.norm2() * mass < staticFrictionThreshold * staticFrictionThreshold) &&
			(angAcc * momentOfInertia < staticFrictionThreshold))
		{
			// fully stop movement
			acc = 0.;
			angAcc = 0.;
			speed = 0.;
			angSpeed = 0.;
			return;
		}*/
		
		Vector acc = 0.;
		double angAcc = 0.;
		
		// dry friction, set speed to zero if bigger
		Vector dryFriction = - speed.unitary() * g * dryFrictionCoefficient;
		if ((dryFriction * dt).norm2() > speed.norm2())
			speed = 0.;
		else
			acc += dryFriction;
		
		// dry rotation friction, set angSpeed to zero if bigger
		double dryAngFriction = - sgn(angSpeed) * g * dryFrictionCoefficient;
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

	void PhysicalObject::initPhysicsInteractions()
	{
		computeTransformedShape();
	}

	void PhysicalObject::finalizePhysicsInteractions(double dt)
	{
		// do nothing for now
	}
	
	
	
	void PhysicalObject::collideWithStaticObject(const Vector &n, const Point &cp)
	{
		// only perform physics if we are in a physically-realistic collision situation,
		if (n * speed > 0)
		{
			//std::cerr << this << " Warning collideWithStaticObject " << std::endl; 
			return;
		}
		
		// from http://www.myphysicslab.com/collision.html
		Vector r_ap = (cp - pos);
		Vector v_ap = speed + r_ap.crossFromZVector(angSpeed);
		double num = -(1 + collisionElasticity) * (v_ap * n);
		double denom = (1 / mass) + (r_ap.cross(n) * r_ap.cross(n)) / momentOfInertia;
		double j = num / denom;
		speed += (n * j) / mass;
		angSpeed += r_ap.cross(n * j) / momentOfInertia;
	}

	void PhysicalObject::collideWithObject(PhysicalObject &that, const Point &cp, const Vector &dist)
	{
		// handle infinite mass case
		if (mass < 0)
		{
			if (that.mass < 0)
			{
				//assert(false);
				return;
			}
			else
			{
				// if colliding with wall
				Vector n = dist.unitary() * -1;
				that.collideWithStaticObject(n, cp);
				that.pos -= dist;
				return;
			}
		}
		else
		{
			// if colliding with wall
			if (that.mass < 0)
			{
				Vector n = dist.unitary();
				collideWithStaticObject(n, cp);
				pos += dist;
				return;
			}
		}
		
		if ((dist * speed > 0) || (dist * that.speed < 0))
		{
			//std::cerr << this << " Warning collideWithObject" << std::endl;
		}
		
		// only perform physics if we are in a physically-realistic collision situation,
		// otherwise we experience a simulation artefact and we just deinterlace
		if ((dist * speed <= 0) && (dist * that.speed >= 0))
		{
			// point of this in inside object
			// we use model from http://www.myphysicslab.com/collision.html
			// this is object A, that is object B
			Vector n = dist.unitary();
			
			Vector r_ap = (cp - pos);
			Vector r_bp = (cp - that.pos);
			
			Vector v_ap = speed + r_ap.crossFromZVector(angSpeed);
			Vector v_bp = that.speed + r_bp.crossFromZVector(angSpeed);
			Vector v_ab = v_ap - v_bp;
			
			double num = -(1 + collisionElasticity * that.collisionElasticity) * (v_ab * n);
			double denom = (1/mass) + (1/that.mass) + (r_ap.cross(n) * r_ap.cross(n)) / momentOfInertia + (r_bp.cross(n) * r_bp.cross(n)) / that.momentOfInertia;
			double j = num / denom;
			
			speed += (n * j) / mass;
			that.speed -= (n * j) / that.mass;
			angSpeed += r_ap.cross(n * j) / momentOfInertia;
			that.angSpeed -= r_bp.cross(n * j) / that.momentOfInertia;
		}
		
		// calculate deinterlace vector to put that out of contact - mass ratios ensure physics
		double massSum = mass + that.mass;
		pos += dist*that.mass/massSum;
		that.pos -= dist*mass/massSum;
	}

	//! A functor then compares the radius of two local interactions
	struct InteractionRadiusCompare
	{
		//! Return true if li1->r is smaller then li2->r, false otherwise
		bool operator()(const LocalInteraction *li1,const LocalInteraction *li2)
		{
			return li1->getRange() > li2->getRange();
		}
	};


	void Robot::addLocalInteraction(LocalInteraction *li)
	{
		localInteractions.push_back(li);
		sortLocalInteractions();
	}
	
	void Robot::sortLocalInteractions(void)
	{
		// instantiate the compare function object
		InteractionRadiusCompare irCompare;
		// sort the interaction by r
		std::sort(localInteractions.begin(), localInteractions.end(), irCompare);
	}

	void Robot::initLocalInteractions()
	{
		for (size_t i=0; i<localInteractions.size(); i++ )
		{
			localInteractions[i]->init();
		}
	}


	void Robot::doLocalInteractions(World *w, PhysicalObject *po, double dt)
	{
		for (size_t i=0; i<localInteractions.size(); i++)
		{
			Vector vectCenter(this->pos.x - po->pos.x, this->pos.y - po->pos.y );
			if (vectCenter.norm2() <  (localInteractions[i]->r+po->getRadius())*(localInteractions[i]->r+po->getRadius()))
				localInteractions[i]->objectStep(dt, po, w);
			else 
				return;
		}
	}


	void Robot::doLocalWallsInteraction(World *w)
	{
		for (size_t i=0; i<localInteractions.size(); i++)
		{
			if ((this->pos.x>localInteractions[i]->r) && (this->pos.y>localInteractions[i]->r) && (w->w-this->pos.x>localInteractions[i]->r) && (w->h-this->pos.y>localInteractions[i]->r))
				return;
			else
				localInteractions[i]->wallsStep(w);
		}
	}

	void Robot::finalizeLocalInteractions(double dt)
	{
		for (size_t i=0; i<localInteractions.size(); i++ )
		{
			localInteractions[i]->finalize(dt);
		}
	}

	void Robot::doGlobalInteractions(World *w, double dt)
	{
		for (size_t i=0; i<globalInteractions.size(); i++)
		{
			globalInteractions[i]->step(dt, w);
		}
	}

	World::World(double width, double height) :
		w(width),
		h(height),
		bluetoothBase(NULL)
	{
		collideEven = true;
		useWalls = true;
		
		// walls are gray
		setWallsColor(Color::gray);
	}

	World::~World()
	{
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			delete (*i);
		
		if (bluetoothBase)
			delete bluetoothBase;
	}
	
	void World::setWallsColor(const Color& color)
	{
		for (size_t i = 0; i < 4; i++)
			wallTextures[i].resize(1, color);
	}

	bool World::isPointInside(const Point &p, const Point &c, const Polygone &bs, Vector *distVector)
	// p = candidate point of object; c = pos of object; bs = bounding surface of other object; distVector = deinterlacing dist to be calculated
	{
		Vector centerToPoint = p-c;
		double score = -10;
		for (size_t i=0; i<bs.size(); i++)
		{
			unsigned next=(i+1)%bs.size();
			Segment seg(bs[i].x, bs[i].y, bs[next].x, bs[next].y);

			Vector nn(seg.a.y-seg.b.y, seg.b.x-seg.a.x);	//orthog. vector for segment
			Vector u = nn.unitary();

			double d = (p-seg.a)*u;		//distance it is inside; if neg. it is outside
			if (d<0)
			{
				return false;
			}
			else
			{
				double newScore = -d;
				if (newScore >= score)
				{
					Vector dv = u * (-d);
					if (dv * centerToPoint < 0)
					{
						*distVector = dv;
						score = newScore;
					}
				}
			}
		}
		return true;
	}

	void World::collideWithWalls(PhysicalObject *object)
	{
		// object is circle only
		if (object->hull.empty())
		{
			double x = object->pos.x;
			double y = object->pos.y;
			double r = object->r;
			if (x-r < 0)
			{
				object->collideWithStaticObject(Vector(1, 0), Vector(0, y));
				object->pos.x += r-x;
			}
			if (y-r < 0)
			{
				object->collideWithStaticObject(Vector(0, 1), Vector(x, 0));
				object->pos.y += r-y;
			}
			if (x+r > w)
			{
				object->collideWithStaticObject(Vector(-1, 0), Vector(w, y));
				object->pos.x += w-(x+r);
			}
			if (y+r > h)
			{
				object->collideWithStaticObject(Vector(0, -1), Vector(x, h));
				object->pos.y += h-(y+r);
			}
		}
		else
		{
			// iterate over all shapes
			for (PhysicalObject::Parts::const_iterator it = object->hull.begin(); it != object->hull.end(); ++it)
			{
				const Polygone& shape = it->getTransformedShape();
				
				// let's assume walls are infinite
				Point cp1, cp2; // cp1 is on x, cp2 is on y
				Vector cp;
				
				double dist = 0;
				double n = 0;
				for (size_t i=0; i<shape.size(); i++)
				{
					double x = shape[i].x;
					double y = shape[i].y;
					if (x < -dist)
					{
						dist = -x;
						cp.x = 0;
						cp.y = y;
						n = 1;
					}
					if (x-w > -dist)
					{
						dist = w-x;
						cp.x = w;
						cp.y = y;
						n = -1;
					}
				}
				if (dist != 0)
				{
					object->collideWithStaticObject(Vector(n, 0), cp);
					object->pos.x += dist;
				}
				
				dist = 0;
				n = 0;
				for (size_t i=0; i<shape.size(); i++)
				{
					double x = shape[i].x;
					double y = shape[i].y;
					if (y < -dist)
					{
						dist = -y;
						cp.x = x;
						cp.y = 0;
						n = 1;
					}
					if (y-h > -dist)
					{
						dist = h-y;
						cp.x = x;
						cp.y = h;
						n = -1;
					}
				}
				if (dist != 0)
				{
					object->collideWithStaticObject(Vector(0, n), cp);
					object->pos.y += dist;
				}
			}
		}
	}

	void World::collideCircleWithShape(PhysicalObject *circularObject, PhysicalObject *shapedObject, const Polygone &shape)
	{
		// test if circularObject is inside a shape
		for (unsigned i=0; i<shape.size(); i++)
		{
			unsigned next=(i+1)%shape.size();
			Segment s(shape[i].x, shape[i].y, shape[next].x, shape[next].y);

			Vector nn(s.a.y-s.b.y, s.b.x-s.a.x);	//orthog. vector
			Vector u = nn.unitary();

			double d = (circularObject->pos-s.a)*u;
			// if we are inside the circularObject
			if ((d<0) && (-d<circularObject->r))
			{
				Point proj = circularObject->pos - u*d;

				if ((((proj-s.a)*(s.b-s.a))>0) && (((proj-s.b)*(s.a-s.b))>0))
				{
					// if there is a segment which is inside the circularObject, and the projection of the center lies within this segment, this projection is the nearest point. So we return. This is a consequence of having convexe polygones.
					Vector dist = u*-(circularObject->r+d);
					Point collisionPoint = circularObject->pos - u*(d);
					circularObject->collideWithObject(*shapedObject, collisionPoint, dist);
					return;
				}
			}
		}

		double r2 = circularObject->r * circularObject->r;
		double pointInsideD2 = r2;
		Point pointInside;
		Vector centerToPointInside;
		
		// test if there is vertex of shape is inside the circularObject. If so, take the closest to the center
		for (unsigned i=0; i<shape.size(); i++)
		{
			const Point &candidate = shape[i];
			Vector centerToPoint = candidate - circularObject->pos;
			double d2 = centerToPoint.norm2();
			if (d2 < pointInsideD2)
			{
				pointInsideD2 = d2;
				pointInside = candidate;
				centerToPointInside = centerToPoint;
			}
		}

		// we get a collision, one point of shape is inside the circularObject
		if (pointInsideD2 < r2)
		{
			double pointInsideDist = sqrt(pointInsideD2);
			Vector dist = (centerToPointInside / pointInsideDist) * (circularObject->r - pointInsideDist);
			Point collisionPoint = pointInside + dist;
			shapedObject->collideWithObject(*circularObject, collisionPoint, dist);
		}
	}

	void World::collideObjects(PhysicalObject *object1, PhysicalObject *object2)
	{
		// Is there a possible contact ?
		Vector distOCtoOC = object1->pos-object2->pos;
		double addedRay = object1->r+object2->r;
		if (distOCtoOC.norm2() > (addedRay*addedRay))
			return;

		Vector dist, trueDist;
		PhysicalObject *o1, *o2;
		o1 = o2 = NULL;
		Point collisionPoint;
		double maxNorm = 0;

		// for each point of object 1, look if it is in object2
		if (!object1->hull.empty())
		{
			if (!object2->hull.empty())
			{
				// iterate on all shapes of both objects
				for (PhysicalObject::Parts::const_iterator it = object1->hull.begin(); it != object1->hull.end(); ++it)
					for (PhysicalObject::Parts::const_iterator jt = object2->hull.begin(); jt != object2->hull.end(); ++jt)
					{
						const Polygone& shape1 = it->getTransformedShape();
						const Polygone& shape2 = jt->getTransformedShape();
						// TODO: move this into a polygone collision function
						
						// look in both directions
						for (size_t i = 0; i < shape1.size(); i++)
						{
							const Point &candidate = shape1[i];
							if (isPointInside(candidate, object1->pos, shape2, &dist))
							{
								if (dist.norm2() > maxNorm)
								{
									maxNorm = dist.norm2();
									o1 = object1;
									o2 = object2;
									collisionPoint = candidate + dist;
									trueDist = dist;
								}
							}
						}
						
						for (size_t i=0; i < shape2.size(); i++)
						{
							const Point &candidate = shape2[i];
							if (isPointInside(candidate, object2->pos, shape1, &dist))
							{
								if (dist.norm2() > maxNorm)
								{
									maxNorm = dist.norm2();
									o2 = object1;
									o1 = object2;
									collisionPoint = candidate + dist;
									trueDist = dist;
								}
							}
						}
					}
			}
			else
			{
				// collide circle 2 on shape 1
				for (PhysicalObject::Parts::const_iterator it = object1->hull.begin(); it != object1->hull.end(); ++it)
					collideCircleWithShape(object2, object1, it->getTransformedShape());
				return;
			}
		}
		else if (!object2->hull.empty())
		{
			// collide circle 1 on shape 2
			for (PhysicalObject::Parts::const_iterator jt = object2->hull.begin(); jt != object2->hull.end(); ++jt)
				collideCircleWithShape(object1, object2, jt->getTransformedShape());
			return;
		}
		else
		{
			// collide 2 circles
			Vector ud = distOCtoOC.unitary();
			//std::cout << object1 << " to " << object2 << " : " << ud << std::endl;
			double dLength = distOCtoOC.norm();
			dist = ud * (addedRay-dLength);
			//std::cout << object1 << " to " << object2 << " : " << addedRay << " " << dLength << std::endl;
			collisionPoint = object2->pos + ud * object2->r;
			
			//std::cout << "o1 a : " << object1->pos << " " << object1->speed << std::endl;
			//std::cout << "o2 a : " << object2->pos << " " << object2->speed << std::endl;
			object1->collideWithObject(*object2, collisionPoint, dist);
			//std::cout << "o1 b : " << object1->pos << " " << object1->speed << std::endl;
			//std::cout << "o2 b : " << object2->pos << " " << object2->speed << std::endl;
			return;
		}

		// if collision
		if (maxNorm)
		{
			assert(o1);
			assert(o2);
			o1->collideWithObject(*o2, collisionPoint, trueDist);
		}
	}

	void World::step(double dt, unsigned physicsOversampling)
	{
		// oversampling physics
		double overSampledDt = dt / (double)physicsOversampling;
		for (int po = 0; po < physicsOversampling; po++)
		{
			// init interactions
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
				(*i)->initPhysicsInteractions();
			
			// collide objects together
			unsigned iCounter, jCounter;
			iCounter = 0;
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			{
				jCounter = 0;
				for (ObjectsIterator j = objects.begin(); j != objects.end(); ++j)
				{
					if ((*i) != (*j))
					{
						if (iCounter < jCounter)
						{
							if (collideEven)
								collideObjects((*i), (*j));
							else
								collideObjects((*j), (*i));
						}
					}
					jCounter++;
				}
				iCounter++;
			}
	
			collideEven = !collideEven;
	
			// collide objects with walls and physics step
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			{
				if (useWalls)
					collideWithWalls(*i);
				(*i)->finalizePhysicsInteractions(overSampledDt);
				(*i)->physicsStep(overSampledDt);
			}
		}
		
		// init interactions
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			(*i)->initLocalInteractions();
			(*i)->initGlobalInteractions();
		}

		// interract objects together
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			for (ObjectsIterator j = objects.begin(); j != objects.end(); ++j)
			{
				if ((*i) != (*j))
				{
					(*i)->doLocalInteractions(this, (*j), dt);
				}
			}
		}

		// interract objects with walls and control step
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			if (useWalls)
				(*i)->doLocalWallsInteraction(this);
			(*i)->doGlobalInteractions(this, dt);
			
			(*i)->finalizeLocalInteractions(dt);
			(*i)->finalizeGlobalInteractions();
			(*i)->controlStep(overSampledDt);
		}
		if (bluetoothBase)
			bluetoothBase->step(dt, this);
	}
	
	void World::addObject(PhysicalObject *o)
	{
		objects.insert(o);
	}

	void World::removeObject(PhysicalObject *o)
	{
		objects.erase(o);
	}
	
	void World::disconnectExternalObjectsUserData()
	{
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			if ((*i)->userData && (!(*i)->userData->deletedWithObject))
				(*i)->userData = 0;
	}
	
	void World::setRandomSeed(unsigned long seed)
	{
		random.setSeed(seed);
	}
	
	void World::initBluetoothBase()
	{
		bluetoothBase = new BluetoothBase();
	}
	
	BluetoothBase* World::getBluetoothBase()
	{
		if (!bluetoothBase)
			bluetoothBase = new BluetoothBase();
	
		return bluetoothBase;
	}
}

