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
	
	const double PhysicalObject::g = 9.81;
	
	PhysicalObject::PhysicalObject(void)
	{
		userData = NULL;
		
		// default physical parameters
		collisionElasticity = 0.9;
		//staticFrictionThreshold = 0.5;
		dryFrictionCoefficient = 0.25;
		viscousFrictionCoefficient = 0.01;
		viscousMomentFrictionCoefficient = 0.01;
		
		angle = 0;
		
		angSpeed = 0;
		
		mass = 1;
		
		r = 1;
		height = 1;
		
		infraredReflectiveness = 1;
		color = Color::black;
	}
	
	PhysicalObject::~PhysicalObject(void)
	{
		if (userData && (userData->deletedWithObject))
		{
			delete userData;
		}
	}
	
	void PhysicalObject::commitPhysicalParameters()
	{
		computeMomentOfInertia();
	}
	
	void PhysicalObject::computeMomentOfInertia()
	{
		if (boundingSurface.empty())
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
					if (boundingSurface.isPointInside(Point(ix, iy)))
					{
						momentOfInertia += ix * ix + iy * iy;
						area++;
					}
			
			momentOfInertia *= mass / area;
		}
	}
	
	void PhysicalObject::setMass(double mass)
	{
		this->mass = mass;
	}
	
	void PhysicalObject::setCylindric(double radius, double height)
	{
		r = radius;
		boundingSurface.clear();
		this->height = height;
	}
	
	void PhysicalObject::setupBoundingSurface(const Polygone& boundingSurface)
	{
		// get bounding box
		Point bottomLeft, topRight;
		bool validBB = boundingSurface.getAxisAlignedBoundingBox(bottomLeft, topRight);
		assert(validBB);
		
		// numerically compute the center of mass of the shape
		Point cm;
		double area = 0;
		double dx = (topRight-bottomLeft).x / 100;
		double dy = (topRight-bottomLeft).y / 100;
		for (double ix = bottomLeft.x; ix < topRight.x; ix += dx)
			for (double iy = bottomLeft.y; iy < topRight.y; iy += dy)
			{
				if (boundingSurface.isPointInside(Point(ix, iy)))
				{
					cm.x += ix;
					cm.y += iy;
					area++;
				}
			}
		cm /= area;
		area = area / (dx * dy);
		
		// copy bounding surface such that it is centered around center of mass
		size_t faceCount = boundingSurface.size();
		textures.resize(faceCount, Texture(color, 1));
		this->boundingSurface.resize(faceCount);
		r = 0;
		for (size_t i=0; i<faceCount; i++)
		{
			this->boundingSurface[i] = boundingSurface[i] - cm;
			r = std::max(r, this->boundingSurface[i].norm());
		}
	}
	
	void PhysicalObject::setShape(const Polygone& boundingSurface, double height)
	{
		this->height = height;
		
		setupBoundingSurface(boundingSurface);
	}
	
	void PhysicalObject::setColor(const Color &color)
	{
		this->color = color;
		if (!boundingSurface.empty())
			for (size_t i=0; i<boundingSurface.size(); i++)
				textures[i].resize(1, color);
	}
	
	void PhysicalObject::setTextures(const Texture* textures)
	{
		for (size_t i=0; i<boundingSurface.size(); i++)
			this->textures[i] = textures[i];
	}
	
	void PhysicalObject::setInfraredReflectiveness(double infraredReflectiveness)
	{
		this->infraredReflectiveness = infraredReflectiveness;
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

	void PhysicalObject::step(double dt)
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
		if ((abs(dryAngFriction) * dt) > abs(angSpeed))
			angSpeed = 0.;
		else
			angAcc += dryAngFriction;
		
		// viscous friction
		acc += - speed * viscousFrictionCoefficient;
		angAcc += - angSpeed * viscousMomentFrictionCoefficient;
		
		// el cheapos integration
		speed += acc * dt;
		angSpeed += angAcc * dt;
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

	void PhysicalObject::initPhysicsInteractions()
	{
		deinterlaceVector = 0.0;
		computeAbsBoundingSurface();
	}

	void PhysicalObject::doPhysicsInteractions(World *w, PhysicalObject *o, double dt, bool firstInteraction)
	{
		if (firstInteraction)
		{
			if (w->getCollideEven())
				w->collideObjects(this, o);
			else
				w->collideObjects(o, this);
		}
	}

	void PhysicalObject::doPhysicsWallsInteraction(World *w)
	{
		w->collideWithWalls(this);
	}

	void PhysicalObject::finalizePhysicsInteractions(double dt)
	{
		pos += deinterlaceVector;
	}

	void PhysicalObject::computeAbsBoundingSurface(void)
	{
		if (!boundingSurface.empty())
		{
			absBoundingSurface.resize(boundingSurface.size());
			Matrix22 rotMat(angle);
			for (size_t i=0; i<boundingSurface.size(); i++)
			{
				Point realPoint = rotMat*(boundingSurface)[i] + pos;
				absBoundingSurface[i] = realPoint;
			}
		}
	}
	
	

	void PhysicalObject::collideWithStaticObject(const Vector &n, const Point &cp)
	{
		// from http://www.myphysicslab.com/collision.html
		Vector r_ap = (cp - pos);
		Vector v_ap = speed + r_ap.crossFromZVector(angSpeed);
		double num = -(1 + collisionElasticity) * (v_ap * n);
		double denom = (1 / mass) + (r_ap.cross(n) * r_ap.cross(n)) / momentOfInertia;
		double j = num / denom;
		speed += (n * j) / mass;
		angSpeed += r_ap.cross(n * j) / momentOfInertia;
	}

	void PhysicalObject::collideWithStaticObject(const Point &cp1, const Point &cp2, const Vector &n1, const Vector &n2, const Vector &dist)
	{
		if (n1.norm2() > std::numeric_limits<double>::epsilon())
			collideWithStaticObject(n1, cp1);
		if (n2.norm2() > std::numeric_limits<double>::epsilon())
			collideWithStaticObject(n2, cp2);
		pos += dist;
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
		
		// calculate deinterlace vector to put that out of contact - mass ratios ensure physics
		double massSum = mass + that.mass;
		deinterlaceVector += dist*that.mass/massSum;
		that.deinterlaceVector -= dist*mass/massSum;
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
			if (vectCenter.norm2() <  (localInteractions[i]->r+po->_radius())*(localInteractions[i]->r+po->_radius()))
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
		for (size_t i = 0; i < 4; i++)
			wallTextures[i].resize(1, Color::gray);
	}

	World::~World()
	{
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			delete (*i);
		
		if (bluetoothBase)
			delete bluetoothBase;
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
		// let's assume walls are infinite
		Vector dist;
		Point cp1, cp2; // cp1 is on x, cp2 is on y
		Vector n1, n2;

		// object is circle only
		if (object->boundingSurface.empty())
		{
			double x = object->pos.x;
			double y = object->pos.y;
			double r = object->r;
			if (x-r < 0)
			{
				dist.x = r-x;
				cp1.x = 0;
				cp1.y = y;
				n1.x = 1;
			}
			if (y-r < 0)
			{
				dist.y = r-y;
				cp2.y = 0;
				cp2.x = x;
				n2.y = 1;
			}
			if (x+r > w)
			{
				dist.x = w-(x+r);
				cp1.x = w;
				cp1.y = y;
				n1.x = -1;
			}
			if (y+r > h)
			{
				dist.y = h-(y+r);
				cp2.x = x;
				cp2.y = h;
				n2.y = -1;
			}
		}
		else
		{
			const Polygone &bs = object->getTrueBoundingSurface();

			for (unsigned i=0; i<bs.size(); i++)
			{
				double x = bs[i].x;
				double y = bs[i].y;

				if (x<0)
				{
					if (x < -dist.x)
					{
						dist.x = -x;
						cp1.x = 0;
						cp1.y = y;
						n1.x = 1;
					}
				}
				if (y<0)
				{
					if (y < -dist.y)
					{
						dist.y = -y;
						cp2.x = x;
						cp2.y = 0;
						n2.y = 1;
					}
				}
				if (x>w)
				{
					if (x-w > -dist.x)
					{
						dist.x = w-x;
						cp1.x = w;
						cp1.y = y;
						n1.x = -1;
					}
				}
				if (y>h)
				{
					if (y-h > -dist.y)
					{
						dist.y = h-y;
						cp2.y = h;
						cp2.x = x;
						n2.y = -1;
					}
				}
			}
		}

		object->collideWithStaticObject(cp1, cp2, n1, n2, dist);
	}

	void World::collideCircleWithBS(PhysicalObject *circle, PhysicalObject *objectBS, const Polygone &bs)
	{
		// test if circle is inside BS
		for (unsigned i=0; i<bs.size(); i++)
		{
			unsigned next=(i+1)%bs.size();
			Segment s(bs[i].x, bs[i].y, bs[next].x, bs[next].y);

			Vector nn(s.a.y-s.b.y, s.b.x-s.a.x);	//orthog. vector
			Vector u = nn.unitary();

			double d = (circle->pos-s.a)*u;
			// if we are inside the circle
			if ((d<0) && (-d<circle->r))
			{
				Point proj = circle->pos - u*d;

				if ((((proj-s.a)*(s.b-s.a))>0) && (((proj-s.b)*(s.a-s.b))>0))
				{
					// if there is a segment which is inside the circle, and the projection of the center lies within this segment, this projection is the nearest point. So we return. This is a consequence of having convexe polygones.
					Vector dist = u*-(circle->r+d);
					Point collisionPoint = circle->pos - u*(d);
					circle->collideWithObject(*objectBS, collisionPoint, dist);
					return;
				}
			}
		}

		double r2 = circle->r * circle->r;
		double pointInsideD2 = r2;
		Point pointInside;
		Vector centerToPointInside;
		
		// test if there is vertex of BS is inside the circle. If so, take the closest to the center
		for (unsigned i=0; i<bs.size(); i++)
		{
			const Point &candidate = bs[i];
			Vector centerToPoint = candidate - circle->pos;
			double d2 = centerToPoint.norm2();
			if (d2 < pointInsideD2)
			{
				pointInsideD2 = d2;
				pointInside = candidate;
				centerToPointInside = centerToPoint;
			}
		}

		// we get a collision, one point of BS is inside the circle
		if (pointInsideD2 < r2)
		{
			double pointInsideDist = sqrt(pointInsideD2);
			Vector dist = (centerToPointInside / pointInsideDist) * (circle->r - pointInsideDist);
			Point collisionPoint = pointInside + dist;
			objectBS->collideWithObject(*circle, collisionPoint, dist);
		}
	}

	void World::collideObjects(PhysicalObject *object1, PhysicalObject *object2)
	{
		// Is there a possible contact ?
		Vector distOCtoOC = object1->pos-object2->pos;
		double addedRay = object1->r+object2->r;
		if (distOCtoOC.norm2() > (addedRay*addedRay))
			return;

		const Polygone &bs1 = object1->getTrueBoundingSurface();
		const Polygone &bs2 = object2->getTrueBoundingSurface();
		
		Vector dist, trueDist;
		PhysicalObject *o1, *o2;
		o1 = o2 = NULL;
		Point collisionPoint;
		double maxNorm = 0;

		// for each point of object 1, look if it is in object2
		if (!object1->boundingSurface.empty())
		{
			if (!object2->boundingSurface.empty())
			{
				for (unsigned i=0; i<bs1.size(); i++)
				{
					const Point &candidate = bs1[i];
					if (isPointInside(candidate, object1->pos, bs2, &dist))
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
			}
			else
			{
				// collide circle 2 on bs1
				collideCircleWithBS(object2, object1, bs1);
				return;
			}
		}
		else if (!object2->boundingSurface.empty())
		{
			// collide circle 1 on bs2
			collideCircleWithBS(object1, object2, bs2);
			return;
		}
		else
		{
			// collide 2 circles
			Vector ud = distOCtoOC.unitary();
			double dLength = distOCtoOC.norm();
			//collisionPoint = object2->pos+distOCtoOC * (object2->r/addedRay);
			dist = ud * (addedRay-dLength);
			collisionPoint = object2->pos + ud * object2->r;
			object1->collideWithObject(*object2, collisionPoint, dist);
			return;
		}

		// for each point of object 2, look if it is in object1
		if (!object2->boundingSurface.empty())
		{
			for (unsigned i=0; i<bs2.size(); i++)
			{
				const Point &candidate = bs2[i];
				if (isPointInside(candidate, object2->pos, bs1, &dist))
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
						// is it the first interaction between these objects in this world step?
						if (collideEven)
							(*i)->doPhysicsInteractions(this, (*j), overSampledDt, iCounter < jCounter);
						else
							(*j)->doPhysicsInteractions(this, (*i), overSampledDt, jCounter < iCounter);
					}
					jCounter++;
				}
				iCounter++;
			}
	
			collideEven = !collideEven;
	
			// collide objects with walls and step
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			{
				if (useWalls)
					(*i)->doPhysicsWallsInteraction(this);
				(*i)->finalizePhysicsInteractions(overSampledDt);
				(*i)->step(overSampledDt);
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

		// collide objects with walls and step
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			if (useWalls)
				(*i)->doLocalWallsInteraction(this);
			(*i)->doGlobalInteractions(this, dt);
			
			(*i)->finalizeLocalInteractions(dt);
			(*i)->finalizeGlobalInteractions();
		}
		if (bluetoothBase)
			bluetoothBase->step(dt,this);
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

