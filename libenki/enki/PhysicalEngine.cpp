/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2006 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006 Laboratory of Robotics Systems, EPFL, Lausanne
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

// _________________________________
//
// Implementation of PhysicalObject
// _________________________________
// All objects in the world are derived from a PhysicalObject

namespace Enki
{
	FastRandom random;
	
	PhysicalObject::PhysicalObject(void) 
	{
		boundingSurface = NULL;
		angle = 0;
		angSpeed = 0;
		mass = 1;
		collisionWithWalls=false;
		r = 1;
		reflection = 1;
		height = 1;
		staticFrictionThreshold = 0;
		viscousFrictionTau = 0;
		viscousMomentFrictionTau = 0;
		collisionAngularFrictionFactor = 0;
		color = Color::black;
	}

	PhysicalObject::~PhysicalObject(void)
	{

	}

	void PhysicalObject::step(double dt)
	{
		pos += speed * dt;
		angle += angSpeed * dt;
		angle = normalizeAngle(angle);
		// TODO : optimise this using ExpDecay from external math lib !
		
		
		if (viscousFrictionTau < dt)
		{
			speed = 0.0;
		}
		else
		{
		
			double factor = (viscousFrictionTau - dt * 0.5) / (viscousFrictionTau + dt * 0.5); 
			speed *= factor;
			
		}
		if (viscousMomentFrictionTau < dt)
		{
			angSpeed = 0;
		}
		else
		{
		    //std::cerr << "0  f:" << angSpeed << std::endl;
		    double factor = (viscousMomentFrictionTau - dt * 0.5) / (viscousMomentFrictionTau + dt * 0.5); 
			angSpeed *= factor;
			//std::cerr << "angSpeed:" << angSpeed << std::endl;
			//std::cerr << "factor:" << factor << std::endl;
		}
	}

	void PhysicalObject::initLocalInteractions()
	{
		deinterlaceVector = 0.0;
		computeAbsBoundingSurface();
	}

	void PhysicalObject::doLocalInteractions(World *w, PhysicalObject *o, double dt, bool firstInteraction)
	{
		if (firstInteraction)
		{
			if (w->getCollideEven())
				w->collideObjects(this, o);
			else
				w->collideObjects(o, this);
		}
	}

	void PhysicalObject::doLocalWallsInteraction(World *w)
	{
		w->collideWithWalls(this);
	}

	void PhysicalObject::finalizeLocalInteractions(double dt)
	{
		// if smaller than threshold, don't move
		double displacementThreshold = staticFrictionThreshold * dt;
		if (deinterlaceVector.norm2() > displacementThreshold*displacementThreshold)
			pos += deinterlaceVector;
	}

	void PhysicalObject::computeAbsBoundingSurface(void)
	{
		if (boundingSurface)
		{
			absBoundingSurface.resize(boundingSurface->size());
			Matrix22 rotMat(angle);
			for (size_t i=0; i<boundingSurface->size(); i++)
			{
				Point realPoint = rotMat*(*boundingSurface)[i] + pos;
				absBoundingSurface[i] = realPoint;
			}
		}
	}

	void PhysicalObject::setBoundingSurface(const Polygone *bs)
	{
		assert(bs);
		boundingSurface = bs;
		r = 0;
		size_t faceCount = boundingSurface->size();
		textures.resize(faceCount);
		for (unsigned i=0; i<faceCount; i++)
		{
			const Point &p = (*boundingSurface)[i];
			r = std::max(r, p.norm());
			textures[i].resize(1, color);
		}
	}

	void PhysicalObject::collideWithStaticObject(const Vector &n)
	{
		// angular friction
		angSpeed -= (speed.x*n.y - speed.y*n.x) * collisionAngularFrictionFactor;
	}

	void PhysicalObject::collideWithStaticObject(const Point &cp1, const Point &cp2, const Vector &n1, const Vector &n2, const Vector &dist)
	{
		collideWithStaticObject(n1);
		collideWithStaticObject(n2);
		pos += dist;
	}

	void PhysicalObject::collideWithObject(PhysicalObject &object, const Point &cp, const Vector &dist)
	{
		// handle infinite mass case
		if (mass < 0)
		{
			if (object.mass < 0)
			{
				//assert(false);
				return;
			}
			else
			{
				// if colliding with wall
				Vector n = dist.unitary() * -1;
				object.collideWithStaticObject(n);
				object.pos -= dist;
				return;
			}
		}
		else
		{
			// if colliding with wall
			if (object.mass < 0)
			{
				Vector n = dist.unitary();
				collideWithStaticObject(n);
				pos += dist;
				return;
			}
		}

		// total mass of 2 objects
		double masssum = mass + object.mass;
		// vectors to collision
		Vector dPos1 = cp-pos;
		Vector dPos2 = cp-object.pos;

		// handle rotation
		// those values should be multiplied by the lookuped value of rotation friction for the given two 
		// surfaces (collAngFrictFact is just an approximation)
		angSpeed = (dPos1.x*dPos2.y-dPos1.y*dPos2.x)*(object.mass/masssum)*collisionAngularFrictionFactor;
		object.angSpeed = (dPos2.x*dPos1.y-dPos2.y*dPos1.x)*(mass/masssum)*collisionAngularFrictionFactor;

		// calculate deinterlace vector to put object out of contact - mass ratios ensure physics
		deinterlaceVector += dist*object.mass/masssum;
		object.deinterlaceVector -= dist*mass/masssum;
	}

	Robot::Robot(void)
	{

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
		for (unsigned i=0; i<localInteractions.size(); i++ )
		{
			localInteractions[i]->init();
		}
		PhysicalObject::initLocalInteractions();
	}


	void Robot::doLocalInteractions(World *w, PhysicalObject *po, double dt, bool firstInteraction)
	{
		for (unsigned i=0; i<localInteractions.size(); i++)
		{
			Vector vectCenter(this->pos.x - po->pos.x, this->pos.y - po->pos.y );
			if (vectCenter.norm2() <  (localInteractions[i]->r+po->r)*(localInteractions[i]->r+po->r))
				localInteractions[i]->objectStep(dt, po, w);
			else 
				return;
		}
		// is done only if first interaction is true !!
		PhysicalObject::doLocalInteractions(w, po, dt, firstInteraction);
	}


	void Robot::doLocalWallsInteraction(World *w)
	{
		for (unsigned i=0; i<localInteractions.size(); i++)
		{
			if ((this->pos.x>localInteractions[i]->r) && (this->pos.y>localInteractions[i]->r) && (w->w-this->pos.x>localInteractions[i]->r) && (w->h-this->pos.y>localInteractions[i]->r))
				return;
			else
				localInteractions[i]->wallsStep(w);
		}
		PhysicalObject::doLocalWallsInteraction(w);
	}

	void Robot::finalizeLocalInteractions(double dt)
	{
		for (unsigned i=0; i<localInteractions.size(); i++ )
		{
			localInteractions[i]->finalize(dt);
		}
		PhysicalObject::finalizeLocalInteractions(dt);
	}

	void Robot::doGlobalInteractions(World *w, double dt)
	{
		for (unsigned i=0; i<globalInteractions.size(); i++)
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
		for (unsigned i=0; i<bs.size(); i++)
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
		// check if collision
		if ((object->pos.x>object->r) && (object->pos.y>object->r) && (w-object->pos.x>object->r) && (h-object->pos.y>object->r)) {
			object->collisionWithWalls=false;
			return;
		}
		object->collisionWithWalls=true;


		// let's assume walls are infinite
		Vector dist;
		Point cp1, cp2; // cp1 is on x, cp2 is on y
		Vector n1, n2;

		// object is circle only
		if (!object->boundingSurface)
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
		if (object1->boundingSurface)
		{
			if (object2->boundingSurface)
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
		else if (object2->boundingSurface)
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
			collisionPoint = object2->pos+distOCtoOC * (object2->r/addedRay);
			dist = ud * (addedRay-dLength);
			object1->collideWithObject(*object2, collisionPoint, dist);
			return;
		}

		// for each point of object 2, look if it is in object1
		if (object2->boundingSurface)
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

	void World::step(double dt)
	{
		// init interactions
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			(*i)->initLocalInteractions();
			(*i)->initGlobalInteractions();
		}

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
						(*i)->doLocalInteractions(this, (*j), dt, iCounter < jCounter);
					else
						(*j)->doLocalInteractions(this, (*i), dt, jCounter < iCounter);
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
				(*i)->doLocalWallsInteraction(this);
			(*i)->doGlobalInteractions(this, dt);
			
			(*i)->finalizeLocalInteractions(dt);
			(*i)->finalizeGlobalInteractions();
			(*i)->step(dt);
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
	/*
	bool World::isObject(PhysicalObject *o)
	{
		ObjectsIterator oIt = objects.find(o);
		return oIt != objects.end();
	}
	*/
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

