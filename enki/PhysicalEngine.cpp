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
		computeAreaAndCentroid();
		
		transformedShape.resize(shape.size());
	}
	
	PhysicalObject::Part::Part(const Polygone& shape, double height, const Textures& textures) :
		height(height),
		shape(shape),
		textures(textures)
	{
		computeAreaAndCentroid();
		
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
		height(height),
		area(l1*l2),
		centroid(0, 0)
	{
		const double hl1 = l1 / 2;
		const double hl2 = l2 / 2;
		
		shape << Point(-hl1, -hl2) << Point(hl1, -hl2) << Point(hl1, hl2) << Point(-hl1, hl2);
		transformedShape.resize(shape.size());
	}
	
	void PhysicalObject::Part::computeAreaAndCentroid()
	{
		// from: http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
		const size_t size = shape.size();
		
		// area
		area = 0;
		for (size_t i = 0; i < size; ++i)
		{
			area += (shape[i].x * shape[(i+1) % size].y - shape[(i+1) % size].x * shape[i].y);
		}
		area /= 2;
		
		// centroid
		centroid = Point(0, 0);
		for (size_t i = 0; i < shape.size(); ++i)
		{
			const double multiplicator = (shape[i].x * shape[(i+1) % size].y - shape[(i+1) % size].x * shape[i].y);
			centroid.x += (shape[i].x + shape[(i+1) % size].x) * multiplicator;
			centroid.y += (shape[i].y + shape[(i+1) % size].y) * multiplicator;
		}
		centroid /= (6 * area);
	}
	
	void PhysicalObject::Part::computeTransformedShape(const Matrix22& rot, const Point& trans)
	{
		assert(!shape.empty());
		assert(transformedShape.size() == shape.size());
		for (size_t i = 0; i < shape.size(); ++i)
			transformedShape[i] = rot * (shape)[i] + trans;
		transformedCentroid = rot * centroid + trans;
	}
	
	void PhysicalObject::Part::applyTransformation(const Matrix22& rot, const Point& trans, double* radius = 0)
	{
		for (size_t i = 0; i < shape.size(); ++i)
		{
			(shape)[i] = rot * (shape)[i] + trans;
			if (radius)
				*radius = std::max(*radius, shape[i].norm());
		}
		centroid = rot * centroid + trans;
	}
	
	
	
	// Hull
	
	Polygone PhysicalObject::Hull::getConvexHull() const
	{
		// see http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
		// construct a vector of all points and get the left most
		// the operator < on Vector sorts primary by x coordinate, and if equal, by y coordnate
		typedef std::set<Point> Points;
		Points points;
		for (Hull::const_iterator it = begin(); it != end(); ++it)
		{
			const Polygone& part = it->getShape();
			for (Polygone::const_iterator jt = part.begin(); jt != part.end(); ++jt)
				points.insert(*jt);
		}
		
		// do nothing for empty hulls
		if (points.empty())
			return Polygone();
		
		//  Jarvis march/gift wrapping
		Polygone convexHull;
		convexHull.push_back(*points.begin());
		points.erase(points.begin());
		while (!points.empty())
		{
			Points::iterator candidate = points.begin();
			Vector perp = (*candidate - convexHull.back()).perp();
			
			Points::iterator it = points.begin();
			++it;
			for (; it != points.end(); ++it)
			{
				const Point newVect = *it - convexHull.back();
				if (newVect * perp < 0)
				{
					candidate = it;
					perp = (*candidate - convexHull.back()).perp();
				}
			}
			const Point endVect = convexHull.front() - convexHull.back();
			if (endVect * perp < 0)
				break;
			convexHull.push_back(*candidate);
			points.erase(candidate);
		}
		
		return convexHull;
	}
	
	PhysicalObject::Hull PhysicalObject::Hull::operator+(const Hull& that) const
	{
		Hull newHull(*this);
		for (const_iterator it = that.begin(); it != that.end(); ++it)
			newHull.push_back(*it);
		return newHull;
	}
	
	PhysicalObject::Hull& PhysicalObject::Hull::operator+=(const Hull& that)
	{
		for (const_iterator it = that.begin(); it != that.end(); ++it)
			push_back(*it);
		return *this;
	}
	
	void PhysicalObject::Hull::applyTransformation(const Matrix22& rot, const Point& trans, double* radius)
	{
		if (radius)
			*radius = 0;
		for (iterator it = begin(); it != end(); ++it)
		{
			it->applyTransformation(rot, trans, radius);
		}
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
		interlacedDistance(0),
		id(0)
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
	
	void PhysicalObject::dirtyUserData()
	{
		if (userData)
		{
			userData->deleteIfRequired();
			userData = 0;
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
		
		dirtyUserData();
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
		
		dirtyUserData();
	}
	
	void PhysicalObject::setCustomHull(const Hull& hull, double mass)
	{
		// assign the new hull
		this->hull = hull;
		height = 0;
		for (Hull::const_iterator it = hull.begin(); it != hull.end(); ++it)
			height = std::max(height, it->getHeight());
		
		// compute the center of mass
		setupCenterOfMass();
		
		// set the mass
		this->mass = mass;
		
		// update the moment of inertia
		computeMomentOfInertia();
		
		dirtyUserData();
	}
	
	void PhysicalObject::setColor(const Color &color)
	{
		this->color = color;
		
		dirtyUserData();
		
	}
	
	void PhysicalObject::computeMomentOfInertia()
	{
		if (hull.empty())
		{
			momentOfInertia = 0.5 * mass * r * r;
		}
		else
		{
			// Numerical method:
			// arbitrary shaped object, numerically compute moment of inertia
			momentOfInertia = 0;
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
			
			momentOfInertia *= mass / numericalArea;
			
			// Exact method:
			/*
			TODO: check this and implement
			http://lab.polygonal.de/2006/08/17/calculating-the-moment-of-inertia-of-a-convex-polygon/
			*/
		}
	}
	
	void PhysicalObject::setupCenterOfMass()
	{
		if (hull.empty())
			return;
		
		// Numerical method:
		/*
		// get bounding box of the whole hull
		Point bottomLeft, topRight;
		Hull::iterator it = hull.begin();
		bool validBB = it->shape.getAxisAlignedBoundingBox(bottomLeft, topRight);
		assert(validBB);
		++it;
		for (;it != hull.end(); ++it)
			it->shape.extendAxisAlignedBoundingBox(bottomLeft, topRight);
		
		// numerically compute the center of mass of the shape
		Point cm;
		double area = 0;
		const double dx = (topRight-bottomLeft).x / 100;
		const double dy = (topRight-bottomLeft).y / 100;
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
		*/
		
		// Exact method:
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
		
		// FIXME: this shift is really ugly. We can only do it for non-robots
		// because otherwise the local interactions are missplaced.
		Robot* robot(dynamic_cast<Robot*>(this));
		if (!robot)
		{
			pos += Matrix22(angle) * cm;
			hull.applyTransformation(Matrix22::identity(), -cm, &r);
		}
		else
		{
			// we need to compute radius
			hull.applyTransformation(Matrix22::identity(), Vector(), &r);
		}
	}
	
	void PhysicalObject::computeTransformedShape()
	{
		if (!hull.empty())
		{
			Matrix22 rotMat(angle);
			for (Hull::iterator it = hull.begin(); it != hull.end(); ++it)
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

	#if 0
	void PhysicalObject::physicsStep(double dt)
	{
		// NOTE: not used for now, see later if we should remove or not
		
		/*applyForces(dt);
		
		pos += speed * dt;
		angle += angSpeed * dt;
		*/
		/* TODO: Runge-Kutta
			but this needs a refactoring in order to harvest equations up to now.
			furthermore, we have a so simple model that it is seldom useful for now.
		xn+1 = xn + h⁄6 (a + 2 b + 2 c + d)  where 
		a = f (tn, xn)
		b = f (tn + h⁄2, xn + h⁄2 a)
		c = f (tn + h⁄2, xn + h⁄2 b)
		d = f (tn + h, xn + h c)
		*/
	}
	#endif
	
	void PhysicalObject::controlStep(double dt)
	{
		interlacedDistance = 0.;
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

	void PhysicalObject::initPhysicsInteractions(double dt)
	{
		computeTransformedShape();
		
		applyForces(dt);
		
		pos += speed * dt;
		angle += angSpeed * dt;
		
		// store position after integration
		posBeforeCollision  = pos;
	}

	void PhysicalObject::finalizePhysicsInteractions(double dt)
	{
		// increment interlacedDistance based on pos before and after physics
		interlacedDistance += (posBeforeCollision - pos).norm();
		angle = normalizeAngle(angle);
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
		const Vector r_ap = (cp - pos);
		const Vector v_ap = speed + r_ap.crossFromZVector(angSpeed);
		const double num = -(1 + collisionElasticity) * (v_ap * n);
		const double denom = (1 / mass) + (r_ap.cross(n) * r_ap.cross(n)) / momentOfInertia;
		const double j = num / denom;
		speed += (n * j) / mass;
		angSpeed += r_ap.cross(n * j) / momentOfInertia;
		
		// call the collision callback
		collisionEvent(0);
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
				const Vector n = dist.unitary() * -1;
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
				const Vector n = dist.unitary();
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
		//if ((dist * speed <= 0) && (dist * that.speed >= 0))
		{
			// point of this in inside object
			// we use model from http://www.myphysicslab.com/collision.html
			// this is object A, that is object B
			const Vector n = dist.unitary();
			
			const Vector r_ap = (cp - pos);
			const Vector r_bp = (cp - that.pos);
			
			const Vector v_ap = speed + r_ap.crossFromZVector(angSpeed);
			const Vector v_bp = that.speed + r_bp.crossFromZVector(that.angSpeed);
			const Vector v_ab = v_ap - v_bp;
			
			const double num = -(1 + collisionElasticity * that.collisionElasticity) * (v_ab * n);
			const double denom = (1/mass) + (1/that.mass) + (r_ap.cross(n) * r_ap.cross(n)) / momentOfInertia + (r_bp.cross(n) * r_bp.cross(n)) / that.momentOfInertia;
			const double j = num / denom;
			
			speed += (n * j) / mass;
			that.speed -= (n * j) / that.mass;
			angSpeed += r_ap.cross(n * j) / momentOfInertia;
			that.angSpeed -= r_bp.cross(n * j) / that.momentOfInertia;
		}
		//else
		//	std::cerr << "Non physics collideWithObject between " << this << " and " << &that << std::endl;
		
		// call the collision callback
		collisionEvent(&that);
		that.collisionEvent(this);
		
		// FIXME: this is fully non physic
		// calculate deinterlace vector to put that out of contact - mass ratios ensure physics
		const double massSum = mass + that.mass;
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

	void Robot::initLocalInteractions(double dt, World* w)
	{
		for (size_t i=0; i<localInteractions.size(); i++ )
		{
			localInteractions[i]->init(dt, w);
		}
	}


	void Robot::doLocalInteractions(double dt, World *w, PhysicalObject *po)
	{
		for (size_t i=0; i<localInteractions.size(); i++)
		{
			const Vector vectCenter(this->pos.x - po->pos.x, this->pos.y - po->pos.y );
			if (vectCenter.norm2() <  (localInteractions[i]->r+po->getRadius())*(localInteractions[i]->r+po->getRadius()))
				localInteractions[i]->objectStep(dt, w, po);
			else
				return;
		}
	}


	void Robot::doLocalWallsInteraction(double dt, World* w)
	{
		for (size_t i=0; i<localInteractions.size(); i++)
		{
			if ((this->pos.x>localInteractions[i]->r) && (this->pos.y>localInteractions[i]->r) && (w->w-this->pos.x>localInteractions[i]->r) && (w->h-this->pos.y>localInteractions[i]->r))
				return;
			else
				localInteractions[i]->wallsStep(dt, w);
		}
	}

	void Robot::finalizeLocalInteractions(double dt, World* w)
	{
		for (size_t i=0; i<localInteractions.size(); i++ )
		{
			localInteractions[i]->finalize(dt, w);
		}
	}

	void Robot::doGlobalInteractions(double dt, World* w)
	{
		for (size_t i=0; i<globalInteractions.size(); i++)
		{
			globalInteractions[i]->step(dt, w);
		}
	}
	
	World::GroundTexture::GroundTexture():
		width(0),
		height(0)
	{}
	
	World::GroundTexture::GroundTexture(unsigned width, unsigned height, const uint32_t* data):
		width(width),
		height(height),
		data(data, data+width*height)
	{}

	World::World(double width, double height, const Color& color, const GroundTexture& groundTexture) :
		wallsType(WALLS_SQUARE),
		w(width),
		h(height),
		r(0),
		color(color),
		groundTexture(groundTexture),
		takeObjectOwnership(true),
		bluetoothBase(NULL),
		idNewObject(1)
	{
	}
	
	World::World(double r, const Color& color, const GroundTexture& groundTexture) :
		wallsType(WALLS_CIRCULAR),
		w(0),
		h(0),
		r(r),
		color(color),
		groundTexture(groundTexture),
		takeObjectOwnership(true),
		bluetoothBase(NULL),
		idNewObject(1)
	{
	}
	
	World::World() :
		wallsType(WALLS_NONE),
		w(0),
		h(0),
		r(0),
		color(Color::gray),
		takeObjectOwnership(true),
		bluetoothBase(NULL),
		idNewObject(1)
	{
	}

	World::~World()
	{
		if (takeObjectOwnership)
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
				delete (*i);
		
		if (bluetoothBase)
			delete bluetoothBase;
	}
	
	bool World::hasGroundTexture() const
	{
		return !groundTexture.data.empty();
	}
	
	Color World::getGroundColor(const Point& p) const
	{
		if (groundTexture.data.empty() || wallsType == WALLS_NONE)
			return color;
		int texX, texY;
		if (wallsType == WALLS_SQUARE)
		{
			texX = p.x * groundTexture.width / w;
			texY = p.y * groundTexture.height / h;
		}
		else if (wallsType == WALLS_CIRCULAR)
		{
			texX = (p.x+r) * groundTexture.width / (2*r);
			texY = (p.y+r) * groundTexture.height / (2*r);
		}
		else
			abort();
		
		if (texX < 0 || texX >= groundTexture.width || texY < 0 || texY >= groundTexture.height)
			return color;
		uint32_t data = groundTexture.data[texY * groundTexture.width + texX];
		return Color::fromARGB(data);
	}
	
	/*
	Texture of world walls is disabled now, re-enable a proper support if required
	void World::setWallsColor(const Color& color)
	{
		for (size_t i = 0; i < 4; i++)
			wallTextures[i].resize(1, color);
	}
	*/
	
	bool World::isPointInside(const Point &p, const Point &c, const Polygone &bs, Vector *distVector)
	// p = candidate point of object; c = pos of object; bs = bounding surface of other object; distVector = deinterlacing dist to be calculated
	{
		// Segment 1 from points c to d
		const Point d = p;
		bool intersection_found = false;
		 
		for (size_t i=0; i<bs.size(); i++)
		{
			const size_t next = (i+1)%bs.size();
			// Segment 2 from points a to b
			const Point a(bs[i].x, bs[i].y);
			const Point b(bs[next].x, bs[next].y);
			
			// test if segments 1 and 2 overlap 
			// see: Real-time collision detection, C. Ericson, Page 152-153
			
			const double a1 = getTriangleAreaTwice(a,b,d);
			const double a2 = getTriangleAreaTwice(a,b,c);
			
			if (a1 * a2 < 0.0f)
			{
				const double a3 = getTriangleAreaTwice(c,d,a);
				const double a4 = a3 + a2 - a1;
				if (a3 * a4 < 0.0f)
				{
					// Segments 1 and 2 intersect
					intersection_found = true;
					
					const double dist = getTriangleHeight(a,b,d); // TODO: abs necessary?
					
					if (dist < 0)
					{
						// both c and p are outside bs
						// the intersection can be handled when checking the points of bs					
						return false; 
					}
					
					Vector n = (b-a).perp().unitary();			
					*distVector = n * (-dist); // TODO: ok that we modify this even if we might return false??						
					/*
						std::cout << "Hull: " << bs << std::endl;
						std::cout << "i: " << i << std::endl;
						std::cout << "next: " << next << std::endl;						
						std::cout << "p: " << p  << std::endl;
						std::cout << "c: " << c  << std::endl;	
						assert(false);					
					*/
				}
			}
		}
		return intersection_found;
	}

	void World::collideWithSquareWalls(PhysicalObject *object)
	{
		// object is circle only
		if (object->hull.empty())
		{
			const double x = object->pos.x;
			const double y = object->pos.y;
			const double r = object->r;
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
			for (PhysicalObject::Hull::const_iterator it = object->hull.begin(); it != object->hull.end(); ++it)
			{
				const Polygone& shape = it->getTransformedShape();
				
				// let's assume walls are infinite
				Point cp1, cp2; // cp1 is on x, cp2 is on y
				Vector cp;
				
				double dist = 0;
				double n = 0;
				for (size_t i=0; i<shape.size(); i++)
				{
					const double x = shape[i].x;
					const double y = shape[i].y;
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
					const double x = shape[i].x;
					const double y = shape[i].y;
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
	
	void World::collideWithCircularWalls(PhysicalObject *object)
	{
		const double r2 = r * r;
		// object is circle only
		if (object->hull.empty())
		{
			const double distToWall = r - (object->pos.norm() + object->r);
			if (distToWall < 0)
			{
				const Vector dirU = object->pos.unitary();
				object->collideWithStaticObject(-dirU, dirU * r);
				object->pos += dirU * distToWall;
			}
		}
		else
		{
			// iterate over all shapes
			for (PhysicalObject::Hull::const_iterator it = object->hull.begin(); it != object->hull.end(); ++it)
			{
				const Polygone& shape = it->getTransformedShape();
				Point cp;
				double dist = 0;
				for (size_t i=0; i<shape.size(); i++)
				{
					if (shape[i].norm2() > r2)
					{
						double newDist = shape[i].norm() - r;
						if (newDist > dist)
						{
							dist = newDist;
							cp = shape[i];
						}
					}
				}
				if (dist > 0)
				{
					const Vector dirU = cp.unitary();
					object->collideWithStaticObject(-dirU, dirU * r);
					object->pos -= dirU * dist;
				}
			}
			// TODO: verify this code
		}
	}
	
	void World::collideCircleWithShape(PhysicalObject *circularObject, PhysicalObject *shapedObject, const Polygone &shape)
	{
		// test if circularObject is inside a shape
		for (unsigned i=0; i<shape.size(); i++)
		{
			const size_t next=(i+1)%shape.size();
			const Segment s(shape[i].x, shape[i].y, shape[next].x, shape[next].y);

			const Vector nn(s.a.y-s.b.y, s.b.x-s.a.x);	//orthog. vector
			const Vector u = nn.unitary();

			const double d = (circularObject->pos-s.a)*u;
			// if we are inside the circularObject
			if ((d<0) && (-d<circularObject->r))
			{
				const Point proj = circularObject->pos - u*d;

				if ((((proj-s.a)*(s.b-s.a))>0) && (((proj-s.b)*(s.a-s.b))>0))
				{
					// if there is a segment which is inside the circularObject, and the projection of the center lies within this segment, this projection is the nearest point. So we return. This is a consequence of having convexe polygones.
					const Vector dist = u*-(circularObject->r+d);
					const Point collisionPoint = circularObject->pos - u*(d);
					circularObject->collideWithObject(*shapedObject, collisionPoint, dist);
					return;
				}
			}
		}

		const double r2 = circularObject->r * circularObject->r;
		double pointInsideD2 = r2;
		Point pointInside;
		Vector centerToPointInside;
		
		// test if there is vertex of shape is inside the circularObject. If so, take the closest to the center
		for (unsigned i=0; i<shape.size(); i++)
		{
			const Point &candidate = shape[i];
			const Vector centerToPoint = candidate - circularObject->pos;
			const double d2 = centerToPoint.norm2();
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
			const double pointInsideDist = sqrt(pointInsideD2);
			const Vector dist = (centerToPointInside / pointInsideDist) * (circularObject->r - pointInsideDist);
			const Point collisionPoint = pointInside + dist;
			shapedObject->collideWithObject(*circularObject, collisionPoint, dist);
		}
	}

	void World::collideObjects(PhysicalObject *object1, PhysicalObject *object2)
	{
		// Is there a possible contact ?
		const Vector distOCtoOC = object1->pos-object2->pos;
		const double addedRay = object1->r+object2->r;
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
				for (PhysicalObject::Hull::const_iterator it = object1->hull.begin(); it != object1->hull.end(); ++it)
				{
					const Point& shape1Centroid = it->getTransformedCentroid();
					const Polygone& shape1 = it->getTransformedShape();
					for (PhysicalObject::Hull::const_iterator jt = object2->hull.begin(); jt != object2->hull.end(); ++jt)
					{
						const Point& shape2Centroid = jt->getTransformedCentroid();
						const Polygone& shape2 = jt->getTransformedShape();
						// TODO: move this into a polygone collision function
						
						// look in both directions
						for (size_t i = 0; i < shape1.size(); i++)
						{
							const Point &candidate = shape1[i];
							if (isPointInside(candidate, shape1Centroid, shape2, &dist))
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
							if (isPointInside(candidate, shape2Centroid, shape1, &dist))
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
			}
			else
			{
				// collide circle 2 on shape 1
				for (PhysicalObject::Hull::const_iterator it = object1->hull.begin(); it != object1->hull.end(); ++it)
					collideCircleWithShape(object2, object1, it->getTransformedShape());
				return;
			}
		}
		else if (!object2->hull.empty())
		{
			// collide circle 1 on shape 2
			for (PhysicalObject::Hull::const_iterator jt = object2->hull.begin(); jt != object2->hull.end(); ++jt)
				collideCircleWithShape(object1, object2, jt->getTransformedShape());
			return;
		}
		else
		{
			// collide 2 circles
			const Vector ud = distOCtoOC.unitary();
			const double dLength = distOCtoOC.norm();
			dist = ud * (addedRay-dLength);
			collisionPoint = object2->pos + ud * object2->r;
			
			object1->collideWithObject(*object2, collisionPoint, dist);
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
		const double overSampledDt = dt / (double)physicsOversampling;
		for (unsigned po = 0; po < physicsOversampling; po++)
		{
			// init physics interactions
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
				(*i)->initPhysicsInteractions(overSampledDt);
			
			// collide objects together
			unsigned iCounter, jCounter;
			iCounter = 0;
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			{
				jCounter = 0;
				for (ObjectsIterator j = objects.begin(); j != objects.end(); ++j)
				{
					if (iCounter < jCounter)
					{
						collideObjects((*i), (*j));
					}
					jCounter++;
				}
				iCounter++;
			}
			
			// collide objects with walls and physics step
			for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			{
				switch (wallsType)
				{
					case WALLS_SQUARE: collideWithSquareWalls(*i); break;
					case WALLS_CIRCULAR: collideWithCircularWalls(*i); break;
					default: break;
				}
				(*i)->finalizePhysicsInteractions(overSampledDt);
			}
		}
		
		// init non-physics interactions
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			(*i)->initLocalInteractions(dt, this);
			(*i)->initGlobalInteractions(dt, this);
		}

		// interact objects together
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			for (ObjectsIterator j = objects.begin(); j != objects.end(); ++j)
			{
				if ((*i) != (*j))
				{
					(*i)->doLocalInteractions(dt, this, (*j));
				}
			}
		}

		// interact objects with walls and control step
		for (ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
		{
			PhysicalObject* o = *i;
			if (wallsType != WALLS_NONE)
				o->doLocalWallsInteraction(dt, this);
			o->doGlobalInteractions(dt, this);
			o->finalizeLocalInteractions(dt, this);
			o->finalizeGlobalInteractions(dt, this);
			o->controlStep(dt);
		}
		
		// do a control step for the world
		controlStep(dt);
		// TODO: cleanup this
		if (bluetoothBase)
			bluetoothBase->step(dt, this);
	}
	
	void World::addObject(PhysicalObject *o)
	{
		if (o->getId() == 0)
		{
			o->id = idNewObject;
			idNewObject++;
		}
		// This branch of the conditionnal is required when deserializing
		// objects since we want to use the id of the corresponding objects on
		// the server.
		else if (idNewObject < o->getId())
		{
			idNewObject = o->getId() + 1;
		}
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

