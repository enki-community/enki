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

#include "Geometry.h"
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <exception>
#include <cmath>

namespace Enki
{
	
	template<class T>
	bool almost_equal(T x, T y, int ulp = 2)
	{
		// the machine epsilon has to be scaled to the magnitude of the values used
		// and multiplied by the desired precision in ULPs (units in the last place)
		return std::abs(x-y) < std::numeric_limits<T>::epsilon() * std::abs(x+y) * ulp
		// unless the result is subnormal
			|| std::abs(x-y) < std::numeric_limits<T>::min();
	}

	std::ostream & operator << (std::ostream & outs, const Vector &vector)
	{
		outs << "(" << vector.x << ", " << vector.y << ")";
		return outs;
	}
	
	std::ostream & operator << (std::ostream & outs, const Segment &segment)
	{
		outs << segment.a << "-" << segment.b;
		return outs;
	}
	
	std::ostream & operator << (std::ostream & outs, const Polygon &polygon)
	{
		for (Polygon::const_iterator it = polygon.begin(); it != polygon.end(); ++it)
			outs << *it << " ";
		return outs;
	}
	
	double Segment::dist(const Point &p) const
	{
		const Vector n(a.y-b.y, b.x-a.x);
		const Vector u = n.unitary();
		const Vector ap = p-a;
		return ap * u;
	}
	
	bool Segment::doesIntersect(const Segment &that, Point* intersectionPoint) const
	{
		const Vector r(this->b - this->a);
		const Vector s(that.b - that.a);
		const Vector thatAMinThisA(that.a - this->a);
		const double rCrossS(r.cross(s));
		if (almost_equal(rCrossS, 0.))
		{
			if (almost_equal(thatAMinThisA.cross(r), 0.))
			{
				// colinear, check if overlap
				if (this->isDegenerate())
				{
					if (that.isDegenerate())
					{
						// both degenerate, are they equal?
						if (this->a == that.a)
						{
							// yes? intersection
							if (intersectionPoint)
								*intersectionPoint = a;
							return true;
						}
						return false;
					}
					else
					{
						// only this is degenerate, is it on that?
						if ((a.x >= fmin(that.a.x, that.b.x)) &&
							(a.y >= fmin(that.a.y, that.b.y)) &&
							(a.x <= fmax(that.a.x, that.b.x)) &&
							(a.y <= fmax(that.a.y, that.b.y)))
						{
							// yes, intersection
							if (intersectionPoint)
									*intersectionPoint = a;
							return true;
						}
						return false;
					}
				}
				// both segments have non-zero length
				const Vector rOnNorm2(r / r.norm2());
				const double t0(thatAMinThisA * rOnNorm2);
				const double t1(t0 + s * rOnNorm2);
				if (fmin(t0, t1) > 1)
					return false;
				if (fmax(t0, t1) < 0)
					return false;
				if (intersectionPoint)
				{
					// intersection is in the middle of the overlapping interval
					const double t0Clamped(fmax(fmin(t0, 1.), 0.));
					const double t1Clamped(fmax(fmin(t1, 1.), 0.));
					const double tMean((t0Clamped + t1Clamped) / 2);
					*intersectionPoint = a + r * tMean;
				}
				return true;
			}
			return false;
		}
		else
		{
			const double t(thatAMinThisA.cross(s) / rCrossS);
			const double u(thatAMinThisA.cross(r) / rCrossS);
			if (0 <= t && t <= 1 && 0 <= u && u <= 1)
			{
				if (intersectionPoint)
					*intersectionPoint = a + r * t;
				return true;
			}
			return false;
		}
	}
	
	Segment Polygon::getSegment(size_t i) const
	{
		if (size() < 2)
			throw std::runtime_error("trying to get segment of a polygon with less than two points");
		return Segment((*this)[i % size()], (*this)[(i + 1) % size()]);
	}
	
	bool Polygon::isPointInside(const Point& p) const
	{
		for (size_t i = 0; i < size(); i++)
		{
			if (getSegment(i).dist(p) < 0)
				return false;
		}
		return true;
	}
	
	bool Polygon::getAxisAlignedBoundingBox(Point& bottomLeft, Point& topRight) const
	{
		if (empty())
			return false;
		
		bottomLeft = (*this)[0];
		topRight = (*this)[0];
		
		extendAxisAlignedBoundingBox(bottomLeft, topRight);
		
		return true;
	}
	
	void Polygon::extendAxisAlignedBoundingBox(Point& bottomLeft, Point& topRight) const
	{
		for (const_iterator it = begin(); it != end(); ++it)
		{
			const Point& p = *it;
			
			if (p.x < bottomLeft.x)
				bottomLeft.x = p.x;
			else if (p.x > topRight.x)
				topRight.x = p.x;
			
			if (p.y < bottomLeft.y)
				bottomLeft.y = p.y;
			else if (p.y > topRight.y)
				topRight.y = p.y;
		}
	}
	
	double Polygon::getBoundingRadius() const
	{
		double radius = 0;
		for (size_t i = 0; i < size(); i++)
			radius = std::max<double>(radius, (*this)[i].norm());
		return radius;
	}
	
	void Polygon::translate(const Vector& delta)
	{
		for (iterator it = begin(); it != end(); ++it)
			*it += delta;
	}
	
	void Polygon::rotate(const double angle)
	{
		Matrix22 rot(angle);
		for (iterator it = begin(); it != end(); ++it)
			*it = rot * (*it);
	}
	
	void Polygon::flipX()
	{
		for (size_t i = 0; i < size(); i++)
			(*this)[i].x = -(*this)[i].x;
		for (size_t i = 0; i < size() / 2; i++)
		{
			Point p = (*this)[i];
			(*this)[i] = (*this)[size() - i - 1];
			(*this)[size() - i - 1] = p;
		}
	}
	
	void Polygon::flipY()
	{
		for (size_t i = 0; i < size(); i++)
			(*this)[i].y = -(*this)[i].y;
		for (size_t i = 0; i < size() / 2; i++)
		{
			Point p = (*this)[i];
			(*this)[i] = (*this)[size() - i - 1];
			(*this)[size() - i - 1] = p;
		}
	}
	
	bool Polygon::doesIntersect(const Polygon& that, Vector& mtv, Point& intersectionPoint) const
	{
		// Note: does not handle optimally the case of full overlapping
		
		// Using the Separate Axis Theorem, see for instance: http://www.dyn4j.org/2010/01/sat/
		double minMTVDist(std::numeric_limits<double>::max());
		Vector minMTV;
		Vector minCollisionPoint;
		
		// do points of that are inside this
		for (size_t i = 0; i < this->size(); ++i)
		{
			const Segment segment(this->getSegment(i));
			double maxDist(0);
			size_t maxJ(0);
			for (size_t j = 0; j < that.size(); ++j)
			{
				// positive distance for inside
				const double dist(segment.dist(that[j]));
				if (dist > maxDist)
				{
					maxDist = dist;
					maxJ = j;
				}
			}
			// if all points of that are outside, we found a separate axis
			if (maxDist == 0)
				return false;
			// if this side has a lower penetration than best so far, take as best
			if (maxDist < minMTVDist)
			{
				minMTVDist = maxDist;
				minMTV = segment.getDirection().perp().unitary() * maxDist;
				minCollisionPoint = that[maxJ];
			}
		}
		
		// do points of this are inside that
		for (size_t i = 0; i < that.size(); ++i)
		{
			const Segment segment(that.getSegment(i));
			double maxDist(0);
			size_t maxJ(0);
			for (size_t j = 0; j < this->size(); ++j)
			{
				// positive distance for inside
				const double dist(segment.dist((*this)[j]));
				if (dist > maxDist)
				{
					maxDist = dist;
					maxJ = j;
				}
			}
			// if all points of this are outside, we found a separate axis
			if (maxDist == 0)
				return false;
			// if this side has a lower penetration than best so far, take as best
			if (maxDist < minMTVDist)
			{
				minMTVDist = maxDist;
				minMTV = -segment.getDirection().perp().unitary() * maxDist;
				minCollisionPoint = (*this)[maxJ] + minMTV;
			}
		}
		
		// there was no separate axis found, update collision variables...
		mtv = minMTV;
		intersectionPoint = minCollisionPoint;
		
		// ... and return true
		return true;
	}
	
	bool Polygon::doesIntersect(const Point& center, const double r, Vector& mtv, Point& intersectionPoint) const
	{
		// Note: does not handle optimally the case of full overlapping
		
		// Using the Separate Axis Theorem, see for instance: http://www.dyn4j.org/2010/01/sat/
		double minMTVDist(std::numeric_limits<double>::max());
		Vector minMTV;
		Vector minCollisionPoint;
		
		// test if circle is inside a shape
		for (size_t i = 0; i < size(); ++i)
		{
			const Segment segment(getSegment(i));
			const Vector normal(segment.getDirection().perp());
			const Vector u(normal.unitary());
			// positive distance for inside
			double dist((center-segment.a)*u + r);
			// if circle is outside, we found a separate axis
			if (dist <= 0)
				return false;
			// no, we need to check whether the projection of center is on the segment
			const Point proj(center + u*(r-dist));
			const double prodA((proj - segment.a) * segment.getDirection());
			const double prodB((proj - segment.b) * segment.getDirection());
			// yes?
			if (prodA >= 0 && prodB <= 0)
			{
				// is this distance smaller than previous ones?
				if (dist < minMTVDist)
				{
					minMTVDist = dist;
					minMTV = u * dist;
					minCollisionPoint = proj + minMTV;
				}
			}
		}
		
		// if found a solution so far, update collision variables and return it
		if (minMTVDist != std::numeric_limits<double>::max())
		{
			mtv = minMTV;
			intersectionPoint = minCollisionPoint;
			return true;
		}
		
		// at this point if there is a collision, we know that there is a vertex inside the circle
		double minPointCenterDist2(std::numeric_limits<double>::max());
		
		// test if there is vertex of shape is inside the circle. If so, take the closest to the center.
		for (size_t i = 0; i < size(); ++i)
		{
			const Vector centerToPoint((*this)[i] - center);
			const double d2(centerToPoint.norm2());
			if (d2 < minPointCenterDist2 && d2 <= r*r)
			{
				minPointCenterDist2 = d2;
				minMTV = centerToPoint.unitary() * (r - sqrt(d2));
				minCollisionPoint = center + centerToPoint + minMTV;
			}
		}
		
		// no vertex inside the circle, no collision
		if (minPointCenterDist2 == std::numeric_limits<double>::max())
			return false;
		
		// collision, update collision variables...
		mtv = minMTV;
		intersectionPoint = minCollisionPoint;
		
		// ... and return true
		return true;
	}
}
