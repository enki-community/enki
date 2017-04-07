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

namespace Enki
{
	std::ostream & operator << (std::ostream & outs, const Vector &vector)
	{
		outs << "(" << vector.x << ", " << vector.y << ")";
		return outs;
	}
	
	std::ostream & operator << (std::ostream & outs, const Polygone &polygone)
	{
		for (Polygone::const_iterator it = polygone.begin(); it != polygone.end(); ++it)
			outs << *it << " ";
		return outs;
	}
	
	bool doIntersect(const Polygone& shape1, const Polygone& shape2, Vector& mtv, Point& collisionPoint, bool& mtvApplyToShape1)
	{
		// Note: does not handle optimally the case of full overlapping
		
		// Using the Separate Axis Theorem, see for instance: http://www.dyn4j.org/2010/01/sat/
		double minMTVDist(std::numeric_limits<double>::max());
		Vector minMTV;
		Vector minInsidePoint;
		bool minMtvApplyToShape1;
		
		// do points of shape2 have overlap on shape1
		for (size_t i = 0; i < shape1.size(); ++i)
		{
			const Segment segment(shape1.getSegment(i));
			double maxDist(0);
			size_t maxJ(0);
			for (size_t j = 0; j < shape2.size(); ++j)
			{
				// positive distance for inside
				const double dist(segment.dist(shape2[j]));
				if (dist > maxDist)
				{
					maxDist = dist;
					maxJ = j;
				}
			}
			// if all points of shape2 are outside, we found a separate axis
			if (maxDist == 0)
				return false;
			// if this side has a lower penetration than best so far, take as best
			if (maxDist < minMTVDist)
			{
				minMTVDist = maxDist;
				minMTV = -segment.getDirection().perp().unitary() * maxDist;
				minInsidePoint = shape2[maxJ];
				minMtvApplyToShape1 = false;
			}
		}
		
		// do points of shape1 have overlap on shape2
		for (size_t i = 0; i < shape2.size(); ++i)
		{
			const Segment segment(shape2.getSegment(i));
			double maxDist(0);
			size_t maxJ(0);
			for (size_t j = 0; j < shape1.size(); ++j)
			{
				// positive distance for inside
				const double dist(segment.dist(shape1[j]));
				if (dist > maxDist)
				{
					maxDist = dist;
					maxJ = j;
				}
			}
			// if all points of shape1 are outside, we found a separate axis
			if (maxDist == 0)
				return false;
			// if this side has a lower penetration than best so far, take as best
			if (maxDist < minMTVDist)
			{
				minMTVDist = maxDist;
				minMTV = -segment.getDirection().perp().unitary() * maxDist;
				minInsidePoint = shape1[maxJ];
				minMtvApplyToShape1 = true;
			}
		}
		
		// there was no separate axis found, update collision variables...
		mtv = minMTV;
		collisionPoint = minInsidePoint + minMTV;
		mtvApplyToShape1 = minMtvApplyToShape1;
		
		// ... and return true
		return true;
	}
	
	bool Polygone::doIntersect(const Point& center, const double r, Vector& mtv, Point& collisionPoint) const
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
		
		// if found solution so far, return it
		if (minMTVDist != std::numeric_limits<double>::max())
		{
			mtv = minMTV;
			collisionPoint = minCollisionPoint;
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
		
		// collision, return values
		mtv = minMTV;
		collisionPoint = minCollisionPoint;
		
		return true;
	}
	
	Point getIntersection(const Segment &s1, const Segment &s2)
	{
		// compute first segment's equation
		const double c1 = s1.a.y + (-s1.a.x / (s1.b.x - s1.a.x)) * (s1.b.y - s1.a.y);
		const double m1 = (s1.b.y - s1.a.y) / (s1.b.x - s1.a.x);
 
		// compute second segment's equation
		const double c2 = s2.a.y + (-s2.a.x / (s2.b.x - s2.a.x)) * (s2.b.y - s2.a.y);
		const double m2 = (s2.b.y - s2.a.y) / (s2.b.x - s2.a.x);

		// are the lines parallel ?
		if (m1 == m2)
			return Point(HUGE_VAL, HUGE_VAL);

		double x1 = s1.a.x;
		double x2 = s1.b.x;
		double x3 = s2.a.x;
		double x4 = s2.b.x;
		double y1 = s1.a.y;
		double y2 = s1.b.y;
		double y3 = s2.a.y;
		double y4 = s2.b.y;

		// make sure x1 < x2
		if (x1 > x2)
		{
			double temp = x1;
			x1 = x2;
			x2 = temp;
		}

		// make sure x3 < x4
		if (x3 > x4)
		{
			double temp = x3;
			x3 = x4;
			x4 = temp;
		}

		// make sure y1 < y2
		if (y1 > y2)
		{
			double temp = y1;
			y1 = y2;
			y2 = temp;
		}

		// make sure y3 < y4
		if (y3 > y4)
		{
			double temp = y3;
			y3 = y4;
			y4 = temp;
		}

		// intersection point in case of infinite slopes
		double x;
		double y;

		// infinite slope m1
		if (x1 == x2)
		{
			x = x1;
			y = m2 * x1 + c2;
			if (x > x3 && x < x4 && y > y1 && y <y2)
				return Point(x, y);
			else
				return Point(HUGE_VAL, HUGE_VAL);
		}

		// infinite slope m2
		if (x3 == x4)
		{
			x = x3;
			y = m1 * x3 + c1;
			if (x > x1 && x < x2 && y > y3 && y < y4)
				return Point(x, y);
			else
				return Point(HUGE_VAL, HUGE_VAL);
		}
		
		// compute lines intersection point
		x = (c2 - c1) / (m1 - m2);

		// see whether x in in both ranges [x1, x2] and [x3, x4]
		if (x > x1 && x < x2 && x > x3 && x < x4)
			return Point(x, m1 * x + c1);
  
		return Point(HUGE_VAL, HUGE_VAL);
	}
}
