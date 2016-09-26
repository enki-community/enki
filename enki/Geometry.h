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

#ifndef __ENKI_GEOMETRY_H
#define __ENKI_GEOMETRY_H

#ifdef WIN32
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif
#include <cmath>
#include <vector>
#include <limits>
#include <ostream>
#include <algorithm>

#ifdef _MSC_VER
#define round(x) floor((x) + 0.5)
#endif

/*!	\file Geometry.h
	\brief The mathematic classes for 2D geometry
*/

namespace Enki
{
	//! A vector in a 2D space
	/*! \ingroup an 
		Notation of values and constructor order arguments are column based:
		\code
		x
		y
		\endcode
	*/
	struct Vector
	{
		//! x component
		double x;
		//! y component
		double y;
	
		//! Constructor, create vector with coordinates (0, 0)
		Vector() { x = y = 0; }
		//! Constructor, create vector with coordinates (v, v)
		Vector(double v) { this->x = v; this->y = v; }
		//! Constructor, create vector with coordinates (x, y)
		Vector(double x, double y) { this->x = x; this->y = y; }
		//! Constructor, create vector with coordinates (array[0], array[1])
		Vector(double array[2]) { x = array[0]; y = array[1]; }
	
		//! Add vector v component by component
		void operator +=(const Vector &v) { x += v.x; y += v.y; }
		//! Substract vector v component by component
		void operator -=(const Vector &v) { x -= v.x; y -= v.y; }
		//! Multiply each component by scalar f
		void operator *=(double f) { x *= f; y *= f; }
		//! Divive each component by scalar f
		void operator /=(double f) { x /= f; y /= f; }
		//! Add vector v component by component and return the resulting vector
		Vector operator +(const Vector &v) const { Vector n; n.x = x + v.x; n.y = y + v.y; return n; }
		//! Substract vector v component by component and return the resulting vector
		Vector operator -(const Vector &v) const { Vector n; n.x = x - v.x; n.y = y - v.y; return n; }
		//! Multiply each component by scalar f and return the resulting vector
		Vector operator /(double f) const { Vector n; n.x = x/f; n.y = y/f; return n; }
		//! Divive each component by scalar f and return the resulting vector
		Vector operator *(double f) const { Vector n; n.x = x*f; n.y = y*f; return n; }
		//! Invert this vector
		Vector operator -() const { return Vector(-x, -y); }
	
		//! Return the scalar product with vector v
		double operator *(const Vector &v) const { return x*v.x + y*v.y; }
		//! Return the norm of this vector
		double norm(void) const { return sqrt(x*x + y*y); }
		//! Return the square norm of this vector (and thus avoid a square root)
		double norm2(void) const { return x*x+y*y; }
		//! Return the cross product with vector v
		double cross(const Vector &v) const { return x * v.y - y * v.x; }
		//! Return a unitary vector of same direction
		Vector unitary(void) const { if (norm() < std::numeric_limits<double>::epsilon()) return Vector(); return *this / norm(); }
		//! Return the angle with the horizontal (arc tangant (y/x))
		double angle(void) const { return atan2(y, x); }
		//! Return the perpendicular of the same norm in math. orientation (CCW)
		Vector perp(void) const { return Vector(-y, x); }
		
		//! Return the cross with (this x other) a (virtual, as we are in 2D) perpendicular vector (on axis z) of given norm. 
		Vector crossWithZVector(double l) const { return Vector(y * l, -x * l); }
		//! Return the cross from (other x this) a (virtual, as we are in 2D) perpendicular vector (on axis z) of given norm. 
		Vector crossFromZVector(double l) const { return Vector(-y * l, x * l); }
		
		//! Comparison operator
		bool operator <(const Vector& that) const { if (this->x == that.x) return (this->y < that.y); else return (this->x < that.x); }
	};
	
	//! Print a vector to a stream
	std::ostream & operator << (std::ostream & outs, const Vector &vector);
	
	//! A point in a 2D space, another name for a vector
	/*! \ingroup an */
	typedef Vector Point;
	
	//! A 2x2 matrix
	/*! \ingroup an 
		Notation of values and constructor order arguments are column based:
		\code
		a c
		b d
		\endcode
	*/
	struct Matrix22
	{
		// line-column component
		//! 11 components
		double _11;
		//! 21 components
		double _21;
		//! 12 components
		double _12;
		//! 22 components
		double _22;
	
		//! Constructor, create matrix with 0
		Matrix22() { _11 = _21 = _12 = _22 = 0; }
		//! Constructor, create matrix with _11 _21 _12 _22
		Matrix22(double _11, double _21, double _12, double _22) { this->_11 = _11; this->_21 = _21; this->_12 = _12; this->_22 = _22; }
		//! Constructor, create rotation matrix of angle alpha in radian
		Matrix22(double alpha) { _11 = cos(alpha); _21 = sin(alpha); _12 = -_21; _22 = _11; }
		//! Constructor, create matrix with array[0] array[1] array[2] array[3]
		Matrix22(double array[4]) { _11=array[0]; _21=array[1]; _12=array[2]; _22=array[3]; }
		
		//! Fill with zero
		void zeros() { _11 = _21 = _12 = _22 = 0; }
		
		//! Add matrix v component by component
		void operator +=(const Matrix22 &v) { _11 += v._11; _21 += v._21; _12 += v._12; _22 += v._22; }
		//! Substract matrix v component by component
		void operator -=(const Matrix22 &v) { _11 -= v._11; _21 -= v._21; _12 -= v._12; _22 -= v._22; }
		//! Multiply each component by scalar f
		void operator *=(double f) { _11 *= f; _21 *= f; _12 *= f; _22 *= f; }
		//! Divive each component by scalar f
		void operator /=(double f) { _11 /= f; _21 /= f; _12 /= f; _22 /= f; }
		//! Add matrix v component by component and return the resulting matrix
		Matrix22 operator +(const Matrix22 &v) const { Matrix22 n; n._11 = _11 + v._11; n._21 = _21 + v._21; n._12 = _12 + v._12; n._22 = _22 + v._22; return n; }
		//! Subtract matrix v component by component and return the resulting matrix
		Matrix22 operator -(const Matrix22 &v) const { Matrix22 n; n._11 = _11 - v._11; n._21 = _21 - v._21; n._12 = _12 - v._12; n._22 = _22 - v._22; return n; }
		//! Multiply each component by scalar f and return the resulting matrix
		Matrix22 operator *(double f) const { Matrix22 n; n._11 = _11 * f; n._21 = _21 * f; n._12 = _12 * f; n._22 = _22 * f; return n; }
		//! Divide each component by scalar f and return the resulting matrix
		Matrix22 operator /(double f) const { Matrix22 n; n._11 = _11 / f; n._21 = _21 / f; n._12 = _12 / f; n._22 = _22 / f; return n; }
		//! Return the transpose of the matrix
		Matrix22 transpose() const { Matrix22 n; n._11 = _11; n._21 = _12; n._12 = _21; n._22 = _22; return n; }
		
		//! Multiply vector v and return the resulting vector
		Point operator*(const Point &v) const { Point n; n.x = v.x*_11 + v.y*_12; n.y = v.x*_21 + v.y*_22; return n; }
		
		//! Creates a diagonal matrix
		static Matrix22 fromDiag(double _1, double _2 ) { return Matrix22(_1, 0, 0, _2); }
		//! Create an identity matrix
		static Matrix22 identity() { return fromDiag(1,1); }
	};
	
	//! A segment in a 2D space, basically two points
	/*! \ingroup an */
	struct Segment
	{
		//! Constructor, create segment from point (ax, ay) to point (bx, by)
		Segment(double ax, double ay, double bx, double by) { this->a.x = ax; this->a.y = ay; this->b.x = bx; this->b.y = by; }
		//! Constructor, create segment from point (array[0], array[1]) to point (array[2], array[3])
		Segment(double array[4]) { a.x = array[0]; a.y = array[1]; b.x = array[2]; b.y = array[3]; }
		//! Constructor, create segment from point p1 to point p2
		Segment(const Point &p1, const Point &p2) { a = p1; b = p2; }
		
		//! Start point
		Point a;
		//! End point
		Point b;
	
		//! Compute the distance of p to this segment
		double dist(const Point &p) const
		{
			const Vector n(a.y-b.y, b.x-a.x);
			const Vector u = n.unitary();
			const Vector ap = p-a;
			return ap * u;
		}
	
		//! Return true if o intersect this segment
		bool doesIntersect(const Segment &o) const
		{
			const double s2da = dist (o.a);
			const double s2db = dist (o.b);
			const double s1da = o.dist (a);
			const double s1db = o.dist (b);
			return (s2da*s2db<0) && (s1da*s1db<0);
		}
		
		//! Return the middle point
		Point getMiddlePoint() const
		{
			return (a + b) / 2;
		}
	};
	
	//! Polygone, which is a vector of points. Anti-clockwise, standard trigonometric orientation
	/*! \ingroup an */
	struct Polygone: public std::vector<Point>
	{
		//! Return the i-th segment
		Segment getSegment(size_t i) const
		{
			return Segment((*this)[i], (*this)[(i + 1) % size()]);
		}
		
		//! Return true if p is inside this polygone
		bool isPointInside(const Point& p) const
		{
			for (size_t i = 0; i < size(); i++)
			{
				if (getSegment(i).dist(p) < 0)
					return false;
			}
			return true;
		}
		
		//! Get the axis aligned bounding box and return whether it exists
		bool getAxisAlignedBoundingBox(Point& bottomLeft, Point& topRight) const
		{
			if (empty())
				return false;
			
			bottomLeft = (*this)[0];
			topRight = (*this)[0];
			
			extendAxisAlignedBoundingBox(bottomLeft, topRight);
			
			return true;
		}
		
		//! Extend an axis aligned bounding box with this object
		void extendAxisAlignedBoundingBox(Point& bottomLeft, Point& topRight) const
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
		
		//! Return the bounding radius of this polygon
		double getBoundingRadius() const
		{
			double radius = 0;
			for (size_t i = 0; i < size(); i++)
				radius = std::max<double>(radius, (*this)[i].norm());
			return radius;
		}
		
		//! Translate of a specific distance
		void translate(const Vector& delta)
		{
			for (iterator it = begin(); it != end(); ++it)
				*it += delta;
		}
		
		//! Translate of a specific distance, overload for convenience
		void translate(const double x, const double y)
		{
			translate(Vector(x,y));
		}
		
		//! Rotate by a specific angle
		void rotate(const double angle)
		{
			Matrix22 rot(angle);
			for (iterator it = begin(); it != end(); ++it)
				*it = rot * (*it);
		}
		
		//! Flip coordinates on x
		void flipX()
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
		
		//! Flip coordinates on y
		void flipY()
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
		
		//! Operator to add point inline
		Polygone& operator << (const Point& p) { push_back(p); return *this; }
	};
	
	//! Print a polygone to a stream
	/*! \ingroup an */
	std::ostream & operator << (std::ostream & outs, const Polygone &polygone);
	
	//! Normlize an angle to be between -PI and +PI.
	/*! \ingroup an */
	inline double normalizeAngle(double angle)
	{
		while (angle > M_PI)
			angle -= 2*M_PI;
		while (angle < -M_PI)
			angle += 2*M_PI;
		return angle;
	}
	
	#if 0
	// NOTE: unused for now
	//! Return the area of the triangle formed by a,b,c in mathematical order (CCW)
	inline double getTriangleArea(const Point &a, const Point &b, const Point &c)
	{
		return (a.x-c.x) * (b.y-c.y) - (a.y-c.y) * (b.x-c.x);
	}
	#endif
  
	//! get the intersection point between two line segments
	//! returns Point(HUGE_VAL, HUGE_VAL) if there's no intersection
	//! added by yvan.bourquin@epfl.ch
	/*! \ingroup an */
	inline Point getIntersection(const Segment &s1, const Segment &s2)
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
	
	//! Returns 2 times the signed triangle area. 
	/** positive if abc winds counter-clockwise,
	   negative if abc winds clockwise,
	   zero if abc is degenerate.
	   See: Real-time collision detection, C. Ericson, Page 152 */
	/*! \ingroup an */
	inline double getTriangleAreaTwice(const Point &a, const Point &b, const Point &c)
	{
		return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
	}
	
	//! Returns signed height of triangle abc with base ab
	/** positive if abc winds counter-clockwise
	   negative if abc winds clockwise,
	   zero if abc is degenerate. */
	/*! \ingroup an */
	inline double getTriangleHeight(const Point &a, const Point &b, const Point &c)
	{
		const double ba_norm = (b-a).norm();
		if (ba_norm < std::numeric_limits<double>::epsilon())
			return 0;
		
		// area of the parallelogram divided by the length of the base
		return getTriangleAreaTwice(a, b, c) / ba_norm;
	}
}

#endif
