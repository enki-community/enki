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
		//! Comparison operator
		bool operator ==(const Vector& that) const { return (this->x == that.x) && (this->y == that.y); }
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
		static Matrix22 identity() { return fromDiag(1, 1); }
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
		double dist(const Point &p) const;
	
		//! Return true if o intersect this segment
		bool doesIntersect(const Segment &that, Point* intersectionPoint = 0) const;
		
		//! Return the middle point
		Point getMiddlePoint() const { return (a + b) / 2; }
		
		//! Return a vector of the direction of the segment
		Vector getDirection() const { return b - a; }
		
		//! Return whether the segment is degenerate, if a == b
		bool isDegenerate() const { return a == b; }
	};
	
	//! Print a segment to a stream
	/*! \ingroup an */
	std::ostream & operator << (std::ostream & outs, const Segment &segment);
	
	//! Polygon, which is a vector of points. Anti-clockwise, standard trigonometric orientation
	/*! \ingroup an */
	struct Polygon: public std::vector<Point>
	{
		//! Return the i-th segment
		Segment getSegment(size_t i) const;
		
		//! Return true if p is inside this polygon
		bool isPointInside(const Point& p) const;
		
		//! Get the axis aligned bounding box and return whether it exists
		bool getAxisAlignedBoundingBox(Point& bottomLeft, Point& topRight) const;
		
		//! Extend an axis aligned bounding box with this object
		void extendAxisAlignedBoundingBox(Point& bottomLeft, Point& topRight) const;
		
		//! Return the bounding radius of this polygon
		double getBoundingRadius() const;
		
		//! Translate of a specific distance
		void translate(const Vector& delta);
		
		//! Translate of a specific distance, overload for convenience
		void translate(const double x, const double y) { translate(Vector(x, y)); }
		
		//! Rotate by a specific angle
		void rotate(const double angle);
		
		//! Flip coordinates on x
		void flipX();
		
		//! Flip coordinates on y
		void flipY();
		
		//! Operator to add point inline
		Polygon& operator << (const Point& p) { push_back(p); return *this; }
		
		//! Return true and set intersection arguments (passed by reference) if this intersects circle (center, r), return false and do not change anything otherwise
		/*!
			\param center center of circle
			\param r radius of circle
			\param mtv minimum translation vector, how much to move this for de-penetration, set if intersection happens
			\param intersectionPoint point where this touches circle, set if intersection happens
		*/
		bool doesIntersect(const Point& center, const double r, Vector& mtv, Point& intersectionPoint) const;
		
		//! Return true and set intersection arguments (passed by reference) if shape1 intersects shape2, return false and do not change anything otherwise
		/*!
			\param that second polygon
			\param mtv minimum translation vector for de-penetration, how much to move this for de-penetration, set if intersection happens
			\param intersectionPoint point where this touches that, set if intersection happens
		*/
		bool doesIntersect(const Polygon& that, Vector& mtv, Point& intersectionPoint) const;
	};
	
	//! Print a polygon to a stream
	/*! \ingroup an */
	std::ostream & operator << (std::ostream & outs, const Polygon &polygon);
	
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
}

#endif
