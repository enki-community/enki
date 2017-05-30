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

#include "../enki/Geometry.h"
#include <iostream>

using namespace Enki;
using namespace std;

#define CHECK_INTERSECT_POLYGON_CIRCLE(func, val) \
	if (func != val) { \
		cerr << #func << " with r = " << r << " intersect result " << func << " instead of " << val << endl; \
		exit(1); \
	} else if (val) { \
		; \
	}
//cout << #func << " with r = " << r << " intersected at " << cp << " with mtv " << mtv << endl; \

#define CHECK_INTERSECT_SEGMENT_SEGMENT(func, val, p) \
	if (func != val) { \
		cerr << #func << " intersect result " << func << " instead of " << val << endl; \
		exit(1); \
	} else if (val) { \
		if ((fabs(intersectionPoint.x - p.x) > 1e-9) || (fabs(intersectionPoint.y - p.y) > 1e-9)) { \
			cerr << #func << " should intersect at " << p << " but returned " << intersectionPoint << endl; \
			exit(2); \
		} \
	}

void testPolygonCircleIntersection()
{
	//cout << "polygon circle intersection" << endl;
	
	Vector mtv;
	Point cp;
	
	Polygon square;
	square.push_back(Point(1, 1));
	square.push_back(Point(9, 1));
	square.push_back(Point(9, 9));
	square.push_back(Point(1, 9));
	
	// should intersect
	for (double r = 1.01*sqrt(2); r<30; r = r*r)
	{
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(0, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(5, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(10, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(10, 5), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(10, 10), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(5, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(0, 10), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(0, 5), r, mtv, cp), true);
		
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(5, 5), r, mtv, cp), true);
	}
	
	// should not intersect
	for (double r = -2; r < 12; r += 0.1)
	{
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(r, 0), 0.9, mtv, cp), false);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(r, 10), 0.9, mtv, cp), false);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(0, r), 0.9, mtv, cp), false);
		CHECK_INTERSECT_POLYGON_CIRCLE(square.doesIntersect(Point(10, r), 0.9, mtv, cp), false);
	}
	
	Polygon rect;
	rect.push_back(Point(1, 1));
	rect.push_back(Point(9, 1));
	rect.push_back(Point(9, 2));
	rect.push_back(Point(1, 2));
	
	// should intersect
	for (double r = 1.01*sqrt(2); r<5; r += 0.01)
	{
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(0, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(5, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(10, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(10, 1.5), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(10, 3), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(5, 0), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(0, 3), r, mtv, cp), true);
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(0, 3), r, mtv, cp), true);
		
		CHECK_INTERSECT_POLYGON_CIRCLE(rect.doesIntersect(Point(5, 1.5), r, mtv, cp), true);
	}
	
	// special cases that did not work
	Polygon rect2;
	rect2.push_back(Point(5, 1));
	rect2.push_back(Point(-5, 1));
	rect2.push_back(Point(-5, -1));
	rect2.push_back(Point(5, -1));
	double r = 4;
	CHECK_INTERSECT_POLYGON_CIRCLE(rect2.doesIntersect(Point(7.18797, -3.12911), r, mtv, cp), true);
}

void testSegmentSegmentIntersection()
{
	Segment a(-1,0,1,0);       // base
	Segment b0(0,-1,0,1);      // yes: perp
	Segment b1(-1,-1,1,1);     // yes: diag
	Segment b2(-1,1,1,-1);     // yes: diag
	Segment c0(0.9,0,1,0);     // yes: colin
	Segment c1(1.0,0,2,0);     // yes: point
	Segment d0(-2,0,-0.9,0);   // yes: colin
	Segment d1(-2,0,-1,0);     // yes: point
	Segment e0(-1.1,-1,-1.1,1);// no: left
	Segment e1(1.1,-1,1.1,1);  // no: right
	Segment e2(0,0.1,0,1);     // no: top
	Segment e3(0,-0.1,0,-1);   // no: bottom
	Segment null0(0,0,0,0);    // null segment at 0
	Segment null1(1,1,1,1);    // null segment at 1
	
	Point intersectionPoint;
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(b0, &intersectionPoint), true, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(b0.doesIntersect(a, &intersectionPoint), true, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(b1, &intersectionPoint), true, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(b1.doesIntersect(a, &intersectionPoint), true, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(b2, &intersectionPoint), true, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(b2.doesIntersect(a, &intersectionPoint), true, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(c0, &intersectionPoint), true, Point(0.95,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(c0.doesIntersect(a, &intersectionPoint), true, Point(0.95,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(c1, &intersectionPoint), true, Point(1,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(c1.doesIntersect(a, &intersectionPoint), true, Point(1,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(d0, &intersectionPoint), true, Point(-0.95,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(d0.doesIntersect(a, &intersectionPoint), true, Point(-0.95,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(d1, &intersectionPoint), true, Point(-1,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(d1.doesIntersect(a, &intersectionPoint), true, Point(-1,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(e0, &intersectionPoint), false, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(e0.doesIntersect(a, &intersectionPoint), false, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(e1, &intersectionPoint), false, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(e1.doesIntersect(a, &intersectionPoint), false, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(e2, &intersectionPoint), false, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(e2.doesIntersect(a, &intersectionPoint), false, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(e3, &intersectionPoint), false, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(e3.doesIntersect(a, &intersectionPoint), false, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(a.doesIntersect(null0, &intersectionPoint), true, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(null0.doesIntersect(a, &intersectionPoint), true, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(b0.doesIntersect(null0, &intersectionPoint), true, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(null0.doesIntersect(b0, &intersectionPoint), true, Point(0,0));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(null0.doesIntersect(null0, &intersectionPoint), true, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(null1.doesIntersect(null1, &intersectionPoint), true, Point(1,1));
	
	CHECK_INTERSECT_SEGMENT_SEGMENT(null0.doesIntersect(null1, &intersectionPoint), false, Point(0,0));
	CHECK_INTERSECT_SEGMENT_SEGMENT(null1.doesIntersect(null0, &intersectionPoint), false, Point(0,0));
}

int main()
{
	testPolygonCircleIntersection();
	testSegmentSegmentIntersection();
	
	return 0;
}
