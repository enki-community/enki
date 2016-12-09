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

#ifndef __ENKI_TYPES_H
#define __ENKI_TYPES_H

#include <vector>
#include <sstream>
#include <string>
#include <cassert>
#include <stdint.h> // C99 in waiting for widespread C++11 support

/*!	\file Types.h
	\brief Basic useful types
*/

/*!	\defgroup an Some software bricks extracted from the An support library.
	This includes the Color type, a collection of several random distributions and
	some geometric primitives.
*/

namespace Enki
{
	//! A color in RGBA
	struct Color
	{
		//! RGBA values in range [0..1]
		double components[4];
		
		//! Constructor from separated components
		Color(double r = 0.0, double g = 0.0, double b = 0.0, double a = 1.0)
		{
			components[0] = r;
			components[1] = g;
			components[2] = b;
			components[3] = a;
		}
		
		//! access component i
		const double& operator[](size_t i) const { assert(i < 4); return components[i]; }
		//! access component i
		double& operator[](size_t i) { assert(i < 4); return components[i]; }
		
		// operations with scalar
		//! Add d to each component
		void operator +=(double d) { for (size_t i=0; i<3; i++) components[i] += d; }
		//! Add d to each component and return result in a new color. I'm left unchanged
		Color operator +(double d) const { Color c; for (size_t i=0; i<3; i++) c.components[i] = components[i] + d; return c; }
		
		//! Substract d from each component
		void operator -=(double d) { for (size_t i=0; i<3; i++) components[i] -= d; }
		//! Substract d from each component and return result in a new color. I'm left unchanged
		Color operator -(double d) const { Color c; for (size_t i=0; i<3; i++) c.components[i] = components[i] - d; return c; }
		
		//! Multiply each component with d
		void operator *=(double d) { for (size_t i=0; i<3; i++) components[i] *= d; }
		//! Multiply each component with d and return result in a new color. I'm left unchanged
		Color operator *(double d) const { Color c; for (size_t i=0; i<3; i++) c.components[i] = components[i] * d; return c; }
		
		//! Divide each component with d
		void operator /=(double d) { for (size_t i=0; i<3; i++) components[i] /= d; }
		//! Divide each component with d and return result in a new color. I'm left unchanged
		Color operator /(double d) const { Color c; for (size_t i=0; i<3; i++) c.components[i] = components[i] / d; return c; }
		
		// operation with another color
		//! Add oc's components to ours
		void operator +=(const Color &oc) { for (size_t i=0; i<3; i++) components[i] += oc.components[i]; }
		//! Add oc's components to ours and return result in a new color. I'm left unchanged
		Color operator +(const Color &oc) const { Color c; for (size_t i=0; i<3; i++) c.components[i] = components[i] + oc.components[i]; return c; }
		
		//! Substract oc's components to ours
		void operator -=(const Color &oc) { for (size_t i=0; i<3; i++) components[i] -= oc.components[i]; }
		//! Substract oc's components to ours and return result in a new color. I'm left unchanged
		Color operator -(const Color &oc) const { Color c; for (size_t i=0; i<3; i++) c.components[i] = components[i] - oc.components[i]; return c; }
		
		//! Compare all components and return true if they're the same.
		bool operator ==(const Color &c) const { for (size_t i=0; i<4; i++) if (components[i] != c.components[i]) return false; return true; }
		//! Compare all components and return false if they're the same.
		bool operator !=(const Color &c) const { return !(*this == c); }
		//! Threshold the color using limit. For each component, if value is below limit, set it to 0
		void threshold(const Color &limit) { for (size_t i=0; i<3; i++) components[i] = components[i] > limit.components[i] ? components[i] : 0; }
		//! Return the grey level value
		double toGray() const { return (components[0] + components[1] + components[2]) / 3; }
		
		//! Return a string describing this color
		std::string toString() const { std::ostringstream oss; oss << *this; return oss.str(); }
		
		//! Red component value getter
		double r() const { return components[0]; }
		
		//! Set the value of red component
		void setR(double value) { components[0] = value; }
		
		//! Green component value getter
		double g() const { return components[1]; }
		
		//! Set the value of green component
		void setG(double value) { components[1] = value; }
		
		//! Blue component value getter
		double b() const { return components[2]; }
		
		//! Set the value of blue component
		void setB(double value) { components[2] = value; }
		
		//! Alpha component value getter
		double a() const { return components[3]; }
		
		//! Set the value of alpha component
		void setA(double value) { components[3] = value; }
		
		//! Build from an ARGB uint32_t (0xAARRGGBB in little endian)
		static Color fromARGB(uint32_t color);
		//! Build from an ABGR uint32_t (0xAABBGGRR in little endian)
		static Color fromABGR(uint32_t color);
		//! Pack into ABGR uint32_t (0xAABBGGRR in little endian)
		static uint32_t toARGB(Color color);
		
		//! black (0, 0, 0)
		static const Color black;
		//! white (1, 1, 1)
		static const Color white;
		//! gray (0.5, 0.5, 0.5)
		static const Color gray;
		//! red (1, 0, 0)
		static const Color red;
		//! green (0, 1, 0)
		static const Color green;
		//! blue (0, 0, 1)
		static const Color blue;
		
		friend std::ostream & operator<<(std::ostream &os, const Color& c);
	};
	
	//! A texture
	typedef std::vector<Color> Texture;
	
	//! Textures for all sides of an object
	typedef std::vector<Texture> Textures;
}

#endif
