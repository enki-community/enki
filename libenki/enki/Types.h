/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2006 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <antoine dot beyeler at epfl dot ch>
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

#ifndef __AN_TYPES_H
#define __AN_TYPES_H

#include <valarray>
#include <cassert>

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
	/*! \ingroup an */
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
		const double& operator[](size_t i) const { assert((i >= 0) && (i < 4)); return components[i]; }
		//! access component i
		double& operator[](size_t i) { assert((i >= 0) && (i < 4)); return components[i]; }
		
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
		double gray() const { return (components[0] + components[1] + components[2]) / 3; }
		
		//! black (0, 0, 0)
		static const Color black;
		//! white (1, 1, 1)
		static const Color white;
		//! red (1, 0, 0)
		static const Color red;
		//! green (0, 1, 0)
		static const Color green;
		//! blue (0, 0, 1)
		static const Color blue;
	};

	//! A texture
	/*! \ingroup an */
	typedef std::valarray<Color> Texture;
}

#endif
