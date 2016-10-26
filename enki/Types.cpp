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

#include "Types.h"

/*!	\file Types.cpp
	\brief Constants for basic usefull types
*/

namespace Enki
{
	Color Color::fromARGB(uint32_t color)
	{
		const unsigned a((color>>24)&0xff);
		const unsigned r((color>>16)&0xff);
		const unsigned g((color>>8)&0xff);
		const unsigned b((color>>0)&0xff);
		return Color(double(r)/255., double(g)/255., double(b)/255., double(a)/255.);
	}
	
	Color Color::fromABGR(uint32_t color)
	{
		const unsigned r((color>>0)&0xff);
		const unsigned g((color>>8)&0xff);
		const unsigned b((color>>16)&0xff);
		const unsigned a((color>>24)&0xff);
		return Color(double(r)/255., double(g)/255., double(b)/255., double(a)/255.);
	}

	uint32_t Color::toARGB(Color color)
	{
		const uint8_t r = (255*color.r());
		const uint8_t g = (255*color.g());
		const uint8_t b = (255*color.b());
		const uint8_t a = (255*color.a());
		return ((a<<24)|(r<<16)|(g<<8)|(b<<0));
	}

	const Color Color::black(0, 0, 0);
	const Color Color::white(1, 1, 1);
	const Color Color::gray(0.5, 0.5, 0.5);
	const Color Color::red(1, 0, 0);
	const Color Color::green(0, 1, 0);
	const Color Color::blue(0, 0, 1);
	
	std::ostream & operator<<(std::ostream &os, const Color& c)
	{
		return os << "(r = " << c.components[0] << ", g = " << c.components[1] << ", b = " << c.components[2] << ", a = " << c.components[3] << ")";
	}
}
