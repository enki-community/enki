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

#include "AliceCam.h"
#include <iostream>

/*! \file AliceCam.cpp
	\brief Implementation of the Alice 2 pixels camera interaction
*/

namespace Enki
{
	AliceCam::AliceCam (Robot *me)
	{
		this->owner = me;
		this->cvaluestarboard = -1;
		this->cvalueport = -1;
	}

	void AliceCam::step(double dt, World *w)
	{
		Vector v1 = Vector(w->w,w->h)-this->owner->pos;
		Vector v2 = Vector(0,w->h)-this->owner->pos;

		// camera port
		Vector dir(cos(this->owner->angle - M_PI/10), sin(this->owner->angle - M_PI/10));

		if ( (dir.cross(v1) < 0) && (dir.cross(v2) > 0))
		{
			// nest
			cvalueport = 1;
		}
		else
		{
			// not nest
			cvalueport = 0;
		}

		// camera starboard
		dir=Vector(cos(this->owner->angle + M_PI/10), sin(this->owner->angle + M_PI/10));

		if ( (dir.cross(v1) < 0) && (dir.cross(v2) > 0))
		{
			// nest
			cvaluestarboard = 1;
		}
		else
		{
			// not nest
			cvaluestarboard = 0;
		}

	}

	double AliceCam::getCValueStarboard()
	{
		return cvaluestarboard;
	}
	
	double AliceCam::getCValuePort()
	{
		return cvalueport;
	}

}


