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

#ifndef __ENKI_RANDOM_H
#define __ENKI_RANDOM_H

#include <cmath>
#include <cstdlib>

/*!	\file Random.h
	\brief The mathematic classes for random work
*/

namespace Enki
{
	//! A fast random generator
	/*! \ingroup an */
	class FastRandom
	{
	private:
		unsigned long randx; //!< value used to compute next pseudo-random value
	
	public:
		//! Construct the random generator, initialize with a seed of 0
		FastRandom(void) { randx = 0; }
		//! Set the seed
		void setSeed(unsigned long seed) { randx = seed; }
		//! Get a random number between 0 and 2^31
		unsigned long get(void) { return (randx = randx*1103515245 + 12345) & 0x7fffffff; }
		//! Get a random double between 0 and range, use get() internally
		double getRange(double range) { return (static_cast<double>(get()) * range) / 2147483648.0; }
	};
	
	//! Return a number in [0;1[ in a uniform distribution
	/*! \ingroup an */
	inline double uniformRand(void)
	{
		return double(rand())/RAND_MAX;
	}
	
	//! Functor to be used with \<algorithm\>
	struct UniformRand
	{
		double from; //!< lower bound of uniform distribution
		double to;//!< upper bound of uniform distribution
		
		//! Constructor. Params define the range of the uniform distribution.
		UniformRand(double from = 0.0, double to = 1.0) { this->from = from; this->to = to; }
		
		//! Functor operator for use with, e.g., std::generate
		double operator()() const { return from + (to-from)*uniformRand(); }
	};
	
	//! Return a number between [0;max[ in integer in a uniform distribution
	/*! \ingroup an */
	inline unsigned intRand(unsigned max)
	{
		if (max)
			return rand() % max;
		else
			return 0;
	}
	
	//! Return true with a probability prob. If no argument is given, prob = 0.5
	/*! \ingroup an */
	inline bool boolRand(double prob = 0.5)
	{
		return uniformRand() < prob;
	}
	
	//! Return a random number with a gaussian distribution of a certain mean and standard deviation.
	/*! \ingroup an */
	inline double gaussianRand(double mean, double sigm)
	{
		// Generation using the Polar (Box-Mueller) method.
		// Code inspired by GSL, which is a really great math lib.
		// http://sources.redhat.com/gsl/
		// C++ wrapper available.
		// http://gslwrap.sourceforge.net/
		double r, x, y;
		
		// Generate random number in unity circle.
		do
		{
			x = uniformRand()*2 - 1;
			y = uniformRand()*2 - 1;
			r = x*x + y*y;
		}
		while (r > 1.0 || r == 0);
		
		// Box-Muller transform.
		return sigm * y * sqrt (-2.0 * log(r) / r) + mean;
	}
}

#endif
