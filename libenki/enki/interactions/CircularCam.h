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

#ifndef __ENKI_CIRCULARCAM_H
#define __ENKI_CIRCULARCAM_H

#include "../Interaction.h"
#include "../PhysicalEngine.h"

#include <valarray>

/*!	\file CircularCam.h
	\brief Header of the 1D circular camera
*/
namespace Enki
{
	//! Functor for pixel operation
	/*! This functor is to be called for each pixel. It should perform pixel operations such as depth buffer test
		\ingroup interaction */
	struct PixelOperationFunctor
	{
		//! Virtual destructor, do nothing
		virtual ~PixelOperationFunctor() { }
		//! Modify the pixel and depth buffer² for a given object color and distance²
		virtual void operator()(double &zBuffer2, Color &pixelBuffer, const double &objectDist2, const Color &objectColor) = 0;
	};
	
	//! 1D Circular camera
	/*! \ingroup interaction */
	class CircularCam : public LocalInteraction
	{
	protected:
		//! Position offset based on owner position
		Vector positionOffset;
		//! Height above ground, the camera will not see any object of smaller height
		double height;
		//! Absolute position in the world, updated on init()
		Vector absPos;
		//! Absolute angle in the world, updated on init()
		double absOrientation;

	public:
		//! zbuffer (array of size pixelCount of double)
		std::valarray<double> zbuffer;
		//! Image (array of size pixelCount of Color)
		std::valarray<Color> image;
		//! Field of view = [-fieldOfView; + fieldOfView]. [0; PI/2]
		double fieldOfView;
		//! Angular offset based on owner angle
		double angleOffset;
		
		//! Fog switch, exponential decay of light with distance
		bool useFog;
		//! Density of fog, used to compute light attenuation with the function: light = light0 * exp(-fogDensity * distance)
		double fogDensity;
		//! Minimum incoming light, otherwise 0. Only used if useFog is true
		Color lightThreshold;
		
		//! Pointer to active pixel operation
		PixelOperationFunctor *pixelOperation;

	public :
		//! Constructor. r is the vision radius and owner is the Sbot the camera belongs to
		CircularCam(Robot *owner, Vector pos, double height, double orientation, double fieldOfView, unsigned pixelCount);
		//! Destructor
		virtual ~CircularCam(){}
		virtual void init();
		virtual void objectStep (double dt, PhysicalObject *po, World *w);
		virtual void wallsStep(World *w);
		virtual void finalize(double dt);
		
		//! Return the absolute position (world coordinates) of the camera, updated at each time step on init()
		Point getAbsolutePosition(void) { return absPos; }
		//! Return the absolute orientation (world coordinates) of the camera, updated at each time step on init()
		double getAbsoluteOrientation(void) { return absOrientation; }
		
	protected:
		//! Return linear interpolated value between d0 and d1, given a sensorvalue sv between s0 and s1
		double interpolateLinear(double s0, double s1, double sv, double d0, double d1);
		//! Draw a textured line from point p0 to p1 using texture - WTF are p0 and p1??
		void drawTexturedLine(const Point &p0, const Point &p1, const Texture &texture);
	};
}
#endif

