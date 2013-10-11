/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2013 Stephane Magnenat <stephane at magnenat dot net>
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
	/*!
		This functor is to be called for each pixel. It should perform pixel operations such as depth buffer test.
		\ingroup interaction
	*/
	struct PixelOperationFunctor
	{
		//! Virtual destructor, do nothing
		virtual ~PixelOperationFunctor() { }
		//! Modify the pixel and depth buffer² for a given object color and distance²
		virtual void operator()(double &zBuffer2, Color &pixelBuffer, const double &objectDist2, const Color &objectColor) = 0;
	};
	
	
	//! 1D Circular camera
	/*!
		The maximum aperture angle of this camera is PI, so this is not an omnicam.
		Pixels start at -halfFieldOfView and then follow mathematical orientation
		\ingroup interaction
	*/
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
		//! zbuffer: distances at square (array of size pixelCount of double)
		std::valarray<double> zbuffer;
		//! Image (array of size pixelCount of Color)
		std::valarray<Color> image;
		//! Field of view = [-halfFieldOfView; + halfFieldOfView]. [0; PI/2]
		double halfFieldOfView;
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
		//! Constructor.
		/*!
			\param owner robot this camera is attached to
			\param pos position of this camera on the robot
			\param height height of this camera with respect to ground
			\param orientation orientation of this camera with respect to the robot front
			\param halfFieldOfView half aperture of the camera. The real field of view is twice this value [0; PI/2]
			\param pixelCount number of pixel to cover the full field of view
		*/
		CircularCam(Robot *owner, Vector pos, double height, double orientation, double halfFieldOfView, unsigned pixelCount);
		//! Destructor
		virtual ~CircularCam(){}
		virtual void init(double dt, World* w);
		virtual void objectStep(double dt, World *w, PhysicalObject *po);
		virtual void wallsStep(double dt, World* w);
		virtual void finalize(double dt, World* w);
		
		//! Change the sight range of the camera
		void setRange(double range);
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
	
	
	//! 1D omnidirectional circular camera, based on 2 CircularCam
	//! Pixels start at -PI and then follow mathematical orientation to PI
	/*! \ingroup interaction */
	class OmniCam : public LocalInteraction
	{
	public:
		//! zbuffer: distances at square (array of size pixelCount of double)
		std::valarray<double> zbuffer;
		//! Image (array of size pixelCount of Color)
		std::valarray<Color> image;
		
	protected:
		//! Cameras doing the real job, first part
		CircularCam cam0;
		//! Cameras doing the real job, second part
		CircularCam cam1;

	public :
		//! Constructor
		/*!
			\param owner robot this camera is attached to
			\param height height of this camera with respect to ground
			\param halfPixelCount half the number of pixel to cover the full 2*PI field of view
		*/
		OmniCam(Robot *owner, double height, unsigned halfPixelCount);
		//! Destructor
		virtual ~OmniCam(){}
		virtual void init(double dt, World* w);
		virtual void objectStep(double dt, World *w, PhysicalObject *po);
		virtual void wallsStep(double dt, World* w);
		virtual void finalize(double dt, World* w);
		//! Change the sight range of the camera
		void setRange(double range);
		//! Change the fog condition for this camera. If useFog is true, an exponential fog with density will be used. Additionally, a threshold can be applied on the resulting color
		void setFogConditions(bool useFog, double density = 0.0, Color threshold = Color::black);
		//! Change the pixel operation functor
		void setPixelOperationFunctor(PixelOperationFunctor *pixelOperationFunctor);
	};
}
#endif

