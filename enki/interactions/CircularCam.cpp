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

#include "CircularCam.h"
#include <algorithm>
#include <limits>
#include <assert.h>
#include <cmath>

/*!	\file CircularCam.cpp
	\brief Implementation of the 1D circular camera
*/
namespace Enki
{
	//! Standard depth test
	struct DepthTest : public PixelOperationFunctor
	{
		//! If objectDist2 < zBuffer2, then pixelBuffer = objectColor and zBuffer2 = objectDist2
		virtual void operator()(double &zBuffer2, Color &pixelBuffer, const double &objectDist2, const Color &objectColor)
		{
			if (objectDist2 < zBuffer2)
			{
				zBuffer2 = objectDist2;
				pixelBuffer = objectColor;
			}
		}
	} depthTest; //!< Standard depth test instance
	
	
	CircularCam::CircularCam(Robot *owner, Vector pos, double height, double orientation, double halfFieldOfView, unsigned pixelCount) :
		zbuffer(pixelCount),
		image(pixelCount)
	{
		this->r = std::numeric_limits<double>::max();
		this->owner = owner;
		this->positionOffset = pos;
		this->angleOffset = orientation;
		this->halfFieldOfView = halfFieldOfView;
		this->height = height;
		
		useFog = false;
		fogDensity = 0.0;
		lightThreshold = Color::black;
		
		pixelOperation = &depthTest;
	}

	void CircularCam::objectStep(double dt, World *w, PhysicalObject *po)
	{
		// if we see over the object
		if (height > po->getHeight())
			return;
		
		if (!po->isCylindric())
		{
			// object has a hull
			for (PhysicalObject::Hull::const_iterator it = po->getHull().begin(); it != po->getHull().end(); ++it)
			{
				if (height > it->getHeight())
					continue;
				
				const Polygon& shape = it->getTransformedShape();
				const size_t faceCount = shape.size();
				if (it->isTextured())
				{
					for (size_t i = 0; i<faceCount; i++)
						drawTexturedLine(shape[i], shape[(i+1) % faceCount], it->getTextures()[i]);
				}
				else
				{
					Texture texture(1, po->getColor());
					for (size_t i = 0; i<faceCount; i++)
						drawTexturedLine(shape[i], shape[(i+1) % faceCount], texture);
				}
			}
		}
		else
		{
			// object has no bounding surface, monocolor
			const double radius = po->getRadius();
			const Color& color = po->getColor();
			
			// compute basic parameter
			if (radius == 0)
				return;
			const Vector poCenter = po->pos - absPos;
			const double poDist = poCenter.norm();
			if (poDist == 0)
				return;
			const double poAngle = normalizeAngle(poCenter.angle() - absOrientation);
			const double poAperture = atan(radius / poDist);
			assert(poAperture > 0);
			
			// clip object
			const double poBegin = poAngle - poAperture;
			const double poEnd = poAngle + poAperture;
			
			if (poBegin > halfFieldOfView || poEnd < -halfFieldOfView)
				return;
			
			const double beginAngle = std::max(poBegin, -halfFieldOfView);
			const double endAngle = std::min(poEnd, halfFieldOfView);
			
			// compute first pixel used
			// formula is (beginAngle + fov) / pixelAngle, with
			// pixelAngle = 2fov / (numPix-1)
			const size_t firstPixelUsed = static_cast<size_t>(floor((zbuffer.size() - 1) * 0.5 * (beginAngle / halfFieldOfView + 1)));
			const size_t lastPixelUsed = static_cast<size_t>(ceil((zbuffer.size() - 1) * 0.5 * (endAngle / halfFieldOfView + 1)));
			
			const double poDist2 = poDist * poDist;
			for (size_t i = firstPixelUsed; i <= lastPixelUsed; i++)
			{
				// apply pixel operation to framebuffer
				(*pixelOperation)(zbuffer[i], image[i], poDist2, color);
			}
		}
	};
	
	double CircularCam::interpolateLinear(double s0, double s1, double sv, double d0, double d1)
	{
		return d0 + ( (sv - s0) / (s1 - s0) ) * (d1 - d0) ;
	}
	
	void CircularCam::drawTexturedLine(const Point &p0, const Point &p1, const Texture &texture)
	{
		bool invertTextureIndex = false;
		
		// Express p0 and p1 in the camera coordinate system.
		// In cam coord sys, x axis is the optical axis.
		const Matrix22 rot(-absOrientation);
		Vector p0c = rot * (p0 - absPos);
		Vector p1c = rot * (p1 - absPos);
		
		// Find angle of interest. Here we order p0 and p1 so that
		// p0 is the point with the smallest angle (in the [-pi;pi]
		// range).
		double p0dir = p0c.angle(); 			// [-pi;pi]
		double p1dir = p1c.angle(); 			// [-pi;pi]
		if (p0dir > p1dir)
		{
			std::swap(p0dir, p1dir);
			std::swap(p0c, p1c);
			invertTextureIndex = !invertTextureIndex;
		}
		
		const double beginAperture = -halfFieldOfView; 	// [-pi/2;0]
		const double endAperture = halfFieldOfView; 		// [0; pi/2]
		
		// check if the line is going "behind us"
		if (p1dir - p0dir > M_PI)
		{
			// dismiss line if not in field of view.
			if (p0dir < beginAperture && p1dir > endAperture)
				return;
			std::swap(p0dir, p1dir);
			std::swap(p0c, p1c);
			if (p1dir < -halfFieldOfView)
				p1dir += 2*M_PI;
			else
				p0dir -= 2*M_PI;
			invertTextureIndex = !invertTextureIndex;
		}
		
		// TODO: understand why this happens
		if (!(p1dir > p0dir))
			return;
		assert(p1dir > p0dir);
		
		// dismiss line if not in field of view.
		if ((p1dir < beginAperture) || (p0dir > endAperture))
			return;
		
		const size_t pixelCount = zbuffer.size();
		const double beginAngle = std::max(p0dir, beginAperture);
		const double endAngle = std::min(p1dir, endAperture);
		const double dAngle = 2*halfFieldOfView / (pixelCount - 1);
		
		// align begin and end angle to our sampled angles
 		const double beginIndex = ceil((beginAngle-beginAperture) / dAngle);
 		const double endIndex = floor((endAngle-beginAperture) / dAngle);
		const double alignedBeginAngle = beginAperture + beginIndex * dAngle;
		const double alignedEndAngle = beginAperture + endIndex * dAngle;

		const double beginPixel = round(interpolateLinear(beginAperture, endAperture, alignedBeginAngle, 0, pixelCount-1));
		const double endPixel = round(interpolateLinear(beginAperture, endAperture, alignedEndAngle, 0, pixelCount-1));
		
		// Optimization stuff
		const double x10 = p1c.x - p0c.x;
		const double y01 = p0c.y - p1c.y;
		const Vector p10c = p1c - p0c;
		double tanAngle;
		bool tanDirty = true;
		const double tanDelta = tan(dAngle);
		
		const size_t beginPixelIndex = static_cast<size_t>(beginPixel);
		const size_t endPixelIndex = static_cast<size_t>(endPixel);
		double angle = alignedBeginAngle;
		for (size_t i = beginPixelIndex; i <= endPixelIndex; i++)
		{
			double lambda = 0;
			
			if (fabs(angle) == M_PI/2)
			{
				lambda  = - p0c.x / x10;
				tanDirty = true;
			}
			else
			{
				// OPTIMIZATION: we compute tan(angle+n*dAngle) recursively, using
				// the formula tan(a+b) = (tan(a) + tan(b))/(1 - tan(a)*tan(b)
				if(tanDirty)
				{
					tanAngle = tan(angle);
					tanDirty = false;
				}
				else
					tanAngle = (tanAngle + tanDelta) / (1 - tanAngle * tanDelta);

				lambda = (p0c.y - p0c.x * tanAngle) / (tanAngle * x10 + y01);
			}
			
			assert(i < image.size());
			
			// Compute zbuffer and texture index.
			size_t texIndex;
			Vector p;
			if (lambda < 0)
			{
				p = p0c;
				texIndex = 0;
			}
			else if (lambda >= 1)
			{
				p = p1c;
				texIndex = texture.size() - 1;
			}
			else
			{
				p = p0c + p10c * lambda;
				texIndex = static_cast<size_t>(floor(lambda * texture.size()));
			}
			
			assert(texIndex < texture.size());
			
			// apply pixel only if distance is inferior to the current one
			const double z = p.norm2();
			if (zbuffer[i] > z)
			{
				if (invertTextureIndex)
					texIndex = texture.size() - texIndex - 1;
				image[i] = texture[texIndex];
				zbuffer[i] = z;
			}
			
			angle += dAngle;
		}
	}

	void CircularCam::init(double dt, World* w)
	{
		// compute absolute position and orientation
		const Matrix22 rot(owner->angle);
		absPos = owner->pos + rot * positionOffset;
		absOrientation = owner->angle + angleOffset;
		
		// fill zbuffer with infinite
		std::fill( &zbuffer[0], &zbuffer[zbuffer.size()], std::numeric_limits<double>::max() );
		std::fill( &image[0], &image[image.size()], w->color);
	}
	
	void CircularCam::wallsStep(double dt, World* w)
	{
		Texture texture(1, w->color);
		
		switch (w->wallsType)
		{
			// TODO: use world texture if any
			case World::WALLS_SQUARE:
			{
				drawTexturedLine(Point(0, 0), Point(w->w, 0), texture);
				drawTexturedLine(Point(w->w, 0), Point(w->w, w->h), texture);
				drawTexturedLine(Point(w->w, w->h), Point(0, w->h), texture);
				drawTexturedLine(Point(0, w->h), Point(0, 0), texture);
			}
			break;
			
			case World::WALLS_CIRCULAR:
			{
				const double r(w->r);
				const int segmentCount((r*2.*M_PI) / 10.);
				for (int i = 0; i < segmentCount; ++i)
				{
					const double angStart(((double)i * 2. * M_PI) / (double)segmentCount);
					const double angEnd(((double)(i+1) * 2. * M_PI) / (double)segmentCount);
					drawTexturedLine(
						Point(cos(angStart)*r, sin(angStart)*r),
						Point(cos(angEnd)*r, sin(angEnd)*r),
						texture
					);
				}
			}
			break;
			
			default:
			break;
		}
		
		// disable world texture for now
		/*if (w->wallTextures[0].size() > 0)
			drawTexturedLine(Point(0, 0), Point(w->w, 0), w->wallTextures[0]);
		if (w->wallTextures[1].size() > 0)
			drawTexturedLine(Point(w->w, 0), Point(w->w, w->h), w->wallTextures[1]);
		if (w->wallTextures[2].size() > 0)
			drawTexturedLine(Point(w->w, w->h), Point(0, w->h), w->wallTextures[2]);
		if (w->wallTextures[3].size() > 0)
			drawTexturedLine(Point(0, w->h), Point(0, 0), w->wallTextures[3]);*/
	}
	
	void CircularCam::finalize(double dt, World* w)
	{
		if (useFog)
		{
			for (size_t i = 0; i < image.size(); i++)
			{
				image[i] *= 1 / (1 + fogDensity * sqrt(zbuffer[i]));
				image[i].threshold(lightThreshold);
			}
		}
	}
	
	void CircularCam::setRange(double range)
	{
		this->r = range;
		owner->sortLocalInteractions();
	}
	
	
	
	OmniCam::OmniCam(Robot *owner, double height, unsigned halfPixelCount) :
		zbuffer(halfPixelCount * 2),
		image(halfPixelCount * 2),
		cam0(owner, Point(0, 0), height, -M_PI/2, M_PI/2, halfPixelCount),
		cam1(owner, Point(0, 0), height, M_PI/2, M_PI/2, halfPixelCount)
	{
		this->r = std::numeric_limits<double>::max();
		this->owner = owner;
	}

	void OmniCam::objectStep(double dt, World *w, PhysicalObject *po) 
	{
		cam0.objectStep(dt, w, po);
		cam1.objectStep(dt, w, po);
	};

	void OmniCam::init(double dt, World* w)
	{
		cam0.init(dt, w);
		cam1.init(dt, w);
	}
	
	void OmniCam::wallsStep(double dt, World* w)
	{
		cam0.wallsStep(dt, w);
		cam1.wallsStep(dt, w);
	}
	
	void OmniCam::finalize(double dt, World* w)
	{
		cam0.finalize(dt, w);
		cam1.finalize(dt, w);
		size_t camPixelCount = cam0.zbuffer.size();
		std::copy(&cam0.zbuffer[0], &cam0.zbuffer[camPixelCount], &zbuffer[0]);
		std::copy(&cam1.zbuffer[0], &cam1.zbuffer[camPixelCount], &zbuffer[camPixelCount]);
		std::copy(&cam0.image[0], &cam0.image[camPixelCount], &image[0]);
		std::copy(&cam1.image[0], &cam1.image[camPixelCount], &image[camPixelCount]);
	}
	
	void OmniCam::setRange(double range)
	{
		this->r = range;
		owner->sortLocalInteractions();
	}
	
	void OmniCam::setFogConditions(bool useFog, double density, Color threshold)
	{
		cam0.useFog = useFog;
		cam0.fogDensity = density;
		cam0.lightThreshold = threshold;
		cam1.useFog = useFog;
		cam1.fogDensity = density;
		cam1.lightThreshold = threshold;
	}
	
	void OmniCam::setPixelOperationFunctor(PixelOperationFunctor *pixelOperationFunctor)
	{
		cam0.pixelOperation = pixelOperationFunctor;
		cam1.pixelOperation = pixelOperationFunctor;
	}
}


