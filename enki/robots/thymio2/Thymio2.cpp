/*
	Enki - a fast 2D robot simulator
	Copyright (C) 1999-2008 Stephane Magnenat <stephane at magnenat dot net>
	Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
	Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
	Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
	Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
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

#include "Thymio2.h"
#include <algorithm>
#include <functional>
#include <climits>
#include <math.h>

/*! \file Thymio2.cpp
	\brief Implementation of the Thymio II robot
*/
namespace Enki
{
	using namespace std;

	unsigned int Thymio2::textureDimension = 512;

	Thymio2::vec2f Thymio2::vec2i::operator* (float f)
	{
		return vec2f(x*f,y*f);
	}
	Thymio2::vec2f Thymio2::vec2f::operator* (float f)
	{
		return vec2f(x*f,y*f);
	}
	Thymio2::vec2f Thymio2::vec2f::operator+ (vec2f v)
	{
		return vec2f(x+v.x,y+v.y);
	}
	
	Thymio2::Thymio2() :
		DifferentialWheeled(9.4, 16.6, 0.027),
		infraredSensor0(this, Vector(6.2, 4.85),   3.4, 0.69813,  14, 4505, 0.03, 73, 2.87),
		infraredSensor1(this, Vector(7.5, 2.55),   3.4, 0.34906,  14, 4505, 0.03, 73, 2.87),
		infraredSensor2(this, Vector(7.95, 0.0),   3.4, 0.0,      14, 4505, 0.03, 73, 2.87),
		infraredSensor3(this, Vector(7.5, -2.55),  3.4, -0.34906, 14, 4505, 0.03, 73, 2.87),
		infraredSensor4(this, Vector(6.2, -4.85),  3.4, -0.69813, 14, 4505, 0.03, 73, 2.87),
		infraredSensor5(this, Vector(-2.95, 2.95), 3.4, -M_PI,    14, 4505, 0.03, 73, 2.87),
		infraredSensor6(this, Vector(-2.95, -2.95),3.4, -M_PI,    14, 4505, 0.03, 73, 2.87),
		groundSensor0(this, Vector(7.2, 1.2),  806.93, 45, 0.3, 10),
		groundSensor1(this, Vector(7.2, -1.2), 806.93, 45, 0.3, 10)
	{
		// add interactions
		addLocalInteraction(&infraredSensor0);
		addLocalInteraction(&infraredSensor1);
		addLocalInteraction(&infraredSensor2);
		addLocalInteraction(&infraredSensor3);
		addLocalInteraction(&infraredSensor4);
		addLocalInteraction(&infraredSensor5);
		addLocalInteraction(&infraredSensor6);
		addLocalInteraction(&groundSensor0);
		addLocalInteraction(&groundSensor1);
		
		//staticFrictionThreshold = 0.5;
		dryFrictionCoefficient = 0.25;
		dryFrictionCoefficient = 2.5;
		
		// define the physical shape of the Thymio
		Enki::Polygone thymio2Shape;
		const double amount = 10.0;
		const double radius = 8.0;
		const double height = 5.1;
		const double angle1 = asin(5.5/8.0);
		const double angle2 = atan(5.5/3.0);
		const double distance = sqrt(3.0*3.0+5.5*5.5);
		for (double a = -angle1; a < angle1+0.01; a += 2*angle1/amount)
			thymio2Shape.push_back(Enki::Point(radius * cos(a), radius * sin(a)));        
		thymio2Shape.push_back(Enki::Point(distance * cos(M_PI - angle2), distance * sin(M_PI - angle2)));
		thymio2Shape.push_back(Enki::Point(distance * cos(M_PI - angle2), distance * sin(M_PI + angle2)));
		Enki::PhysicalObject::Hull hull(Enki::PhysicalObject::Part(thymio2Shape, height));
		setCustomHull(hull, 200);
		setColor(Color(0.7, 0.7, 0.7));

		textureID = 0;
		ledTexture = new uint32_t[textureDimension*textureDimension];
		for (unsigned int i=0; i<textureDimension; i++)
			for (unsigned int j=0; j<textureDimension; j++)
				ledTexture[i*textureDimension+j] = pack(255,255,255,0);
		ledTextureNeedUpdate = true;

		vec2f buttonCenter(0.135f,0.763f);

		for (unsigned int i=0; i<LED_COUNT; i++)
		{
			switch(i)
			{
				case TOP_LEFT:     ledColor[i] = pack(255,255,0,255);
						   ledCenter[i].push_back(vec2f(0.3418f,0.6230f)*textureDimension); ledSize[i].push_back(vec2f(0.2441f,0.1953f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.1709f,0.5840f)*textureDimension); ledSize[i].push_back(vec2f(0.0977f,0.1172f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.5859f,0.6543f)*textureDimension); ledSize[i].push_back(vec2f(0.2344f,0.1172f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.7949f,0.2725f)*textureDimension); ledSize[i].push_back(vec2f(0.0977f,0.2344f)*textureDimension);
						   break;
				case TOP_RIGHT:    ledColor[i] = pack(255,255,0,255);
						   ledCenter[i].push_back(vec2f(0.3418f,0.8984f)*textureDimension); ledSize[i].push_back(vec2f(0.2441f,0.1953f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.1709f,0.9375f)*textureDimension); ledSize[i].push_back(vec2f(0.0977f,0.1172f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.8320f,0.6543f)*textureDimension); ledSize[i].push_back(vec2f(0.2344f,0.1172f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.5859f,0.8359f)*textureDimension); ledSize[i].push_back(vec2f(0.2344f,0.0977f)*textureDimension);
						   break;
				case BOTTOM_LEFT:  ledColor[i] = pack(255,128,0,255);
						   ledCenter[i].push_back(vec2f(0.7227f,0.4648f)*textureDimension); ledSize[i].push_back(vec2f(0.1133f,0.1133f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.6074f,0.4082f)*textureDimension); ledSize[i].push_back(vec2f(0.1133f,0.2295f)*textureDimension);
						   break;
				case BOTTOM_RIGHT: ledColor[i] = pack(0,128,255,255);
						   ledCenter[i].push_back(vec2f(0.7695f,0.7715f)*textureDimension); ledSize[i].push_back(vec2f(0.1133f,0.1133f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.6074f,0.1563f)*textureDimension); ledSize[i].push_back(vec2f(0.1133f,0.2295f)*textureDimension);
						   break;

				case BUTTON_UP:    ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back((buttonCenter + vec2f(-0.0365f,0))*textureDimension); ledSize[i].push_back(vec2f(0.02f,0.04f)*textureDimension); break;
				case BUTTON_DOWN:  ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back((buttonCenter + vec2f( 0.0365f,0))*textureDimension); ledSize[i].push_back(vec2f(0.02f,0.04f)*textureDimension); break;
				case BUTTON_LEFT:  ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back((buttonCenter + vec2f(0, 0.0365f))*textureDimension); ledSize[i].push_back(vec2f(0.04f,0.02f)*textureDimension); break;
				case BUTTON_RIGHT: ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back((buttonCenter + vec2f(0,-0.0365f))*textureDimension); ledSize[i].push_back(vec2f(0.04f,0.02f)*textureDimension); break;

				case RING_0:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f(-0.1f,0))*textureDimension);      ledSize[i].push_back(vec2f(0.04f,0.08f)*textureDimension); break;
				case RING_1:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f(-0.07f, 0.07))*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case RING_2:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f( 0, 0.1f))*textureDimension);     ledSize[i].push_back(vec2f(0.08f,0.04f)*textureDimension); break;
				case RING_3:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f( 0.07f, 0.07))*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case RING_4:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f( 0.1f,0))*textureDimension);      ledSize[i].push_back(vec2f(0.04f,0.08f)*textureDimension); break;
				case RING_5:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f( 0.07f,-0.07))*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case RING_6:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f( 0,-0.1f))*textureDimension);     ledSize[i].push_back(vec2f(0.08f,0.04f)*textureDimension); break;
				case RING_7:       ledColor[i] = pack(255,128,000,255); ledCenter[i].push_back((buttonCenter + vec2f(-0.07f,-0.07))*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;

				case IR_FRONT_0:   ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.5518f,0.0879f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;
				case IR_FRONT_1:   ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.5518f,0.1592f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;
				case IR_FRONT_2:   ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.5518f,0.2441f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;
				case IR_FRONT_3:   ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.5518f,0.3203f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;
				case IR_FRONT_4:   ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.5518f,0.4062f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;
				case IR_FRONT_5:   ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.5518f,0.4746f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;
				case IR_BACK_0:    ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.8672f,0.6543f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;
				case IR_BACK_1:    ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.5547f,0.6543f)*textureDimension); ledSize[i].push_back(vec2f(0.0195f,0.0195f)*textureDimension); break;

				case LEFT_BLUE:    ledColor[i] = pack(000,000,255,255); ledCenter[i].push_back(vec2f(0.7773f,0.4043f)*textureDimension); ledSize[i].push_back(vec2f(0.0391f,0.0391f)*textureDimension); break;
				case LEFT_RED:     ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.7773f,0.4727f)*textureDimension); ledSize[i].push_back(vec2f(0.0391f,0.0391f)*textureDimension); break;
				case RIGHT_BLUE:   ledColor[i] = pack(000,000,255,255); ledCenter[i].push_back(vec2f(0.7070f,0.8242f)*textureDimension); ledSize[i].push_back(vec2f(0.0391f,0.0391f)*textureDimension); break;
				case RIGHT_RED:    ledColor[i] = pack(255,000,000,255); ledCenter[i].push_back(vec2f(0.7715f,0.8242f)*textureDimension); ledSize[i].push_back(vec2f(0.0391f,0.0391f)*textureDimension); break;
				default: break;
			}

			// shrink vector
			std::vector<vec2i>(ledCenter[i]).swap(ledCenter[i]);
			std::vector<vec2i>(ledSize[i]).swap(ledSize[i]);
		}
	}
	
	Thymio2::~Thymio2()
	{
		delete[] ledTexture;
	}

	void Thymio2::setLedIntensity(LED_INDEX ledIndex, float intensity)
	{
		setLedIntensity(ledIndex,(unsigned char)(255*intensity));
	}

	void Thymio2::setLedIntensity(LED_INDEX ledIndex, unsigned char intensity)
	{
		if(ledIndex<0 || ledIndex>=LED_COUNT) return;
		uint32_t c = (intensity<<24);
		if(c == (ledColor[ledIndex]&0xFF000000)) return;
		ledColor[ledIndex] = (ledColor[ledIndex] & 0x00FFFFFF)|c;
		ledTextureNeedUpdate = true;
	}

	void Thymio2::setLedColor(LED_INDEX ledIndex, Color color)
	{
		setLedColor(ledIndex,(unsigned char)(255*color.r()),(unsigned char)(255*color.g()),(unsigned char)(255*color.b()),(unsigned char)(255*color.a()));
	}

	void Thymio2::setLedColor(LED_INDEX ledIndex, unsigned char r, unsigned char g, unsigned char b, unsigned char a)
	{
		if (ledIndex<0 || ledIndex>=LED_COUNT) return;
		uint32_t c;
		switch (ledIndex)
		{
			case TOP_LEFT: case TOP_RIGHT: case BOTTOM_LEFT: case BOTTOM_RIGHT:
				c = pack(r,g,b,a);
				if(c == ledColor[ledIndex]) return;
				ledColor[ledIndex] = c;
				ledTextureNeedUpdate = true;
				break;
			default:
				setLedIntensity(ledIndex,a);
				break;
		}
	}

	unsigned int Thymio2::getColorInt(LED_INDEX ledIndex)
	{
		if (ledIndex<0 || ledIndex>=LED_COUNT) return 0;
		else return ledColor[ledIndex];
	}

	Color Thymio2::getColor(LED_INDEX ledIndex)
	{
		if(ledIndex<0 || ledIndex>=LED_COUNT) return 0;
		else return unpack(ledColor[ledIndex]);
	}

	bool Thymio2::updateLedTexture(uint32_t* base,uint32_t* diffusionMap)
	{
		if (!ledTextureNeedUpdate) return false;
		ledTextureNeedUpdate = false;

		// reset texture
		for (unsigned int i=0; i<textureDimension; i++)
			for (unsigned int j=0; j<textureDimension; j++)
			{
				if(base) ledTexture[i+textureDimension*j] = base[i+textureDimension*j];
				else ledTexture[i+textureDimension*j] = 0xFFFFFFFF;
			}

		// color led area
		for (unsigned int i=0; i<LED_COUNT; i++)
		{
			for (unsigned j=0;j<ledCenter[i].size();j++)
			{
				switch(i)
				{
					case TOP_LEFT: case TOP_RIGHT: case BOTTOM_LEFT: case BOTTOM_RIGHT:
					case RING_0: case RING_1: case RING_2: case RING_3: case RING_4: case RING_5: case RING_6: case RING_7: 
						drawRect(base,ledCenter[i][j],ledSize[i][j],ledColor[i],diffusionMap);
						break;
					default:
						drawRect(base,ledCenter[i][j],ledSize[i][j],ledColor[i],0);
						break;
				}
			}
		}
		return true;
	}

	Color Thymio2::unpack(uint32_t colorInt)
	{
		return Color::fromARGB(colorInt);
	}

	uint32_t Thymio2::pack(unsigned char r,unsigned char g,unsigned char b,unsigned char a)
	{
		// pack color into #AARRGGBB format (aka unsigned int);
		return ((a<<24)|(r<<16)|(g<<8)|(b<<0));
	}

	void Thymio2::drawRect(uint32_t* base,vec2i center,vec2i size,uint32_t color,uint32_t* diffusionMap)
	{
		for (int i=center.x-size.x/2; i<center.x+size.x/2; i++)
			for (int j=center.y-size.y/2; j<center.y+size.y/2; j++)
			{
				if(i<0 || j<0 || i>=textureDimension || j>=textureDimension) continue;

				// get destination color (previous color)
				Color destination = Color::fromARGB(ledTexture[i+textureDimension*j]);
				
				// compute source color (color to add)
				Color source = Color::fromARGB(color);
				if (diffusionMap)
				{
					Color diff = unpack(diffusionMap[i+textureDimension*j]);
					Color c = Color::fromARGB(color);
					source = Color(c.r()*diff.r(), c.g()*diff.g(), c.b()*diff.b(), c.a()*diff.a());
				}
				else
				{
					double x = ((double)i-center.x)/(size.x/2.);
					double y = ((double)j-center.y)/(size.y/2.);
					source.setA(source.a()*std::max(std::min(1.-std::sqrt(x*x+y*y),1.),0.));
				}
				
				// blend collor
				Color c = destination*(1. - source.a()) + source*source.a();
				unsigned char r = (255*c.r());
				unsigned char g = (255*c.g());
				unsigned char b = (255*c.b());
				ledTexture[i+textureDimension*j] = pack(r,g,b,255);
			}
	}
}

