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

#ifndef __ENKI_VIEWER_THYMIO2_MODEL_H
#define __ENKI_VIEWER_THYMIO2_MODEL_H

#include "Viewer.h"
#include <enki/robots/thymio2/Thymio2.h>

namespace Enki
{
	class Thymio2Model : public ViewerWidget::CustomRobotModel
	{
	public:
		Thymio2Model(ViewerWidget* viewer);
		virtual void cleanup(ViewerWidget* viewer);
		virtual void draw(PhysicalObject* object) const;

		unsigned int textureDimension;
		QImage bodyDiffusionMap0,bodyDiffusionMap1,bodyDiffusionMap2,bodyTexture;

	protected:
		template<typename T> class vec2{
		public:
			vec2(T a,T b):x(a),y(b){};
			T x;
			T y;

			template<typename T2> vec2 operator* (T2 f) { return vec2<T>(x*f,y*f); }
			vec2 operator+ (vec2 v) { return vec2(x+v.x,y+v.y); }
			template<typename T2> operator vec2<T2>() { return vec2<T2>(T2(x),T2(y)); }
		};
		typedef vec2<int> vec2i;
		typedef vec2<float> vec2f;

		std::vector<vec2i> ledCenter[Thymio2::LED_COUNT];
		std::vector<vec2i> ledSize[Thymio2::LED_COUNT];

		ViewerWidget* viewer;

		uint32_t pack(unsigned char r,unsigned char g,unsigned char b,unsigned char a = 255) const;
		uint32_t pack(Color c) const;
		Color unpack(uint32_t colorInt) const;
		unsigned int updateLedTexture(Thymio2* thymio) const;
		void drawRect(uint32_t* target, uint32_t* base, vec2i center, vec2i size, uint32_t color, uint32_t* diffTex) const;
	};
} // namespace Enki

#endif // __ENKI_VIEWER_THYMIO2_MODEL_H
