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

#include "Thymio2Model.h"
#include "objects/Objects.h"

//! Asserts a dynamic cast.	Similar to the one in boost/cast.hpp
template<typename Derived, typename Base>
inline Derived polymorphic_downcast(Base base)
{
	Derived derived = dynamic_cast<Derived>(base);
	assert(derived);
	return derived;
}

namespace Enki
{
	Thymio2Model::Thymio2Model(ViewerWidget* v)
	{
		viewer = v;

		textures.resize(2);
		textures[0] = v->bindTexture(QPixmap(QString(":/textures/thymio-bottomLed-diffusionMap.png")), GL_TEXTURE_2D);
		textures[1] = v->bindTexture(QPixmap(QString(":/textures/thymio-wheel-texture.png")), GL_TEXTURE_2D);

		bodyTexture = QImage(QString(":/textures/thymio-body-texture.png"));
		bodyDiffusionMap0 = QImage(QString(":/textures/thymio-body-diffusionMap0.png"));
		bodyDiffusionMap1 = QImage(QString(":/textures/thymio-body-diffusionMap1.png"));
		bodyDiffusionMap2 = QImage(QString(":/textures/thymio-body-diffusionMap2.png"));

		lists.resize(2);
		lists[0] = GenThymio2Body();
		lists[1] = GenThymio2Wheel();

		textureDimension = bodyTexture.width();
		vec2f buttonCenter(0.136f,0.764f);
		for (unsigned int i=0; i<Thymio2::LED_COUNT; i++)
		{
			switch(i)
			{
				case Thymio2::TOP:     	   ledCenter[i].push_back(vec2f(0.5f,0.5f)*textureDimension); ledSize[i].push_back(vec2f(1.f,1.f)*textureDimension); break;
				case Thymio2::BOTTOM_LEFT:  ledCenter[i].push_back(vec2f(0.6074f,0.1841f)*textureDimension); ledSize[i].push_back(vec2f(0.1133f,0.2939f)*textureDimension);
						   ledCenter[i].push_back(vec2f(0.7309f,0.7837f)*textureDimension); ledSize[i].push_back(vec2f(0.1885f,0.1396f)*textureDimension);
						   break;
				case Thymio2::BOTTOM_RIGHT: ledCenter[i].push_back(vec2f(0.6636f,0.4297f)*textureDimension); ledSize[i].push_back(vec2f(0.2236f,0.1875f)*textureDimension); break;

				case Thymio2::BUTTON_UP:    ledCenter[i].push_back((buttonCenter + vec2f(-0.038f,0))*textureDimension); ledSize[i].push_back(vec2f(0.035f,0.045f)*textureDimension); break;
				case Thymio2::BUTTON_DOWN:  ledCenter[i].push_back((buttonCenter + vec2f( 0.038f,0))*textureDimension); ledSize[i].push_back(vec2f(0.035f,0.045f)*textureDimension); break;
				case Thymio2::BUTTON_LEFT:  ledCenter[i].push_back((buttonCenter + vec2f(0, 0.038f))*textureDimension); ledSize[i].push_back(vec2f(0.045f,0.035f)*textureDimension); break;
				case Thymio2::BUTTON_RIGHT: ledCenter[i].push_back((buttonCenter + vec2f(0,-0.038f))*textureDimension); ledSize[i].push_back(vec2f(0.045f,0.035f)*textureDimension); break;

				case Thymio2::RING_0:       ledCenter[i].push_back((buttonCenter + vec2f(-0.105f,0))*textureDimension);      ledSize[i].push_back(vec2f(0.04f,0.08f)*textureDimension); break;
				case Thymio2::RING_1:       ledCenter[i].push_back((buttonCenter + vec2f(-0.0703f, 0.0703f))*textureDimension); ledSize[i].push_back(vec2f(0.065f,0.065f)*textureDimension); break;
				case Thymio2::RING_2:       ledCenter[i].push_back((buttonCenter + vec2f( 0, 0.105f))*textureDimension);     ledSize[i].push_back(vec2f(0.08f,0.04f)*textureDimension); break;
				case Thymio2::RING_3:       ledCenter[i].push_back((buttonCenter + vec2f( 0.0703f, 0.0703f))*textureDimension); ledSize[i].push_back(vec2f(0.065f,0.065f)*textureDimension); break;
				case Thymio2::RING_4:       ledCenter[i].push_back((buttonCenter + vec2f( 0.105f,0))*textureDimension);      ledSize[i].push_back(vec2f(0.04f,0.08f)*textureDimension); break;
				case Thymio2::RING_5:       ledCenter[i].push_back((buttonCenter + vec2f( 0.0703f,-0.0703f))*textureDimension); ledSize[i].push_back(vec2f(0.065f,0.065f)*textureDimension); break;
				case Thymio2::RING_6:       ledCenter[i].push_back((buttonCenter + vec2f( 0,-0.105f))*textureDimension);     ledSize[i].push_back(vec2f(0.08f,0.04f)*textureDimension); break;
				case Thymio2::RING_7:       ledCenter[i].push_back((buttonCenter + vec2f(-0.0703f,-0.0703f))*textureDimension); ledSize[i].push_back(vec2f(0.065f,0.065f)*textureDimension); break;

				case Thymio2::IR_FRONT_0:   ledCenter[i].push_back(vec2f(0.5586f,0.0459f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case Thymio2::IR_FRONT_1:   ledCenter[i].push_back(vec2f(0.5644f,0.1279f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case Thymio2::IR_FRONT_2:   ledCenter[i].push_back(vec2f(0.5673f,0.2441f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case Thymio2::IR_FRONT_3:   ledCenter[i].push_back(vec2f(0.5693f,0.3056f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case Thymio2::IR_FRONT_4:   ledCenter[i].push_back(vec2f(0.5664f,0.4258f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case Thymio2::IR_FRONT_5:   ledCenter[i].push_back(vec2f(0.5615f,0.5185f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case Thymio2::IR_BACK_0:    ledCenter[i].push_back(vec2f(0.8759f,0.6289f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;
				case Thymio2::IR_BACK_1:    ledCenter[i].push_back(vec2f(0.5449f,0.6289f)*textureDimension); ledSize[i].push_back(vec2f(0.06f,0.06f)*textureDimension); break;

				case Thymio2::LEFT_BLUE:    ledCenter[i].push_back(vec2f(0.7163f,0.8428f)*textureDimension); ledSize[i].push_back(vec2f(0.0771f,0.0878f)*textureDimension); break;
				case Thymio2::LEFT_RED:     ledCenter[i].push_back(vec2f(0.7163f,0.8428f)*textureDimension); ledSize[i].push_back(vec2f(0.0771f,0.0878f)*textureDimension); break;
				case Thymio2::RIGHT_BLUE:   ledCenter[i].push_back(vec2f(0.7974f,0.3750f)*textureDimension); ledSize[i].push_back(vec2f(0.0910f,0.0910f)*textureDimension); break;
				case Thymio2::RIGHT_RED:    ledCenter[i].push_back(vec2f(0.7773f,0.4336f)*textureDimension); ledSize[i].push_back(vec2f(0.0400f,0.0400f)*textureDimension); break;
				default: break;
			}

			// shrink vector
			std::vector<vec2i>(ledCenter[i]).swap(ledCenter[i]);
			std::vector<vec2i>(ledSize[i]).swap(ledSize[i]);
		}
	}

	void Thymio2Model::cleanup(ViewerWidget* viewer)
	{
		for (int i = 0; i < textures.size(); i++)
			viewer->deleteTexture(textures[i]);
		for (int i = 0; i < lists.size(); i++)
			glDeleteLists(lists[i], 1);
	}

	void Thymio2Model::draw(PhysicalObject* object) const
	{
		Thymio2* thymio = polymorphic_downcast<Thymio2*>(object);
		if(thymio->ledTextureNeedUpdate)
		{
			viewer->deleteTexture(thymio->textureID);
			thymio->ledTextureNeedUpdate = false;
			thymio->textureID = updateLedTexture(thymio);
		}

		const double wheelRadius = 2.1;
		const double wheelCirc = 2 * M_PI * wheelRadius;

		// body
		glDisable(GL_LIGHTING);
		glColor3d(object->getColor().components[0], object->getColor().components[1], object->getColor().components[2]);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, thymio->textureID);
		glPushMatrix();
		glTranslatef(0.3,0,0);
		glCallList(lists[0]);

		// bottom lighting
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		if(thymio->getColorInt(Thymio2::BOTTOM_LEFT) & 0xFF000000)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDepthMask( GL_FALSE );
			glEnable(GL_POLYGON_OFFSET_FILL);

			Color color = unpack(thymio->getColorInt(Thymio2::BOTTOM_LEFT));
			glColor4d(color.r(),color.g(),color.b(),color.a());

			glBegin (GL_QUADS);
				glNormal3f (0,0,1);
				glTexCoord2f(0.01f,0.01f); glVertex3f(0, 0,0);
				glTexCoord2f(0.01f,0.99f); glVertex3f(8, 0,0);
				glTexCoord2f(0.99f,0.99f); glVertex3f(8, 8,0);
				glTexCoord2f(0.99f,0.01f); glVertex3f(0, 8,0);
			glEnd();

			glDisable(GL_POLYGON_OFFSET_FILL);
			glDepthMask( GL_TRUE );
			glDisable(GL_BLEND);
		}
		if(thymio->getColorInt(Thymio2::BOTTOM_RIGHT) & 0xFF000000)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDepthMask( GL_FALSE );
			glEnable(GL_POLYGON_OFFSET_FILL);

			Color color = unpack(thymio->getColorInt(Thymio2::BOTTOM_RIGHT));
			glColor4d(color.r(),color.g(),color.b(),color.a());

			glBegin (GL_QUADS);
				glNormal3f (0,0,1);
				glTexCoord2f(0.99f,0.01f); glVertex3f(0,-8,0);
				glTexCoord2f(0.99f,0.99f); glVertex3f(8,-8,0);
				glTexCoord2f(0.01f,0.99f); glVertex3f(8, 0,0);
				glTexCoord2f(0.01f,0.01f); glVertex3f(0, 0,0);
			glEnd();

			glDisable(GL_POLYGON_OFFSET_FILL);
			glDepthMask( GL_TRUE );
			glDisable(GL_BLEND);
		}
		glPopMatrix();

		// wheels
		glColor3d(object->getColor().components[0], object->getColor().components[1], object->getColor().components[2]);
		glBindTexture(GL_TEXTURE_2D, textures[1]);

		glPushMatrix();
		glTranslatef(-2.3,0,wheelRadius);
		glRotated(180.f, 0, 0, 1);
			glPushMatrix();
			glTranslatef(0,4,0);
			glRotated(-(fmod(thymio->rightOdometry, wheelCirc) * 360) / wheelCirc, 0, 1, 0);
			glCallList(lists[1]);
			glPopMatrix();

			glPushMatrix();
			glTranslatef(0,-4,0);
			glRotated(180.f, 0, 0, 1);
			glRotated(-(fmod(-thymio->leftOdometry, wheelCirc) * 360) / wheelCirc, 0, 1, 0);
			glCallList(lists[1]);
			glPopMatrix();
		glPopMatrix();

		// end
		glDisable(GL_LIGHTING);
	}

	unsigned int Thymio2Model::updateLedTexture(Thymio2* thymio) const
	{
		if(!thymio->ledTexture)
		{
			thymio->ledTexture = new uint32_t[textureDimension*textureDimension];
			for (unsigned int i=0; i<textureDimension; i++)
				for (unsigned int j=0; j<textureDimension; j++)
					thymio->ledTexture[i*textureDimension+j] = pack(255,255,255,0);
		}

		uint32_t* tex = thymio->ledTexture;
		uint32_t* bodyTex   = (uint32_t*)bodyTexture.bits();
		uint32_t* bodyDiff0 = (uint32_t*)bodyDiffusionMap0.bits();
		uint32_t* bodyDiff1 = (uint32_t*)bodyDiffusionMap1.bits();
		uint32_t* bodyDiff2 = (uint32_t*)bodyDiffusionMap2.bits();

		for (unsigned int i=0; i<textureDimension; i++)
			for (unsigned int j=0; j<textureDimension; j++)
			{
				if(bodyTex) tex[i+textureDimension*j] = bodyTex[i+textureDimension*j];
				else tex[i+textureDimension*j] = 0xFFFFFFFF;
			}

		// color led area
		for (unsigned int i=0; i<Thymio2::LED_COUNT; i++)
		{
			for (unsigned j=0;j<ledCenter[i].size();j++)
			{
				uint32_t ledColor = thymio->getColorInt((Thymio2::LED_INDEX)i);
				switch(i)
				{
					case Thymio2::TOP:
						drawRect(tex, bodyTex, ledCenter[i][j], ledSize[i][j], ledColor, bodyDiff0);
						break;
					case Thymio2::BOTTOM_LEFT: case Thymio2::BOTTOM_RIGHT:
						drawRect(tex, bodyTex, ledCenter[i][j], ledSize[i][j], ledColor, bodyDiff1);
						break;
					default:
						drawRect(tex, bodyTex, ledCenter[i][j], ledSize[i][j], ledColor, bodyDiff2);
						break;
				}
			}
		}

		return viewer->bindTexture(QImage((uint8_t*)(thymio->ledTexture),textureDimension,textureDimension,QImage::Format_ARGB32), GL_TEXTURE_2D);
	}

	void Thymio2Model::drawRect(uint32_t* target, uint32_t* base, vec2i center, vec2i size, uint32_t color, uint32_t* diffTex) const
	{
		for (int i=center.x-size.x/2; i<center.x+size.x/2; i++)
			for (int j=center.y-size.y/2; j<center.y+size.y/2; j++)
			{
				if(i<0 || j<0 || i>=textureDimension || j>=textureDimension) continue;

				// get destination color (previous color)
				Color destination = Color::fromARGB(target[i+textureDimension*j]);
				
				// compute source color (color to add)
				Color source = Color::fromARGB(color);
				if (diffTex)
				{
					Color diff = unpack(diffTex[i+textureDimension*j]);
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
				target[i+textureDimension*j] = pack(r,g,b,255);
			}
	}

	Color Thymio2Model::unpack(uint32_t colorInt) const
	{
		return Color::fromARGB(colorInt);
	}

	uint32_t Thymio2Model::pack(unsigned char r,unsigned char g,unsigned char b,unsigned char a) const
	{
		// pack color into #AARRGGBB format (aka unsigned int);
		return ((a<<24)|(r<<16)|(g<<8)|(b<<0));
	}

	uint32_t Thymio2Model::pack(Color c) const
	{
		unsigned char r = (255*c.r());
		unsigned char g = (255*c.g());
		unsigned char b = (255*c.b());
		unsigned char a = (255*c.a());
		return pack(r,g,b,a);
	}

} // namespace Enki


