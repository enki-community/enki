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

		textures.resize(3);
		textures[0] = v->bindTexture(QPixmap(QString(":/textures/thymio-bottomLed-diffusionMap.png")), GL_TEXTURE_2D, GL_LUMINANCE8);
		textures[1] = v->bindTexture(QPixmap(QString(":/textures/thymio-wheel-texture.png")), GL_TEXTURE_2D);
		textures[2] = v->bindTexture(QPixmap(QString(":/textures/thymio-ground-shadow.png")), GL_TEXTURE_2D, GL_LUMINANCE8);

		bodyTexture = QImage(QString(":/textures/thymio-body-texture.png"));
		bodyDiffusionMap0 = QImage(QString(":/textures/thymio-body-diffusionMap0.png"));
		bodyDiffusionMap1 = QImage(QString(":/textures/thymio-body-diffusionMap1.png"));
		bodyDiffusionMap2 = QImage(QString(":/textures/thymio-body-diffusionMap2.png"));

		lists.resize(2);
		lists[0] = GenThymio2Body();
		lists[1] = GenThymio2Wheel();

		textureDimension = bodyTexture.width();
		Vector buttonCenter(0.136f,0.764f);
		for (unsigned i=0; i<Thymio2::LED_COUNT; i++)
		{
			switch(i)
			{
				case Thymio2::TOP:     	    ledCenter[i].push_back(Vector(0.5f,0.5f));       ledSize[i].push_back(Vector(1.f,1.f)); break;
				case Thymio2::BOTTOM_LEFT:  ledCenter[i].push_back(Vector(0.6074f,0.1841f)); ledSize[i].push_back(Vector(0.1133f,0.2939f));
											ledCenter[i].push_back(Vector(0.7309f,0.7837f)); ledSize[i].push_back(Vector(0.1885f,0.1396f)); break;
				case Thymio2::BOTTOM_RIGHT: ledCenter[i].push_back(Vector(0.6636f,0.4297f)); ledSize[i].push_back(Vector(0.2236f,0.1875f)); break;

				case Thymio2::BUTTON_UP:    ledCenter[i].push_back((buttonCenter + Vector(-0.038f,0))); ledSize[i].push_back(Vector(0.035f,0.045f)); break;
				case Thymio2::BUTTON_DOWN:  ledCenter[i].push_back((buttonCenter + Vector( 0.038f,0))); ledSize[i].push_back(Vector(0.035f,0.045f)); break;
				case Thymio2::BUTTON_LEFT:  ledCenter[i].push_back((buttonCenter + Vector(0, 0.038f))); ledSize[i].push_back(Vector(0.045f,0.035f)); break;
				case Thymio2::BUTTON_RIGHT: ledCenter[i].push_back((buttonCenter + Vector(0,-0.038f))); ledSize[i].push_back(Vector(0.045f,0.035f)); break;

				case Thymio2::RING_0:       ledCenter[i].push_back((buttonCenter + Vector(-0.105f,0)));         ledSize[i].push_back(Vector(0.04f,0.08f)); break;
				case Thymio2::RING_1:       ledCenter[i].push_back((buttonCenter + Vector(-0.0703f,-0.0703f))); ledSize[i].push_back(Vector(0.065f,0.065f)); break;
				case Thymio2::RING_2:       ledCenter[i].push_back((buttonCenter + Vector( 0,-0.105f)));        ledSize[i].push_back(Vector(0.08f,0.04f)); break;
				case Thymio2::RING_3:       ledCenter[i].push_back((buttonCenter + Vector( 0.0703f,-0.0703f))); ledSize[i].push_back(Vector(0.065f,0.065f)); break;
				case Thymio2::RING_4:       ledCenter[i].push_back((buttonCenter + Vector( 0.105f,0)));         ledSize[i].push_back(Vector(0.04f,0.08f)); break;
				case Thymio2::RING_5:       ledCenter[i].push_back((buttonCenter + Vector( 0.0703f, 0.0703f))); ledSize[i].push_back(Vector(0.065f,0.065f)); break;
				case Thymio2::RING_6:       ledCenter[i].push_back((buttonCenter + Vector( 0, 0.105f)));        ledSize[i].push_back(Vector(0.08f,0.04f)); break;
				case Thymio2::RING_7:       ledCenter[i].push_back((buttonCenter + Vector(-0.0703f, 0.0703f))); ledSize[i].push_back(Vector(0.065f,0.065f)); break;

				case Thymio2::IR_FRONT_0:   ledCenter[i].push_back(Vector(0.5586f,0.0459f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;
				case Thymio2::IR_FRONT_1:   ledCenter[i].push_back(Vector(0.5644f,0.1279f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;
				case Thymio2::IR_FRONT_2:   ledCenter[i].push_back(Vector(0.5673f,0.2441f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;
				case Thymio2::IR_FRONT_3:   ledCenter[i].push_back(Vector(0.5693f,0.3056f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;
				case Thymio2::IR_FRONT_4:   ledCenter[i].push_back(Vector(0.5664f,0.4258f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;
				case Thymio2::IR_FRONT_5:   ledCenter[i].push_back(Vector(0.5615f,0.5185f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;
				case Thymio2::IR_BACK_0:    ledCenter[i].push_back(Vector(0.8759f,0.6289f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;
				case Thymio2::IR_BACK_1:    ledCenter[i].push_back(Vector(0.5449f,0.6289f)); ledSize[i].push_back(Vector(0.06f,0.06f)); break;

				case Thymio2::LEFT_BLUE:    ledCenter[i].push_back(Vector(0.7163f,0.8428f)); ledSize[i].push_back(Vector(0.0771f,0.0878f)); break;
				case Thymio2::LEFT_RED:     ledCenter[i].push_back(Vector(0.7163f,0.8428f)); ledSize[i].push_back(Vector(0.0771f,0.0878f)); break;
				case Thymio2::RIGHT_BLUE:   ledCenter[i].push_back(Vector(0.7974f,0.3750f)); ledSize[i].push_back(Vector(0.0910f,0.0910f)); break;
				case Thymio2::RIGHT_RED:    ledCenter[i].push_back(Vector(0.7773f,0.4336f)); ledSize[i].push_back(Vector(0.0400f,0.0400f)); break;
				default: break;
			}

			// shrink vector
			std::vector<Vector>(ledCenter[i]).swap(ledCenter[i]);
			std::vector<Vector>(ledSize[i]).swap(ledSize[i]);
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
		if (thymio->ledTextureNeedUpdate)
		{
			viewer->deleteTexture(thymio->textureID);
			thymio->ledTextureNeedUpdate = false;
			thymio->textureID = updateLedTexture(thymio);
		}

		const double wheelRadius = 2.1;
		const double wheelCirc = 2 * M_PI * wheelRadius;

		// body
		glDisable(GL_LIGHTING);
		glColor3d(1, 1, 1);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, thymio->textureID);
		
		glPushMatrix();
		glTranslatef(2.5,0,0);
		glCallList(lists[0]);
		glPopMatrix();

		// wheels
		glBindTexture(GL_TEXTURE_2D, textures[1]);

		glPushMatrix();
		glTranslatef(0,0,wheelRadius);
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
		
		// shadow
		glBindTexture(GL_TEXTURE_2D, textures[2]);
		glDisable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_ZERO, GL_SRC_COLOR);
		
		// bottom shadow
		glPushMatrix();
		// disable writing of z-buffer
		glDepthMask( GL_FALSE );
		glEnable(GL_POLYGON_OFFSET_FILL);
		glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 0.0f);
		glVertex2f(-10.f, -10.f);
		glTexCoord2f(1.0f, 1.0f);
		glVertex2f(10.f, -10.f);
		glTexCoord2f(0.0f, 1.0f);
		glVertex2f(10.f, 10.f);
		glTexCoord2f(0.0f, 0.0f);
		glVertex2f(-10.f, 10.f);
		glEnd();
		
		glPopMatrix();
		
		// bottom lighting
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		glBlendFunc(GL_SRC_COLOR, GL_ONE);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		if (thymio->getColorLed(Thymio2::BOTTOM_LEFT).a() != 0.0)
		{
			const Color color = thymio->getColorLed(Thymio2::BOTTOM_LEFT) * 0.6;
			glColor4d(color.r(),color.g(),color.b(),color.a());

			glBegin (GL_QUADS);
				glNormal3f (0,0,1);
				glTexCoord2f(0.01f,0.01f); glVertex3f(-2.5, -2,0);
				glTexCoord2f(0.01f,0.99f); glVertex3f(9.5, -2,0);
				glTexCoord2f(0.99f,0.99f); glVertex3f(9.5, 9,0);
				glTexCoord2f(0.99f,0.01f); glVertex3f(-2.5, 9,0);
			glEnd();
		}
		if (thymio->getColorLed(Thymio2::BOTTOM_RIGHT).a() != 0.0)
		{
			const Color color = thymio->getColorLed(Thymio2::BOTTOM_RIGHT) * 0.6;
			glColor4d(color.r(),color.g(),color.b(),color.a());

			glBegin (GL_QUADS);
				glNormal3f (0,0,1);
				glTexCoord2f(0.99f,0.01f); glVertex3f(-2.5,-9,0);
				glTexCoord2f(0.99f,0.99f); glVertex3f(9.5,-9,0);
				glTexCoord2f(0.01f,0.99f); glVertex3f(9.5, 2,0);
				glTexCoord2f(0.01f,0.01f); glVertex3f(-2.5, 2,0);
			glEnd();
		}
		glDisable(GL_POLYGON_OFFSET_FILL);
		glDepthMask( GL_TRUE );

		// end
		glDisable(GL_LIGHTING);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);
	}

	unsigned Thymio2Model::updateLedTexture(Thymio2* thymio) const
	{
		if (!thymio->ledTexture)
		{
			thymio->ledTexture = new uint32_t[textureDimension*textureDimension];
			std::fill(&thymio->ledTexture[0], &thymio->ledTexture[textureDimension*textureDimension], 0xFFFFFFFF);
		}
		
		uint32_t* tex = thymio->ledTexture;
		uint32_t* bodyTex   = (uint32_t*)bodyTexture.bits();
		uint32_t* bodyDiff0 = (uint32_t*)bodyDiffusionMap0.bits();
		uint32_t* bodyDiff1 = (uint32_t*)bodyDiffusionMap1.bits();
		uint32_t* bodyDiff2 = (uint32_t*)bodyDiffusionMap2.bits();
		
		// fill with body texture
		assert(bodyTex);
		std::copy(&bodyTex[0], &bodyTex[textureDimension*textureDimension], &tex[0]);
		
		// color led areas
		for (unsigned i=0; i<Thymio2::LED_COUNT; i++)
		{
			for (unsigned j=0;j<ledCenter[i].size();j++)
			{
				const Color ledColor = thymio->getColorLed((Thymio2::LedIndex)i);
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
		
		const unsigned texId(viewer->bindTexture(QImage((uint8_t*)(thymio->ledTexture), textureDimension, textureDimension, QImage::Format_ARGB32), GL_TEXTURE_2D));
		
		return texId;
	}
	
	// generated with this Python code:
	// str(', ').join(map(lambda x: str(int(x*255)), pow(np.arange(0., 1.001, 1./255.), 0.30)))
	static const uint32_t pow_030_table[256] = { 0, 48, 59, 67, 73, 78, 82, 86, 90, 93, 96, 99, 101, 104, 106, 108, 111, 113, 115, 117, 118, 120, 122, 123, 125, 127, 128, 130, 131, 132, 134, 135, 136, 138, 139, 140, 141, 142, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 160, 161, 162, 163, 164, 165, 166, 166, 167, 168, 169, 169, 170, 171, 172, 173, 173, 174, 175, 175, 176, 177, 178, 178, 179, 180, 180, 181, 182, 182, 183, 184, 184, 185, 185, 186, 187, 187, 188, 189, 189, 190, 190, 191, 191, 192, 193, 193, 194, 194, 195, 195, 196, 197, 197, 198, 198, 199, 199, 200, 200, 201, 201, 202, 202, 203, 203, 204, 204, 205, 205, 206, 206, 207, 207, 208, 208, 209, 209, 210, 210, 211, 211, 212, 212, 213, 213, 213, 214, 214, 215, 215, 216, 216, 217, 217, 217, 218, 218, 219, 219, 220, 220, 220, 221, 221, 222, 222, 222, 223, 223, 224, 224, 224, 225, 225, 226, 226, 226, 227, 227, 228, 228, 228, 229, 229, 230, 230, 230, 231, 231, 231, 232, 232, 233, 233, 233, 234, 234, 234, 235, 235, 236, 236, 236, 237, 237, 237, 238, 238, 238, 239, 239, 239, 240, 240, 240, 241, 241, 241, 242, 242, 242, 243, 243, 243, 244, 244, 244, 245, 245, 245, 246, 246, 246, 247, 247, 247, 248, 248, 248, 249, 249, 249, 250, 250, 250, 251, 251, 251, 251, 252, 252, 252, 253, 253, 253, 254, 254, 254, 255 };
	static const uint32_t pow_035_table[256] = { 0, 36, 46, 53, 59, 64, 68, 72, 75, 79, 82, 84, 87, 89, 92, 94, 96, 98, 100, 102, 104, 106, 108, 109, 111, 113, 114, 116, 117, 119, 120, 121, 123, 124, 125, 127, 128, 129, 130, 132, 133, 134, 135, 136, 137, 138, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 150, 151, 152, 153, 154, 155, 156, 157, 158, 158, 159, 160, 161, 162, 163, 163, 164, 165, 166, 166, 167, 168, 169, 169, 170, 171, 172, 172, 173, 174, 175, 175, 176, 177, 177, 178, 179, 179, 180, 181, 181, 182, 183, 183, 184, 185, 185, 186, 186, 187, 188, 188, 189, 189, 190, 191, 191, 192, 192, 193, 194, 194, 195, 195, 196, 197, 197, 198, 198, 199, 199, 200, 200, 201, 201, 202, 203, 203, 204, 204, 205, 205, 206, 206, 207, 207, 208, 208, 209, 209, 210, 210, 211, 211, 212, 212, 213, 213, 214, 214, 215, 215, 216, 216, 217, 217, 218, 218, 218, 219, 219, 220, 220, 221, 221, 222, 222, 223, 223, 223, 224, 224, 225, 225, 226, 226, 227, 227, 227, 228, 228, 229, 229, 230, 230, 230, 231, 231, 232, 232, 232, 233, 233, 234, 234, 235, 235, 235, 236, 236, 237, 237, 237, 238, 238, 239, 239, 239, 240, 240, 240, 241, 241, 242, 242, 242, 243, 243, 244, 244, 244, 245, 245, 245, 246, 246, 247, 247, 247, 248, 248, 248, 249, 249, 250, 250, 250, 251, 251, 251, 252, 252, 252, 253, 253, 253, 254, 254, 255 };
	static const uint32_t pow_040_table[256] = { 0, 27, 36, 43, 48, 52, 56, 60, 63, 66, 69, 72, 75, 77, 79, 82, 84, 86, 88, 90, 92, 93, 95, 97, 99, 100, 102, 103, 105, 106, 108, 109, 111, 112, 113, 115, 116, 117, 119, 120, 121, 122, 123, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 136, 137, 138, 139, 140, 141, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 152, 153, 154, 155, 156, 157, 157, 158, 159, 160, 161, 161, 162, 163, 164, 165, 165, 166, 167, 168, 168, 169, 170, 171, 171, 172, 173, 173, 174, 175, 176, 176, 177, 178, 178, 179, 180, 180, 181, 182, 182, 183, 184, 184, 185, 186, 186, 187, 187, 188, 189, 189, 190, 191, 191, 192, 192, 193, 194, 194, 195, 195, 196, 197, 197, 198, 198, 199, 200, 200, 201, 201, 202, 202, 203, 204, 204, 205, 205, 206, 206, 207, 207, 208, 208, 209, 210, 210, 211, 211, 212, 212, 213, 213, 214, 214, 215, 215, 216, 216, 217, 217, 218, 218, 219, 219, 220, 220, 221, 221, 222, 222, 223, 223, 224, 224, 225, 225, 226, 226, 227, 227, 228, 228, 229, 229, 229, 230, 230, 231, 231, 232, 232, 233, 233, 234, 234, 235, 235, 235, 236, 236, 237, 237, 238, 238, 239, 239, 239, 240, 240, 241, 241, 242, 242, 242, 243, 243, 244, 244, 245, 245, 245, 246, 246, 247, 247, 248, 248, 248, 249, 249, 250, 250, 250, 251, 251, 252, 252, 252, 253, 253, 254, 254, 255 };

	void Thymio2Model::drawRect(uint32_t* target, uint32_t* base, const Vector& center, const Vector& size, const Color& color, uint32_t* diffTex) const
	{
		assert(diffTex);
		
		const uint32_t colorA(color.a() * 255);
		const uint32_t colorR(color.r() * 255);
		const uint32_t colorG(color.g() * 255);
		const uint32_t colorB(color.b() * 255);
		
		for (int j = center.y*textureDimension - size.y*textureDimension/2; j < center.y*textureDimension + size.y*textureDimension/2; j++)
			for (int i = center.x*textureDimension - size.x*textureDimension/2; i < center.x*textureDimension + size.x*textureDimension/2; i++)
			{
				if (i<0 || j<0 || i>=textureDimension || j>=textureDimension)
					continue;
				
				// index
				const size_t index(i+textureDimension*j);

				// expand destination (prev color) into its components
				uint32_t& destination(target[index]);
				const uint32_t destA((destination>>24) & 0xff);
				const uint32_t destR((destination>>16) & 0xff);
				const uint32_t destG((destination>>8)  & 0xff);
				const uint32_t destB((destination>>0)  & 0xff);
				
				// expand diffuse into its components
				const uint32_t diffuse(diffTex[index]);
				const uint32_t diffA((diffuse>>24) & 0xff);
				const uint32_t diffR((diffuse>>16) & 0xff);
				const uint32_t diffG((diffuse>>8)  & 0xff);
				const uint32_t diffB((diffuse>>0)  & 0xff);
				
				// compute source color
				// gamma correction because LEDs have non-linear transfer functions
				const uint32_t sourceA((colorA * diffA) >> 8);
				const uint32_t sourceR(pow_035_table[(colorR * diffR) >> 8]);
				const uint32_t sourceG(pow_030_table[(colorG * diffG) >> 8]);
				const uint32_t sourceB(pow_040_table[(colorB * diffB) >> 8]);
				const uint32_t oneMSrcA(255-sourceA);
				
				// blend color
				destination = 
					(((destR * oneMSrcA + sourceR * sourceA) >> 8) << 16 ) |
					(((destG * oneMSrcA + sourceG * sourceA) >> 8) << 8  ) |
					(((destB * oneMSrcA + sourceB * sourceA) >> 8) << 0  ) |
					0xff000000
				;
			}
	}

} // namespace Enki


