/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2004 Stephane Magnenat <nct@ysagoon.com>
    Copyright (C) 2004 Autonomous Systems Lab 2, EPFL, Lausanne
    See AUTHORS for details   
     
    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator part of the Teem framework
    http://teem.epfl.ch
    Stephane Magnenat <stephane.magnenat@a3.epfl.ch>,
    Markus Waibel <markus.waibel@epfl.ch>
    Autonomous Systems Lab, EPFL, Lausanne.

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
/*
Sound MICROPHONES for Enki 2D fast simulator
*/

#ifndef __LOUDSPEAKER_H
#define __LOUDSPEAKER_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>

#include <valarray>

namespace Enki
{

	class Loudspeaker: public LocalInteraction {
	protected:
		
		PhysicalObject *owner;
		
	public:
		
		// No of channels of this sound source
		unsigned noOfChannels;
		
		// Produced sound: vector of different pitch as they were channels.
		double *pitch;
		
		Loudspeaker(PhysicalObject *current, unsigned channels) ;
		
		void setSound(unsigned channel, double signal);
		
		double getSound(unsigned channel);

		void realisticSetSound(unsigned channel, double signal);
	};

}

#endif
