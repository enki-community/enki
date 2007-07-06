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

#ifndef __ENKI_ASEBA_MARXBOT_H
#define __ENKI_ASEBA_MARXBOT_H

#include <enki/robots/marxbot/Marxbot.h>
#include <aseba/Network.h>

/*!	\file AsebaMarxbot.h
	\brief Header of the aseba-enabled marXbot robot
*/

namespace Enki
{
	
	//! The aseba-enabled version of the marXbot robot.
	/*! This robot provides a full simulation of event-based architecture
		as present on the real marXbot, using aseba.
		This robot provides a tcp server connection on port ASEBA_DEFAULT_PORT
		(currently 33333), or higher if other AsebaMarxbot already use it.
		This allows the connection of Aseba Studio to this robot.
		The feature is provided by inheriting from a Network server
		\ingroup robot
	*/
	class AsebaMarxbot : public Marxbot, public NetworkServer
	{
	public:
		//! In addition to DifferentialWheeled::step(), update aseba variables and initiate periodic events.
		virtual void step(double dt);
		
		virtual void incomingData(Socket *socket);
		virtual void incomingConnection(Socket *socket);
		virtual void connectionClosed(Socket *socket);
	};

}
#endif

