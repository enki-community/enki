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

#ifndef __ENKI_SYSTEM_H
#define __ENKI_SYSTEM_H

#include <set>
#include <vector>

#include "World.h"

//#include "Component.h"

namespace Enki
{
    class Component;

	struct Phase
	{
		virtual void init(World* world) {}
		virtual void step(double dt) = 0;
	};

	template<typename SystemType, typename ComponentType>
	struct GlobalPhase: Phase
	{
		SystemType* system;
		ComponentSetRef<ComponentType> components;

		GlobalPhase(World* world, SystemType *system):
            system(system),
            components(world->getComponentsByType<ComponentType>())
        {}

		virtual void step(double dt)
        {
            for (auto component: components)
    			component->step(dt, system);
        }
	};

	template<typename SystemType, typename ComponentType, typename TargetComponentType>
	struct LocalPhase: Phase
	{
		SystemType* system;
		ComponentSetRef<ComponentType> components;
		ComponentSetRef<TargetComponentType> targetComponents;

		LocalPhase(World* world, SystemType *system):
    		system(system),
    		components(world->getComponentsByType<ComponentType>()),
    		targetComponents(world->getComponentsByType<TargetComponentType>())
        {}

		virtual void step(double dt)
        {
            // init
    		for (auto component: components)
    			component->init(dt, system);
    		// step to all other objects
    		// TODO: early out through kd-tree
    		for (auto component: components)
    			for (auto targetComponent: targetComponents)
    				component->step(dt, system, targetComponent);
    		// finalize
    		for (auto component: components)
    			component->finalize(dt, system);
        }
	};

	struct System
	{
		World* world = nullptr;
		std::vector<std::unique_ptr<Phase>> phases;

        System() = default;
        System(World* world): world(world) {}

        // delete copy constructors
        //System(const System&) = delete;
        //System& operator=(const System&) = delete;
        //~System() = default;

		virtual void step(double dt);
	};

    struct UndersampledSystem: System
    {
        unsigned subsamplingFactor = 1;
		unsigned subsamplingStep;

        // delete copy constructors
        //UndersampledSystem(const UndersampledSystem&) = delete;
        //UndersampledSystem& operator=(const UndersampledSystem&) = delete;
        //~UndersampledSystem() = default;

        virtual void step(double dt);
    };

    struct TimerSystem: System
    {
        TimerSystem(World* world);
    };

} // namespace Enki

#endif // __ENKI_SYSTEM_H
