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

#ifndef __ENKI_ENTITY_H
#define __ENKI_ENTITY_H

#include <set>
#include <memory>
#include "Geometry.h"

namespace Enki
{
    class Component;

    class Entity
    {
    public:
        //! A vector of owned components
        typedef std::vector<std::unique_ptr<Component>> Components;
        //! A vector of owned children entities
        typedef std::vector<Entity> Entities;

    protected:
        //! The parent entity, nullptr if top-level
        Entity* parent = nullptr;
        //! The children entities
        Entities children;

        //! The components attached to this entity, whose ownerships lie with the entity
        Components components;

        //! The position of the entity
        Point pos;
        //! The yaw of the entity in the world, standard trigonometric yaw
        double yaw = 0.;

        //! Absolute position in the world, updated on setPose()
		Vector absPos;
		//! Absolute yaw in the world, updated on setPose()
		double absYaw = 0.;

    public:
        // delete copy constructors
        //Entity(const Entity&) = delete;
        //Entity& operator=(const Entity&) = delete;
        //~Entity() = default;

        // pose setters
        void setPose(const Point& pos, double yaw);
        void setPos(const Point& pos) { setPose(pos, yaw); }
        void setYaw(double yaw) { setPose(pos, yaw); }
        // pose getters
        Point getPos() const { return pos; }
        double getYaw() const { return yaw; }
        Matrix22 getYawMatrix() const { return Matrix22(yaw); }
        // absolute pose getters
        Point getAbsPos() const { return absPos; }
        double getAbsYaw() const { return absYaw; }
        Matrix22 getAbsYawMatrix() const { return Matrix22(absYaw); }

        // components
        //! Add a component instance, take ownership of it
        void addComponent(Component* component);
        //! Get the list of components
        const Components& getComponents() const;
        //! Get the first component of a given type
        template<typename ComponentType>
        ComponentType& getComponent() const
        {
            // iterate through components, return if dynamic cast succeeds
            for (auto& componentPtr: components)
            {
                ComponentType componentRawPtr(dynamic_cast<ComponentType>(componentPtr.get()));
                if (componentRawPtr)
                    return componentRawPtr;
            }
            // none found, runtime error
            throw std::runtime_error(std::string("Component of type ") + typeid(ComponentType).name() + "not found");
            // FIXME: use specific exception
        }


    protected:
        void updateAbsolutePose();
    };
} // namespace Enki

#endif // __ENKI_ENTITY_H
