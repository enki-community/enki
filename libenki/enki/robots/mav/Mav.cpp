#include "Mav.h"

/*! \file Mav.cpp
	\brief Implementation of the MAV robot
*/

namespace Enki
{
	using namespace An;

	Mav::Mav(unsigned capabilities) :
		comm(100.0, this)
	{
		mass = 1;
		if (capabilities & CAPABILITIY_BASIC_SENSORS)
		{
			addLocalInteraction(&comm);
		}
		speed = roll = currentRoll=0;
		currentRollSet=false;
	}
	
	Mav::~Mav()
	{
	}
	void Mav::step(double dt) 
	{	
		// +/- 5% motor noise
		double realSpeed = forwardSpeed* (0.9 + random.getRange(0.1));
		double realAngularSpeed;
		double rand = An::uniformRand();
		double val = (rand*M_PI/18.0)-(M_PI/36.0);
		//if the current roll is unknown, set it to the desired roll
		if(!currentRollSet)
		{
			currentRollSet=true;
			currentRoll=roll;
		}
		//if the current roll is smaller than the desired roll, increase the current roll by 5°
		if(roll>currentRoll)
		{
			currentRoll+=M_PI/36.0;
		}
		//if the current roll is larger than the desired roll, decrease the current roll by 5°
		else if(roll<currentRoll)
		{
			currentRoll-=M_PI/36.0;
		}
		currentRoll+=val;
		//compute angular velocity
		if (realSpeed != 0.0)
			realAngularSpeed = (tan(currentRoll)*mass*9.81)/realSpeed;
		else
			realAngularSpeed =0.0;
		
		double forwardSpeed = realSpeed;
		angSpeed = realAngularSpeed;
			
		//compute speed vector
		speed = Vector(
			forwardSpeed * cos(angle + angSpeed * dt),
			forwardSpeed * sin(angle + angSpeed * dt));
		
		// Move the MAV	
		PhysicalObject::step(dt);
	}
}

