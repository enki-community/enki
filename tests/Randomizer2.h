#ifndef __RANDOMIZER2_H
#define __RANDOMIZER2_H

#include <enki/PhysicalEngine.h>
#include <enki/robots/thymio2/Thymio2.h>
#include <enki/robots/e-puck/EPuck.h>

namespace Enki
	{
	class Randomizer
	{
	private:
		World* world;

	public:
		Randomizer(World* w);
		~Randomizer()
		{};

		Robot* generateRobot();
		Thymio2* generateThymio();
		EPuck* generateEPuck();

	private:
		Point generatePoint();
		float generateFloat(float range);
		unsigned generateInt(unsigned range);

	};
}
#endif
