#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <aseba/AsebaMarxbot.h>
#include <iostream>

int main(int argc, char *argv[])
{
	// Create the world
	Enki::World world(200, 200);
	
	// Create a Khepera and position it
	Enki::AsebaMarxbot *marXbot = new Enki::AsebaMarxbot();
	marXbot->pos = Enki::Point(100, 100);
	
	// objects are garbage collected by the world on destruction
	world.addObject(marXbot);
	
	// Run for some times
	while (true)
	{
		// step of 50 ms
		world.step(0.05);
		std::cout << "\rmarXbot pos is (" << marXbot->pos.x << "," << marXbot->pos.y << ")              ";
		std::cout.flush();
	}
}

