#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <iostream>

int main(int argc, char *argv[])
{
	// Create the world
	Enki::World world(200, 200);
	
	// Create a Khepera and position it
	Enki::EPuck *ePuck = new Enki::EPuck;
	ePuck->pos = Enki::Point(100, 100);
	ePuck->leftSpeed = 30;
	ePuck->rightSpeed = 20;
	
	// objects are garbage collected by the world on destruction
	world.addObject(ePuck);
	
	// Run for some times
	for (unsigned i=0; i<1000; i++)
	{
		// step of 50 ms
		world.step(0.05);
		std::cout << "E-puck pos is (" << ePuck->pos.x << "," << ePuck->pos.y << ")" << std::endl;
	}
}

