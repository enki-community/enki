#include <enki/PhysicalEngine.h>
#include <enki/robots/khepera/Khepera.h>
#include <enki/robots/s-bot/SbotObject.h>
#include <iostream>

int main(int argc, char *argv[])
{
	// Create the world
	Enki::World world(200, 200);
	
	// Create a Khepera and position it
	Enki::Khepera *khepera = new Enki::Khepera();
	khepera->pos = An::Point(100, 100);
	khepera->leftSpeed = 30;
	khepera->rightSpeed = 20;
	
	// objects are garbage collected by the world on destruction
	world.addObject(khepera);
	
	// Run for some times
	for (unsigned i=0; i<1000; i++)
	{
		// step of 50 ms
		world.step(0.05);
		std::cout << "Khepera pos is (" << khepera->pos.x << "," << khepera->pos.y << ")" << std::endl;
	}

	Enki::SbotActiveSoundObject *saso = new Enki::SbotActiveSoundObject(10, 25);
	saso->speaker.setSound(10, 20);
	world.addObject(saso);
}
