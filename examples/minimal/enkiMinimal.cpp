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
	
	Enki::Polygone p;
	const double amount = 9;
	const double radius = 5;
	const double height = 20;
	for (double a = 0; a < 2*M_PI; a += 2*M_PI/amount)
		p.push_back(Enki::Point(radius * cos(a), radius * sin(a)));
	Enki::PhysicalObject* o = new Enki::PhysicalObject;
	Enki::PhysicalObject::Hull hull(Enki::PhysicalObject::Part(p, height));
	o->setCustomHull(hull, 1);
	o->pos = Enki::Point(100, 100);
	o->setColor(Enki::Color(0.4,0.6,0.8));
	world.addObject(o);
	
	// Run for some times
	for (unsigned i=0; i<10; i++)
	{
		// step of 50 ms
		world.step(0.05);
		std::cout << "E-puck pos is (" << ePuck->pos.x << "," << ePuck->pos.y << ")" << std::endl;
	}
}

