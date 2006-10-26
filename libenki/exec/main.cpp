/***************************************************************
 * main of the simulator for testing the features				*
 *														*
 * main part creating a world and some objects for test as 	*
 * robot, etc.
 *														*
 * (c) Porchet Vincent 2004 								*
 * supervisor Magnenat Stephane								*
 ***************************************************************/

#include "libsim/PhysicalEngine.h"

#include <iostream>

#include "robots/s-bot/SbotCam.h"
#include "robots/s-bot/Sbot.h"


CollisionList collisionList;

int main(int argc, char *argv[])
{
	std::cout<<"main of sim call"<<std::endl;
	
	World *w = new World(10.0,10.0);
// 	std::cout<<"new world : w, created.\n"<<std::endl;
	
	Sbot *r1 = new Sbot();
	Point p1(1.0,1.0);
	r1->pos=p1;
	std::cout << "r1->pos = " << r1->pos.x << "," << r1->pos.y << std::endl;
	std::cout << "r1->angle = " << r1->angle << std::endl;

	w->addObject(r1);

	SbotCam cam1r1(5.0, r1);

	r1->addLocalInteraction(&cam1r1);
/*
	SbotCam cam2r1(3.0, r1);

	r1->addLocalInteraction(&cam2r1);

	SbotCam cam3r1(1.0, r1);

	r1->addLocalInteraction(&cam3r1);
*/

	Sbot *r2 = new Sbot();
	Point p2(3.0 , 3.0);
	r2->pos=p2;
	std::cout<<"r2->pos = "<<r2->pos.x<<","<<r2->pos.y<<std::endl;

	w->addObject(r2);

// 	SbotCam cam1r2(4.0, r2);
// 
// 	r2->addLocalInteraction(&cam1r2);
// 
// 	SbotCam cam2r2(1.0, r2);
// 
// 	r2->addLocalInteraction(&cam2r2);



	w->step(1.0);
// 	std::cout<<"w.step(0.1) done!!!!!!"<<std::endl;

	delete w;
	
	std::cout<<"sim has run.\n"<<std::endl;
}
