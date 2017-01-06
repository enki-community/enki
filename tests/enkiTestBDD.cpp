#include "catch.hpp"

#include <enki/PhysicalEngine.h>
#include <enki/robots/e-puck/EPuck.h>
#include <iostream>

SCENARIO( "An Enki world may contain a robot", "[Enki::World]" ) {

    GIVEN( "An empty world" ) {
        Enki::World world(200, 200);

        REQUIRE( world.w == 200 );
        REQUIRE( world.h == 200 );

        WHEN( "An ePuck is added" ) {
			Enki::EPuck *ePuck = new Enki::EPuck;
			ePuck->pos = Enki::Point(100, 100);
			ePuck->leftSpeed = 30;
			ePuck->rightSpeed = 20;
			world.addObject(ePuck);

            THEN( "The world contains at least one object" ) {
                REQUIRE( world.objects.size() >= 1 );
            }
        }
	}
}
