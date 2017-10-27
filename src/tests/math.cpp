#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <uav/math>

using namespace uav;

TEST_CASE( "Test angle arithmetic.", "[angle]" )
{
    REQUIRE( angle(2) + angle(3) == angle(5) );
    REQUIRE( angle(-0.5, 1) == angle(2*M_PI - 0.5, 0) );
    REQUIRE( angle(3, 1) * 2 == angle(6, 2) );
}

TEST_CASE( "Test angular construction.", "[angular]" )
{
    REQUIRE( angular(2, 20, 45, 194) == angular(2, 20, 45, 194) );
    REQUIRE( angular() == angular(0) );
    REQUIRE( angular(1, 60) == angular(2) );
    REQUIRE( angular(2, 10, 30.5) == angular(2, 10, 30, 500) );
}
