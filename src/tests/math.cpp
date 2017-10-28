#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <uav/math>

TEST_CASE( "Test angle data type.", "[angle]" )
{
    using namespace uav;
    using namespace uav::angle_literals;

    angle a = 4.3_rad,
          b = 25_deg,
          c = b + 10_ms,
          d = 1_rev + 32_deg + 19_min,
          e = -d,
          f = 4_us,
          def;

    SECTION( "Internal representation." );
    {
        REQUIRE( def.micros() == 0 );
        REQUIRE( a == angle::radians(4.3) );
        REQUIRE( a.deg() > 4.3 * 57 );
        REQUIRE( a.deg() < 4.3 * 58 );
        REQUIRE( a.micros() == 886938666862 );
        REQUIRE( b.micros() == 90000000000 );
        REQUIRE( c.micros() == 90000010000 );
        REQUIRE( d.micros() == 1412340000000 );
        REQUIRE( e.micros() == -d.micros() );
        REQUIRE( f.micros() == 4 );
    }
    SECTION( "Angle comparators." )
    {
        REQUIRE( a == a );
        REQUIRE( d == d );
        REQUIRE( b < c );
        REQUIRE( b + 20_deg > c );
        REQUIRE( d <= d );
        REQUIRE( b != e );
        REQUIRE( e == -d );
    }
    SECTION( "Angle literals." )
    {
        REQUIRE( a == angle::radians(4.3) );
        REQUIRE( b == angle::degrees(25) );
        REQUIRE( c == angle::degrees(25) +
                      angle::milliseconds(10) );
        REQUIRE( d == angle::revolutions(1) +
                      angle::degrees(32) +
                      angle::minutes(19) );
        REQUIRE( e == angle::revolutions(-1) +
                      angle::degrees(-32) +
                      angle::minutes(-19) );
    }
    SECTION( "Arithmetic operations." )
    {
        REQUIRE( f * 3 == 12_us );
        REQUIRE( 3 * f == 12_us );
        REQUIRE( f + f + f == 12_us );
        REQUIRE( 1_rev / 60_deg == 6 );
        REQUIRE( 45_deg + 30_min == angle::degrees(45.5) );
        REQUIRE( (77_deg + 39_min + 60_sec + 0.658_ms) -
                 (77_deg + 39_min + 60_sec + 0.672_ms) ==
                 -14_us );
    }
}

