#include "catch.hpp"

#include <uav/logging>

TEST_CASE( "Testing archive packing and unpacking.", "[archive]" )
{
    uav::archive arch;
    auto b = 38241023498;
    auto c = 5.4;
    auto d = -3.23f;
    auto e = -49999;
    arch << b << c << d << e;

    decltype(b) w;
    decltype(c) x;
    decltype(d) y;
    decltype(e) z;
    arch >> w >> x >> y >> z;

    REQUIRE( b == w );
    REQUIRE( c == x );
    REQUIRE( d == y );
    REQUIRE( e == z );

    int throwaway;
    arch >> throwaway;

    REQUIRE( arch.fail() );
    arch.clear();
    REQUIRE( !arch.fail() );

    std::array<double, 4> array{4.5, -3.27, 91.42, 0.5542};
    arch << array;

    decltype(array) new_array;

    arch >> new_array;

    for (int i = 0; i < array.size(); i++)
        REQUIRE( array[i] == new_array[i] );
}
