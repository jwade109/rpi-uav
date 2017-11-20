#include "catch.hpp"

#include <uav/logging>

TEST_CASE( "Testing archive packing and unpacking.", "[archive]" )
{
    uav::archive arch;
    std::string a = "Hello!";
    auto b = 38241023498;
    auto c = 5.4;
    auto d = -3.23f;
    auto e = -49999;
    arch << a << b << c << d << e;

    decltype(a) v;
    decltype(b) w;
    decltype(c) x;
    decltype(d) y;
    decltype(e) z;
    arch >> v >> w >> x >> y >> z;

    REQUIRE( a == v );
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

TEST_CASE( "Test logstream.", "[logstream]" )
{
    uav::logstream test("Test");
    uav::archive arch;
    arch << 45 << 4.3 << -9.21;
    test << arch;
    test << "This is a test!: " << 4.6 << "\nA wonderful test!\n";

    uav::flush();
    uav::flush(); // ensuring it doesn't overwrite the file
    auto v_a = uav::restore();

    REQUIRE( v_a.size() == 3 );
    uav::archive rest = v_a[0];
    REQUIRE( rest.bytes().size() ==
        sizeof(45) + sizeof(4.3) + sizeof(-9.21) );

    decltype(45) x;
    decltype(4.3) y;
    decltype(-9.21) z;

    rest >> x >> y >> z;
    REQUIRE( x == 45 );
    REQUIRE( y == 4.3 );
    REQUIRE( z == -9.21 );

    std::string str;
    v_a[1] >> str;
    REQUIRE( v_a[1].name() == "Test" );
    REQUIRE( str == "This is a test!: 4.600000" );

    v_a[2] >> str;
    REQUIRE( v_a[2].name() == "Test" );
    REQUIRE( str == "A wonderful test!" );
}
