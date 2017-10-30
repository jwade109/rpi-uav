#include "catch.hpp"

#include <uav/logging>

TEST_CASE( "Testing file IO." )
{
    uav::logstream inspvax("INSPVAX");
    inspvax.add({1, 1, 2, 3, 5, 8, 13, 21, 34, 55});
    uav::flush();
}
