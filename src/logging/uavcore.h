#ifndef UAV_CORE_H
#define UAV_CORE_H

#include <fstream>
#include <sstream>
#include <deque>
#include <array>
#include <limits>

namespace uav
{

// takes pressure in atm
double altitude(double atm);

// data representation typedefs ////////////////////////////////////////////

enum : uint8_t
{
    f1hz = 1, f10hz = 10, f20hz = 20, f25hz = 25, f40hz = 40, f50hz = 50,
    f100hz = 100, f125hz = 125, f200hz = 200, f250hz = 250,

    fdefault = f100hz
};

enum : uint8_t
{
    null_status, align, no_vel, pos_seek, pos_hold, high_tilt, upside_down
};

// formatting bitmasks /////////////////////////////////////////////////////

namespace fmt
{
    using bitmask_t = uint16_t;

    enum : bitmask_t
    {
        time            = 1,
        time_full       = (1 << 1) | time,
        position        = 1 << 3,
        attitude        = 1 << 4,
        config          = position | attitude,
        quat            = 1 << 5,
        targets         = 1 << 6,
        motors          = 1 << 7,
        error           = 1 << 8,
        status          = 1 << 9,

        all = std::numeric_limits<bitmask_t>::max(),
        standard = time | config | motors | status
    };
}

// uav::state //////////////////////////////////////////////////////////////

struct state
{
    std::array<uint64_t, 3> time;
    std::array<double, 3> position;
    std::array<double, 3> attitude;
    std::array<double, 4> targets;
    std::array<double, 4> motors;
    uint16_t error;
    uint8_t status;

    static std::string header(fmt::bitmask_t);
    bool operator==(const state& other);
    bool operator!=(const state& other);

    const static size_t fields = 30;
    const static size_t size = 3*sizeof(uint64_t) +
        14*sizeof(double) + sizeof(uint16_t) + sizeof(uint8_t);

    using bin = std::array<uint8_t, size>;
};

// uav::param //////////////////////////////////////////////////////////////

struct param
{
    uint8_t freq; // frequency of updates in hz
    double mass;
    // pid gains for altitude, heading, pitch, roll
    std::array<double, 20> pid_gains;

    static std::string header();
    bool operator==(const param& other);
    bool operator!=(const param& other);

    const static size_t size = sizeof(uint8_t) + 21*sizeof(double);

    using bin = std::array<uint8_t, size>;
};

// conversion functions ////////////////////////////////////////////////////

param::bin serialize(const param& p);

state::bin serialize(const state& s);

param deserialize(const param::bin& b);

state deserialize(const state::bin& b);

// std::string functions ///////////////////////////////////////////////////

std::string to_string(const param& prm);

std::string to_string(const state& it, fmt::bitmask_t mask);

} // namespace uav

#endif // UAV_CORE_H
