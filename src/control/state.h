#ifndef STATE_H
#define STATE_H

#include <uav/math>
#include <uav/logging>

namespace uav
{

class state
{
    public:

    enum : uint8_t
    {
        null_status, align, no_vel, pos_seek,
        pos_hold, high_tilt, upside_down
    };

    std::array<uint64_t, 3> time;
    coordinate position;
    std::array<angle, 3> attitude;
    std::array<angle, 4> targets;
    std::array<double, 4> motors;
    uint16_t error;
    uint8_t status;
};

archive& operator << (archive& a, const angle& b);

archive& operator >> (archive& a, angle& b);

archive& operator << (archive& a, const coordinate& c);

archive& operator >> (archive& a, coordinate& c);

archive& operator << (archive& a, const state& s);

archive& operator >> (archive& a, state& s);

} // namespace uav

#endif // STATE_H
