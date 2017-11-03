#ifndef PARAM_H
#define PARAM_H

#include <uav/logging>

namespace uav
{

struct param
{
    enum : uint8_t
    {
        f1hz = 1, f10hz = 10, f20hz = 20, f25hz = 25,
        f40hz = 40, f50hz = 50, f100hz = 100, f125hz = 125,
        f200hz = 200, f250hz = 250, fdefault = f100hz
    };

    uint8_t freq;
    double mass;
    std::array<double, 20> pid_gains;

    static std::string header();
};

archive& operator << (archive& a, const param& p);

archive& operator >> (archive& a, param& p);

} // namespace uav

#endif // PARAM_H
