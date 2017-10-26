#ifndef ARDIMU_H
#define ARDIMU_H

#include <cstdlib>
#include <thread>
#include <atomic>
#include <vector.h>

namespace uav
{

struct arduino_data
{
    uint64_t millis;
    imu::Vector<3> euler;
    uint8_t calib;
    double pres;
    imu::Vector<3> acc;
};

class arduino
{
    public:

    arduino();
    ~arduino();

    int begin();
    arduino_data get() const;

    private:
    
    std::atomic<arduino_data> data;
    std::atomic<bool> cont;
    std::atomic<int> status, fd;

    std::thread parser;

    void parse();
};

} // namespace uav

#endif // ARDIMU_H
