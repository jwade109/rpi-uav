#ifndef ARDIMU_H
#define ARDIMU_H

#include <cstdlib>
#include <thread>
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
    const arduino_data& get() const;

    private:
    
    arduino_data data;

    int fd;
    std::thread parser;
    bool cont;
    int status;

    void parse();
};

} // namespace uav

#endif // ARDIMU_H
