#ifndef ARDIMU_H
#define ARDIMU_H

#include <fstream>
#include <cstdlib>
#include <thread>
#include <termios.h>

namespace uav
{
    struct imu_packet
    {
        uint64_t millis;
        double heading;
        double pitch;
        double roll;
        uint8_t calib;
        double pres;
        double ax, ay, az;
    };

    class arduino
    {
        public:

        arduino();
        ~arduino();

        int begin();
        const imu_packet& get() const;

        private:
        
        imu_packet data;

        const static size_t buffer_size = 1000;
        const static speed_t baud = B115200;

        int fd;
        std::thread parser;
        bool cont;
        int status;

        void parse();
    };
}

#endif // ARDIMU_H
