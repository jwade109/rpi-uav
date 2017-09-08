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
        float heading;
        float pitch;
        float roll;
        uint8_t calib;
        float temp;
        float pres;
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

        std::ifstream in;
        std::thread parser;
        bool cont;
        int status;

        void parse();
    };
}

#endif // ARDIMU_H
