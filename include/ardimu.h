#ifndef ARDIMU_H
#define ARDIMU_H

#include <fstream>
#include <cstdlib>
#include <thread>

const int msg_len = 100;

namespace uav
{
    typedef struct
    {
        uint64_t millis;
        float heading;
        float pitch;
        float roll;
        uint8_t calib;
        float temp;
        float pres;
    }
    Message;

    class Arduino 
    {
        public:

        Arduino();
        ~Arduino();

        int begin();
        const Message& get();

        private:
        
        Message data;

        char buffer[msg_len];
        std::ifstream in;
        std::thread parser;
        bool cont;
        int init;

        void parse();
    };
}

#endif // ARDIMU_H
