/***********************************
This is the Adafruit GPS library - the ultimate GPS library
for the ultimate GPS module!

Tested and works great with the Adafruit Ultimate GPS module
using MTK33x9 chipset
    ------> http://www.adafruit.com/products/746
Pick one up today at the Adafruit electronics shop 
and help support open source hardware & software! -ada

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/

// modified from original Adafruit library for Raspberry Pi 3
// by Wade Foster on 8/29/2017

#ifndef GPS_H
#define GPS_H

#include <fstream>
#include <inttypes.h>
#include <thread>
#include <string>

struct gps_data
{
    uint8_t hour, minute, seconds, year, month, day;
    uint16_t milliseconds;

    // Floating point latitude and longitude value in degrees.
    float latitude, longitude;

    // Fixed point latitude and longitude value with degrees stored
    // in units of 1/100000 degrees, and minutes stored in units of
    // 1/100000 degrees. See pull #13 for more details:
    // https://github.com/adafruit/Adafruit-GPS-Library/pull/13
    int32_t latitude_fixed, longitude_fixed;
    float latitude_degrees, longitude_degrees;
    float geoidheight, altitude;
    float speed, angle, magvariation, HDOP;
    char lat, lon, mag;
    bool fix;
    uint8_t fixquality, satellites;
};

struct locus_info
{
    uint16_t serial, records;
    uint8_t type, mode, config, interval,
            distance, speed, status, percent;
};

class gps
{
    public:

    std::string datastr;

    gps();
    ~gps();

    gps_data get() const;

    int begin();
   
    private:

    gps_data data;
    std::ifstream in;
    int tty_fd;
    std::thread reader;
    bool cont;

    void dowork();

    static gps_data parse(const std::string& nmea);
    static uint8_t parseHex(char c);
    
    // position echo rate commands (formerly PMTK_SET_NMEA_UPDATE_XXX_HERTZ)
    static const std::string
    pmtk_echo_100mHz, pmtk_echo_200mHz, pmtk_echo_1Hz,
    pmtk_echo_5Hz, pmtk_echo_10Hz,

    // position fix update rate (formerly PMTK_API_SET_FIX_CTL_XXX_HERTZ)
    pmtk_fix_100mHz, pmtk_fix_200mHz, pmtk_fix_1Hz, pmtk_fix_5Hz,

    // baud rate commands
    pmtk_B57600, pmtk_B9600,

    // data format commands
    pmtk_nmea_gprmc, pmtk_nmea_gprmc_gga, pmtk_nmea_all,

    pmtk_locus_startlog, pmtk_locus_stoplog, pmtk_locus_startstopack,
    pmtk_locus_query_status, pmtk_locus_erase_flash,

    pmtk_enable_sbas, pmtk_enable_waas,

    // standby command & boot successful message
    pmtk_standby, pmtk_standby_success, pmtk_awake,

    // ask for the release and version
    pmtk_query_release,

    // request for updates on antenna status 
    pgcmd_antenna, pgcmd_noantenna;
};

#endif // GPS_H

/*

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

*/
