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

#include <inttypes.h>
#include <thread>

typedef struct
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
}
gps_data;

typedef struct
{
    uint16_t serial, records;
    uint8_t type, mode, config, interval,
            distance, speed, status, percent;
}
locus_info;

class GPS
{
    public:

    GPS();
    ~GPS();

    const gps_data& get() const;
    const locus_info& getinfo() const;

    void begin();
    void end();
   
    private:

    gps_data data;
    locus_info linfo;
    std::thread reader;
    bool cont;
    bool paused, asleep;

    char *lastNMEA();
    bool newNMEAreceived();
    void common_init();
    void sendCommand(const char *);
    void pause(bool b);
    bool parseNMEA(char *response);
    uint8_t parseHex(char c);
    char read();
    bool parse(char *);
    void interruptReads(bool r);
    bool wakeup();
    bool standby();

    bool waitForSentence(const char *wait,
            uint8_t max = max_resp_wait);
    bool LOCUS_StartLogger();
    bool LOCUS_StopLogger();
    bool LOCUS_ReadStatus();

    uint8_t parseResponse(char *response);

    /* commonly used sentences --------------------- */

    // how long to wait when we're looking for a response
    static const int max_resp_wait = 5;
    
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
