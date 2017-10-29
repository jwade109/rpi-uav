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

#include <thread>
#include <string>
#include <vector>

#include <uav/math>

namespace uav
{

struct utc_time
{
    uint8_t hour, minute, second;
    uint16_t ms;
};

// contains time, position and fix related data of the GNSS receiver
struct gpgga
{
    utc_time utc;
    coordinate pos;

    uint8_t fix_quality;
    uint8_t num_sats;
    float hdop;
    double altitude;
    char alt_unit;
    double undulation;
    char und_unit;

    bool has_dgps;
    uint8_t corr_age;
    std::string base_ID;
};

// contains GNSS receiver operating mode, satellites used
// for navigation, and DOP values
struct gpgsa
{
    char mode_char;
    uint8_t mode_num;
    // up to 12 satellites' PRN numbers
    std::vector<uint8_t> sat_prns;
    float pdop, hdop, vdop;
};

// contains time, date, position, track made good and speed data
struct gprmc
{
    utc_time utc;

    char pos_status;
    coordinate pos;
    double ground_speed;
    double track_angle;

    uint8_t day, month, year;

    double mag_var;
    char var_dir;
    char pos_mode;
};

// contains the track made good and speed relative to the ground
struct gpvtg
{
    double track_true;
    char track_indicator;
    double track_mag;
    char mag_indicator;
    double ground_speed_knots;
    char speed_ind_knots;
    double ground_speed_kph;
    char speed_ind_kph;
    char mode_ind;
};

// contains the number of GPS SVs in view, PRN numbers,
// elevation, azimuth and SNR value
struct gpgsv
{
    struct sat_info
    {
        uint8_t PRN;
        uint8_t elevation;
        uint16_t azimuth;
        uint8_t SNR;
    };

    uint8_t num_msgs;
    uint8_t msg_num;
    uint8_t sats_in_view;

    std::vector<sat_info> sats;
};

struct gps_data
{
    gpgga gga;
    gpgsa gsa;
    gprmc rmc;
    gpvtg vtg;
    std::vector<gpgsv> gsv;
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

    gps();
    ~gps();

    bool isnew() const;
    gps_data get();
    bool update(gps_data& gp);

    int begin();

    private:

    gps_data data;
    std::thread reader;

    bool newflag;
    bool cont;
    int status;
    int fd;

    void dowork();
    void update_info(std::stringstream& ss);

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

std::ostream& operator << (std::ostream& os, const utc_time& u);

std::ostream& operator << (std::ostream& os, const gpgga& g);

std::ostream& operator << (std::ostream& os, const gpgsa& g);

std::ostream& operator << (std::ostream& os, const gprmc& g);

std::ostream& operator << (std::ostream& os, const gpgsv::sat_info& s);

std::ostream& operator << (std::ostream& os, const gpgsv& g);

std::ostream& operator << (std::ostream& os, const gpvtg& g);

utc_time parse_utc(const std::string& data);

angle parse_lat(const std::string& data);

angle parse_lon(const std::string& data);

gpgga parse_gpgga(const std::string& data);

gpgsa parse_gpgsa(const std::string& nmea);

gprmc parse_gprmc(const std::string& nmea);

gpgsv parse_gpgsv(const std::string& nmea);

gpvtg parse_gpvtg(const std::string& nmea);

} // namespace uav

#endif // GPS_H
