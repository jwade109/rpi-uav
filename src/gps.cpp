/***********************************
This is our gps library

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/

// modified from original Adafruit library for Raspberry Pi 3
// by Wade Foster on 8/29/2017

#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <cctype>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <gps.h>

const std::string gps::pmtk_echo_100mHz("$PMTK220,10000*2F");
const std::string gps::pmtk_echo_200mHz("$PMTK220,5000*1B");
const std::string gps::pmtk_echo_1Hz("$PMTK220,1000*1F");
const std::string gps::pmtk_echo_5Hz("$PMTK220,200*2C");
const std::string gps::pmtk_echo_10Hz("$PMTK220,100*2F");

const std::string gps::pmtk_fix_100mHz("$PMTK300,10000,0,0,0,0*2C");
const std::string gps::pmtk_fix_200mHz("$PMTK300,5000,0,0,0,0*18");
const std::string gps::pmtk_fix_1Hz("$PMTK300,1000,0,0,0,0*1C");
const std::string gps::pmtk_fix_5Hz("$PMTK300,200,0,0,0,0*2F");

const std::string gps::pmtk_B57600("$PMTK251,57600*2C");
const std::string gps::pmtk_B9600("$PMTK251,9600*17");

const std::string gps::pmtk_nmea_gprmc
    ("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
const std::string gps::pmtk_nmea_gprmc_gga
    ("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
const std::string gps::pmtk_nmea_all
    ("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28");

const std::string gps::pmtk_locus_startlog("$PMTK185,0*22");
const std::string gps::pmtk_locus_stoplog("$PMTK185,1*23");
const std::string gps::pmtk_locus_startstopack("$PMTK001,185,3*3C");
const std::string gps::pmtk_locus_query_status("$PMTK183*38");
const std::string gps::pmtk_locus_erase_flash("$PMTK184,1*22");

const std::string gps::pmtk_enable_sbas("$PMTK313,1*2E");
const std::string gps::pmtk_enable_waas("$PMTK301,2*2E");

const std::string gps::pmtk_standby("$PMTK161,0*28");
const std::string gps::pmtk_standby_success("$PMTK001,161,3*36");
const std::string gps::pmtk_awake("$PMTK010,002*2D");

const std::string gps::pmtk_query_release("$PMTK605*31");

gps::gps() : data{0}, newflag(false), cont(true), status(0), tty_fd(0) { }

gps::~gps()
{
    cont = false;
    if (reader.joinable()) reader.join();
}

int gps::begin()
{
    int fd = open("/dev/ttyS0", O_RDONLY | O_NONBLOCK | O_NOCTTY);
    if (fd < 0)
    {
        std::cerr << "gps: Could not generate "
                     "file descriptor" << std::endl;
        return 1;
    }

    tty_fd = fd;

    tcflush(tty_fd, TCIOFLUSH);

    reader = std::thread(&gps::dowork, this);

    while (status == 0);
    if (status == -1)
    {
        std::cerr << "gps: failed to communicate!" << std::endl;
        return 2;
    }

    return 0;
}

bool gps::isnew() const
{
    return newflag;
}

gps_data gps::get()
{
    newflag = false;
    return data;
}

void gps::dowork()
{
    char ch;
    bool reading = false;
    std::vector<char> message;

    auto timeout = std::chrono::seconds(1);
    auto getnow = [](){ return std::chrono::steady_clock::now(); };
    auto start = getnow();

    tcflush(tty_fd, TCIOFLUSH);

    while (cont)
    {
        if (status == 0 && data.seconds != 0) status = 1;
        else if (status == 0 && getnow() > start + timeout)
        {
            cont = false;
            status = -1;
        }
        read(tty_fd, &ch, sizeof(ch));
        if (ch == '$')
        {
            reading = true;
        }
        if (ch == 10 && reading)
        {
            std::string str(message.begin(), message.end());
            if (str.find("GPGGA") != std::string::npos)
            {
                gps_data newgps = parse(str);
                if (checknew(newgps, data))
                {
                    newflag = true;
                    data = newgps;
                }
            }
            reading = false;
            message.clear();
        }
        else if (reading)
        {
            message.push_back(ch);
        }
    }
}

// static helper functions /////////////////////////////////////////////////////

bool gps::checknew(const gps_data& n, const gps_data& o)
{
    if (n.hour == 0 && o.hour == 23) return true;
    if (n.hour != o.hour) return n.hour > o.hour;
    if (n.minute != o.minute) return n.minute > o.minute;
    return n.seconds > o.seconds;
}

uint8_t gps::parseHex(char c)
{
    if (c < '0') return 0;
    if (c <= '9') return c - '0';
    if (c < 'A') return 0;
    if (c <= 'F') return (c - 'A') + 10;
    return 0;
}

gps_data gps::parse(const std::string& nmea)
{
    gps_data newgps;
    // do checksum check

    // first look if we even have one
    if (nmea[nmea.length() - 4] == '*')
    {
        uint16_t sum = parseHex(nmea[nmea.length() - 3]) * 16;
        sum += parseHex(nmea[nmea.length() - 2]);

        // check checksum
        for (uint8_t i=2; i < (nmea.length() - 4); i++)
        {
            sum ^= nmea[i];
        }
        // bad checksum
        if (sum != 0) return newgps;
    }
    int32_t degree;
    long minutes;
    char degreebuff[10];
    // look for a few common sentences
    if (nmea.find("$GPGGA") != std::string::npos)
    {
        // found GGA
        const char *p = nmea.c_str();
        // get time
        p = strchr(p, ',')+1;
        float timef = atof(p);
        uint32_t time = timef;
        newgps.hour = time / 10000;
        newgps.minute = (time % 10000) / 100;
        newgps.seconds = (time % 100);
        newgps.milliseconds = fmod(timef, 1.0) * 1000;

        // parse out latitude
        p = strchr(p, ',')+1;

        if (',' != *p)
        {
            strncpy(degreebuff, p, 2);
            p += 2;
            degreebuff[2] = '\0';
            degree = atol(degreebuff) * 10000000;
            strncpy(degreebuff, p, 2); // minutes
            p += 3; // skip decimal point
            strncpy(degreebuff + 2, p, 4);
            degreebuff[6] = '\0';
            minutes = 50 * atol(degreebuff) / 3;
            newgps.latitude_fixed = degree + minutes;
            newgps.latitude = degree / 100000 + minutes * 0.000006F;
            newgps.latitude_degrees = (newgps.latitude-100*int(newgps.latitude/100))/60.0;
            newgps.latitude_degrees += int(newgps.latitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'S') newgps.latitude_degrees *= -1.0;
            if (p[0] == 'N') newgps.lat = 'N';
            else if (p[0] == 'S') newgps.lat = 'S';
            else if (p[0] == ',') newgps.lat = 0;
            else return newgps;
        }

        // parse out longitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            strncpy(degreebuff, p, 3);
            p += 3;
            degreebuff[3] = '\0';
            degree = atol(degreebuff) * 10000000;
            strncpy(degreebuff, p, 2); // minutes
            p += 3; // skip decimal point
            strncpy(degreebuff + 2, p, 4);
            degreebuff[6] = '\0';
            minutes = 50 * atol(degreebuff) / 3;
            newgps.longitude_fixed = degree + minutes;
            newgps.longitude = degree / 100000 + minutes * 0.000006F;
            newgps.longitude_degrees = (newgps.longitude-100*int(newgps.longitude/100))/60.0;
            newgps.longitude_degrees += int(newgps.longitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'W') newgps.longitude_degrees *= -1.0;
            if (p[0] == 'W') newgps.lon = 'W';
            else if (p[0] == 'E') newgps.lon = 'E';
            else if (p[0] == ',') newgps.lon = 0;
            else return newgps;
        }

        p = strchr(p, ',')+1;
        if (',' != *p) newgps.fixquality = atoi(p);

        p = strchr(p, ',')+1;
        if (',' != *p) newgps.satellites = atoi(p);

        p = strchr(p, ',')+1;
        if (',' != *p) newgps.HDOP = atof(p);

        p = strchr(p, ',')+1;
        if (',' != *p) newgps.altitude = atof(p);

        p = strchr(p, ',')+1;
        p = strchr(p, ',')+1;
        if (',' != *p) newgps.geoidheight = atof(p);
        return newgps;
    }
    if (strstr(nmea.c_str(), "$GPRMC"))
    {
        // found RMC
        const char *p = nmea.c_str();

        // get time
        p = strchr(p, ',')+1;
        float timef = atof(p);
        uint32_t time = timef;
        newgps.hour = time / 10000;
        newgps.minute = (time % 10000) / 100;
        newgps.seconds = (time % 100);

        newgps.milliseconds = fmod(timef, 1.0) * 1000;

        p = strchr(p, ',')+1;
        // Serial.println(p);
        if (p[0] == 'A')
        newgps.fix = true;
        else if (p[0] == 'V')
        newgps.fix = false;
        else
        return newgps;

        // parse out latitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            strncpy(degreebuff, p, 2);
            p += 2;
            degreebuff[2] = '\0';
            long degree = atol(degreebuff) * 10000000;
            strncpy(degreebuff, p, 2); // minutes
            p += 3; // skip decimal point
            strncpy(degreebuff + 2, p, 4);
            degreebuff[6] = '\0';
            long minutes = 50 * atol(degreebuff) / 3;
            newgps.latitude_fixed = degree + minutes;
            newgps.latitude = degree / 100000 + minutes * 0.000006F;
            newgps.latitude_degrees = (newgps.latitude-100*int(newgps.latitude/100))/60.0;
            newgps.latitude_degrees += int(newgps.latitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'S') newgps.latitude_degrees *= -1.0;
            if (p[0] == 'N') newgps.lat = 'N';
            else if (p[0] == 'S') newgps.lat = 'S';
            else if (p[0] == ',') newgps.lat = 0;
            else return newgps;
        }

        // parse out longitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            strncpy(degreebuff, p, 3);
            p += 3;
            degreebuff[3] = '\0';
            degree = atol(degreebuff) * 10000000;
            strncpy(degreebuff, p, 2); // minutes
            p += 3; // skip decimal point
            strncpy(degreebuff + 2, p, 4);
            degreebuff[6] = '\0';
            minutes = 50 * atol(degreebuff) / 3;
            newgps.longitude_fixed = degree + minutes;
            newgps.longitude = degree / 100000 + minutes * 0.000006F;
            newgps.longitude_degrees = (newgps.longitude-100*int(newgps.longitude/100))/60.0;
            newgps.longitude_degrees += int(newgps.longitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'W') newgps.longitude_degrees *= -1.0;
            if (p[0] == 'W') newgps.lon = 'W';
            else if (p[0] == 'E') newgps.lon = 'E';
            else if (p[0] == ',') newgps.lon = 0;
            else return newgps;
        }
        // speed
        p = strchr(p, ',')+1;
        if (',' != *p) newgps.speed = atof(p);

        p = strchr(p, ',')+1;
        if (',' != *p) newgps.angle = atof(p);

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            uint32_t fulldate = atof(p);
            newgps.day = fulldate / 10000;
            newgps.month = (fulldate % 10000) / 100;
            newgps.year = (fulldate % 100);
        }
        // we dont parse the remaining, yet!
        return newgps;
    }
    return newgps;
}
