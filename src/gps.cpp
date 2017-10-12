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
#include <sstream>
#include <vector>

#include <gps.h>
#include <wiringSerial.h>

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

gps::gps() : data{0}, newflag(false), cont(true), status(0), fd(-1) { }

gps::~gps()
{
    cont = false;
    if (reader.joinable()) reader.join();
    serialClose(fd);
}

int gps::begin()
{
    fd = serialOpen("/dev/ttyS0", 9600);
    if (fd < 0)
    {
        std::cerr << "gps: could not open" << std::endl;
        return 1;
    }

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
    std::stringstream message;
    char ch = 0;
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(1);

    while (cont && ch != '$')
    {
        if (start + timeout < std::chrono::steady_clock::now())
        {
            std::cerr << "gps: timeout" << std::endl;
            status = -1;
            return;
        }
        if (serialDataAvail(fd) > 0) ch = serialGetchar(fd);
    }

    status = 1;
    while (cont)
    {
        if (ch == '$')
        {
            std::cout << message.str() << std::endl;
            message.str("");
            message.clear();
        }
        message << ch;
        ch = serialGetchar(fd);
    }
}
