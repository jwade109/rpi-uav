/***********************************
This is our GPS library

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/

// modified from original Adafruit library for Raspberry Pi 3
// by Wade Foster on 8/29/2017

#include <cstring>
#include <cmath>
#include <cctype>

#include <gps.h>

const std::string GPS::pmtk_echo_100mHz("$PMTK220,10000*2F");
const std::string GPS::pmtk_echo_200mHz("$PMTK220,5000*1B");
const std::string GPS::pmtk_echo_1Hz("$PMTK220,1000*1F");
const std::string GPS::pmtk_echo_5Hz("$PMTK220,200*2C");
const std::string GPS::pmtk_echo_10Hz("$PMTK220,100*2F");

const std::string GPS::pmtk_fix_100mHz("$PMTK300,10000,0,0,0,0*2C");
const std::string GPS::pmtk_fix_200mHz("$PMTK300,5000,0,0,0,0*18");
const std::string GPS::pmtk_fix_1Hz("$PMTK300,1000,0,0,0,0*1C");
const std::string GPS::pmtk_fix_5Hz("$PMTK300,200,0,0,0,0*2F");

const std::string GPS::pmtk_B57600("$PMTK251,57600*2C");
const std::string GPS::pmtk_B9600("$PMTK251,9600*17");

const std::string GPS::pmtk_nmea_gprmc
    ("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
const std::string GPS::pmtk_nmea_gprmc_gga
    ("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
const std::string GPS::pmtk_nmea_all
    ("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28");

const std::string GPS::pmtk_locus_startlog("$PMTK185,0*22");
const std::string GPS::pmtk_locus_stoplog("$PMTK185,1*23");
const std::string GPS::pmtk_locus_startstopack("$PMTK001,185,3*3C");
const std::string GPS::pmtk_locus_query_status("$PMTK183*38");
const std::string GPS::pmtk_locus_erase_flash("$PMTK184,1*22");

const std::string GPS::pmtk_enable_sbas("$PMTK313,1*2E");
const std::string GPS::pmtk_enable_waas("$PMTK301,2*2E");

const std::string GPS::pmtk_standby("$PMTK161,0*28");
const std::string GPS::pmtk_standby_success("$PMTK001,161,3*36");
const std::string GPS::pmtk_awake("$PMTK010,002*2D");

const std::string GPS::pmtk_query_release("$PMTK605*31");

bool GPS::parse(char *nmea)
{
    // do checksum check

    // first look if we even have one
    if (nmea[strlen(nmea)-4] == '*')
    {
        uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
        sum += parseHex(nmea[strlen(nmea)-2]);

        // check checksum 
        for (uint8_t i=2; i < (strlen(nmea)-4); i++)
        {
            sum ^= nmea[i];
        }
        // bad checksum
        if (sum != 0) return false;
    }
    int32_t degree;
    long minutes;
    char degreebuff[10];
    // look for a few common sentences
    if (strstr(nmea, "$GPGGA"))
    {
        // found GGA
        char *p = nmea;
        // get time
        p = strchr(p, ',')+1;
        float timef = atof(p);
        uint32_t time = timef;
        data.hour = time / 10000;
        data.minute = (time % 10000) / 100;
        data.seconds = (time % 100);
        data.milliseconds = fmod(timef, 1.0) * 1000;

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
            data.latitude_fixed = degree + minutes;
            data.latitude = degree / 100000 + minutes * 0.000006F;
            data.latitude_degrees = (data.latitude-100*int(data.latitude/100))/60.0;
            data.latitude_degrees += int(data.latitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'S') data.latitude_degrees *= -1.0;
            if (p[0] == 'N') data.lat = 'N';
            else if (p[0] == 'S') data.lat = 'S';
            else if (p[0] == ',') data.lat = 0;
            else return false;
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
            data.longitude_fixed = degree + minutes;
            data.longitude = degree / 100000 + minutes * 0.000006F;
            data.longitude_degrees = (data.longitude-100*int(data.longitude/100))/60.0;
            data.longitude_degrees += int(data.longitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'W') data.longitude_degrees *= -1.0;
            if (p[0] == 'W') data.lon = 'W';
            else if (p[0] == 'E') data.lon = 'E';
            else if (p[0] == ',') data.lon = 0;
            else return false;
        }

        p = strchr(p, ',')+1;
        if (',' != *p) data.fixquality = atoi(p);

        p = strchr(p, ',')+1;
        if (',' != *p) data.satellites = atoi(p);

        p = strchr(p, ',')+1;
        if (',' != *p) data.HDOP = atof(p);

        p = strchr(p, ',')+1;
        if (',' != *p) data.altitude = atof(p);

        p = strchr(p, ',')+1;
        p = strchr(p, ',')+1;
        if (',' != *p) data.geoidheight = atof(p);
        return true;
    }
    if (strstr(nmea, "$GPRMC"))
    {
        // found RMC
        char *p = nmea;

        // get time
        p = strchr(p, ',')+1;
        float timef = atof(p);
        uint32_t time = timef;
        data.hour = time / 10000;
        data.minute = (time % 10000) / 100;
        data.seconds = (time % 100);

        data.milliseconds = fmod(timef, 1.0) * 1000;

        p = strchr(p, ',')+1;
        // Serial.println(p);
        if (p[0] == 'A') 
        data.fix = true;
        else if (p[0] == 'V')
        data.fix = false;
        else
        return false;

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
            data.latitude_fixed = degree + minutes;
            data.latitude = degree / 100000 + minutes * 0.000006F;
            data.latitude_degrees = (data.latitude-100*int(data.latitude/100))/60.0;
            data.latitude_degrees += int(data.latitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'S') data.latitude_degrees *= -1.0;
            if (p[0] == 'N') data.lat = 'N';
            else if (p[0] == 'S') data.lat = 'S';
            else if (p[0] == ',') data.lat = 0;
            else return false;
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
            data.longitude_fixed = degree + minutes;
            data.longitude = degree / 100000 + minutes * 0.000006F;
            data.longitude_degrees = (data.longitude-100*int(data.longitude/100))/60.0;
            data.longitude_degrees += int(data.longitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            if (p[0] == 'W') data.longitude_degrees *= -1.0;
            if (p[0] == 'W') data.lon = 'W';
            else if (p[0] == 'E') data.lon = 'E';
            else if (p[0] == ',') data.lon = 0;
            else return false;
        }
        // speed
        p = strchr(p, ',')+1;
        if (',' != *p) data.speed = atof(p);

        p = strchr(p, ',')+1;
        if (',' != *p) data.angle = atof(p);

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
            uint32_t fulldate = atof(p);
            data.day = fulldate / 10000;
            data.month = (fulldate % 10000) / 100;
            data.year = (fulldate % 100);
        }
        // we dont parse the remaining, yet!
        return true;
    }
    return false;
}

GPS::GPS() : paused(false), asleep(false)
{
    
}

GPS::~GPS()
{
    cont = false;
    if (reader.joinable()) reader.join();
}

void GPS::begin()
{
    // this used to begin the SW/HW serial interfaces
}

void GPS::sendCommand(const char *str)
{
    // unsure of how to implement something like this:
    // gpsSwSerial->println(str);
}

void GPS::pause(bool p)
{
    paused = p;
}

// read a Hex value and return the decimal equivalent
uint8_t GPS::parseHex(char c)
{
    if (c < '0') return 0;
    if (c <= '9') return c - '0';
    if (c < 'A') return 0;
    if (c <= 'F') return (c - 'A') + 10;
    return 0;
}

bool GPS::waitForSentence(const char *wait4me, uint8_t max)
{
    char str[20];
    uint8_t i=0;
    while (i < max)
    {
        if (newNMEAreceived())
        { 
            char *nmea = lastNMEA();
            strncpy(str, nmea, 20);
            str[19] = 0;
            i++;
            if (strstr(str, wait4me)) return true;
        }
    }
    return false;
}

bool GPS::LOCUS_StartLogger(void)
{
    sendCommand(pmtk_locus_startlog.c_str());
    // recvdflag = false;
    return waitForSentence(pmtk_locus_startstopack.c_str());
}

bool GPS::LOCUS_StopLogger(void)
{
    sendCommand(pmtk_locus_startlog.c_str());
    // recvdflag = false;
    return waitForSentence(pmtk_locus_startstopack.c_str());
}

bool GPS::LOCUS_ReadStatus(void)
{
    sendCommand(pmtk_locus_query_status.c_str());
  
    if (!waitForSentence("$PMTKLOG")) return false;

    char *response = lastNMEA();
    uint16_t parsed[10];
    uint8_t i;
  
    for (i=0; i<10; i++) parsed[i] = -1;
  
    response = strchr(response, ',');
    for (i=0; i<10; i++)
    {
        if (!response || (response[0] == 0) || (response[0] == '*')) 
            break;
        response++;
        parsed[i]=0;
        while ((response[0] != ',') && 
	        (response[0] != '*') && (response[0] != 0))
        {
            parsed[i] *= 10;
            char c = response[0];

            if (isdigit(c)) parsed[i] += c - '0';
            else parsed[i] = c;
            response++;
        }
    }
    linfo.serial = parsed[0];
    linfo.type = parsed[1];
    if (isalpha(parsed[2]))
    {
        parsed[2] = parsed[2] - 'a' + 10; 
    }
    linfo.mode = parsed[2];
    linfo.config = parsed[3];
    linfo.interval = parsed[4];
    linfo.distance = parsed[5];
    linfo.speed = parsed[6];
    linfo.status = !parsed[7];
    linfo.records = parsed[8];
    linfo.percent = parsed[9];
    return true;
}

// Standby Mode Switches
bool GPS::standby(void)
{
    if (asleep)
    {
        // Returns false if already in standby mode, so that you
        // do not wake it up by sending commands to GPS
        return false;
    }
    else
    {
        asleep = true;
        sendCommand(pmtk_standby.c_str());
        return true;
    }
}

bool GPS::wakeup(void)
{
    if (asleep)
    {
        asleep = false;
        sendCommand(""); // send byte to wake it up
        return waitForSentence(pmtk_awake.c_str());
    }
    else
    {
        // Returns false if not in standby mode, nothing to wakeup
        return false; 
    }
}
