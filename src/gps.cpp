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
#include <iomanip>
#include <sstream>
#include <vector>

#include <gps.h>
#include <wiringSerial.h>

const std::string uav::gps::pmtk_echo_100mHz("$PMTK220,10000*2F");
const std::string uav::gps::pmtk_echo_200mHz("$PMTK220,5000*1B");
const std::string uav::gps::pmtk_echo_1Hz("$PMTK220,1000*1F");
const std::string uav::gps::pmtk_echo_5Hz("$PMTK220,200*2C");
const std::string uav::gps::pmtk_echo_10Hz("$PMTK220,100*2F");

const std::string uav::gps::pmtk_fix_100mHz("$PMTK300,10000,0,0,0,0*2C");
const std::string uav::gps::pmtk_fix_200mHz("$PMTK300,5000,0,0,0,0*18");
const std::string uav::gps::pmtk_fix_1Hz("$PMTK300,1000,0,0,0,0*1C");
const std::string uav::gps::pmtk_fix_5Hz("$PMTK300,200,0,0,0,0*2F");

const std::string uav::gps::pmtk_B57600("$PMTK251,57600*2C");
const std::string uav::gps::pmtk_B9600("$PMTK251,9600*17");

const std::string uav::gps::pmtk_nmea_gprmc
    ("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
const std::string uav::gps::pmtk_nmea_gprmc_gga
    ("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
const std::string uav::gps::pmtk_nmea_all
    ("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28");

const std::string uav::gps::pmtk_locus_startlog("$PMTK185,0*22");
const std::string uav::gps::pmtk_locus_stoplog("$PMTK185,1*23");
const std::string uav::gps::pmtk_locus_startstopack("$PMTK001,185,3*3C");
const std::string uav::gps::pmtk_locus_query_status("$PMTK183*38");
const std::string uav::gps::pmtk_locus_erase_flash("$PMTK184,1*22");

const std::string uav::gps::pmtk_enable_sbas("$PMTK313,1*2E");
const std::string uav::gps::pmtk_enable_waas("$PMTK301,2*2E");

const std::string uav::gps::pmtk_standby("$PMTK161,0*28");
const std::string uav::gps::pmtk_standby_success("$PMTK001,161,3*36");
const std::string uav::gps::pmtk_awake("$PMTK010,002*2D");

const std::string uav::gps::pmtk_query_release("$PMTK605*31");

uav::gps::gps() : data{0}, newflag(false), cont(false), status(0), fd(-1) { }

uav::gps::~gps()
{
    cont = false;
    if (reader.joinable()) reader.join();
    serialClose(fd);
}

int uav::gps::begin()
{
    if (cont) return 0;

    fd = serialOpen("/dev/ttyS0", 9600);
    if (fd < 0)
    {
        std::cerr << "gps: failed to open /dev/ttyS0" << std::endl;
        return 1;
    }

    cont = true;
    reader = std::thread(&uav::gps::dowork, this);

    while (status == 0);
    if (status == -1)
    {
        cont = false;
        if (reader.joinable()) reader.join();
        std::cerr << "gps: connection timed out" << std::endl;
        return 2;
    }

    return 0;
}

bool uav::gps::isnew() const
{
    return newflag;
}

uav::gps_data uav::gps::get()
{
    newflag = false;
    return data;
}

bool uav::gps::update(gps_data& gp)
{
    if (newflag) gp = data;
    bool changed = newflag;
    newflag = false;
    return changed;
}

void uav::gps::dowork()
{
    std::stringstream message;
    char ch = 0;
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(1);

    while (cont && ch != '$')
    {
        if (start + timeout < std::chrono::steady_clock::now())
        {
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
            update_info(message);
            message.str("");
            message.clear();
        }
        message << ch;
        ch = serialGetchar(fd);
    }
}

void uav::gps::update_info(std::stringstream& nmea)
{
    const char* head[] =
        {"$GPGGA", "$GPGSA", "$GPVTG", "$GPRMC", "$GPGSV"};
    std::string header, msg;
    std::getline(nmea, header, ',');
    std::getline(nmea, msg, '*');
    int id = -1;
    for (int i = 0; i < 5 && id < 0; i++)
        if (header == head[i]) id = i;
    switch (id)
    {
        case 0: data.gga = parse_gpgga(msg); break;
        case 1: data.gsa = parse_gpgsa(msg); break;
        case 2: data.vtg = parse_gpvtg(msg); break;
        case 3: data.rmc = parse_gprmc(msg); break;
        case 4: auto gsv = parse_gpgsv(msg);
            if (gsv.msg_num == 1) data.gsv.clear();
            data.gsv.push_back(gsv);
            break;
    }
    newflag |= (id > -1);
}

std::ostream& uav::operator << (std::ostream& os, const uav::utc_time& u)
{
    os << (int) (u.hour > 3 ? u.hour - 4 : 23 - u.hour)
       << ":" << std::setw(2) << std::setfill('0') << (int) u.minute
       << ":" << std::setw(2) << std::setfill('0') << (int) u.second
       << "." << std::setw(3) << std::setfill('0') << u.ms << " EST";
    return os;
}

std::ostream& uav::operator << (std::ostream& os, const uav::angular& a)
{
    os << (a.sign ? "" : "-")
       << std::setw(2) << std::setfill('0') << (int) a.degrees << "°"
       << std::setw(2) << std::setfill('0') << (int) a.minutes << "'"
       << std::setw(2) << std::setfill('0') << (int) a.seconds << "."
       << std::setw(3) << std::setfill('0') << a.milliseconds << "\"";
    return os;
}

std::ostream& uav::operator << (std::ostream& os, const uav::gpgga& g)
{
    os << "[GPGGA] ";
    os << g.utc << std::endl
       << "lat: " << g.pos.lat << " "
       << " lon: " << g.pos.lon << " "
       << " fix: " << (int) g.fix_quality << std::endl
       << "sats: " << (int) g.num_sats << " hdop: " << g.hdop
       << " alt: " << g.altitude << " " << g.alt_unit
       << " und: " << g.undulation << " " << g.und_unit
       << " dgps: " << g.has_dgps
       << " " << (int) g.corr_age << " s " << g.base_ID;
    return os;
}

std::ostream& uav::operator << (std::ostream& os, const uav::gpgsa& g)
{
    os << "[GPGSA] ";
    os << "mode: " << g.mode_char << " " << (int) g.mode_num
       << std::endl << "sat prns: [";
    for (unsigned i = 0; i < g.sat_prns.size(); i++)
    {
        os << (int) g.sat_prns[i];
        if (i < g.sat_prns.size() - 1) os << " ";
    }
    os << "]\ndops: " << g.pdop << " " << g.hdop << " "
       << g.vdop;
    return os;
}

std::ostream& uav::operator << (std::ostream& os, const uav::gprmc& g)
{
    os << "[GPRMC] ";
    os << g.utc << std::endl
       << "stat: " << g.pos_status
       << " pos: " << g.pos.lat << " " << g.pos.lon
       << " spd/angl: " << g.ground_speed << " "
       << g.track_angle << std::endl
       << "d/m/y: " << (int) g.day << " " << (int) g.month << " "
       << (int) g.year << std::endl
       << "magv: " << g.mag_var << " " << g.var_dir << " "
       << g.pos_mode;
    return os;
}

std::ostream& uav::operator << (std::ostream& os, const uav::gpgsv::sat_info& s)
{
    os << "prn: " << (int) s.PRN << " elev: " << (int) s.elevation
       << " az: " << s.azimuth << " snr: " << (int) s.SNR;
    return os;
}

std::ostream& uav::operator << (std::ostream& os, const uav::gpgsv& g)
{
    os << "[GPGSV] ";
    os << "msg " << (int) g.msg_num << "/" << (int) g.num_msgs
       << " sats: " << (int) g.sats_in_view << std::endl;
    for (unsigned i = 0; i < g.sats.size(); i++)
    {
        os << "[" << g.sats[i] << "]";
        if (i < g.sats.size() - 1) os << std::endl;
    }
    return os;
}

std::ostream& uav::operator << (std::ostream& os, const uav::gpvtg& g)
{
    os << "[GPVTG] ";
    os << "track: " << g.track_true << " " << g.track_indicator
       << " trk mag: " << g.track_mag << " " << g.mag_indicator
       << " knots: " << g.ground_speed_knots << " " << g.speed_ind_knots
       << " kph: " << g.ground_speed_kph << " " << g.speed_ind_kph
       << " " << g.mode_ind;
    return os;
}

int32_t uav::angular::fixed() const
{
    int32_t ms = milliseconds + seconds*1000 + minutes*60*1000 +
        degrees*60*60*1000;
    return sign ? ms : -ms;
}

imu::Vector<2> uav::operator - (const uav::coordinate& l,
    const uav::coordinate& r)
{
    // 30.92 millimeters in a milliarcsecond
    double dx = (l.lon.fixed() - r.lon.fixed()) * 0.03092;
    double dy = (l.lat.fixed() - r.lat.fixed()) * 0.03092;
    return {dx, dy};
}

uav::utc_time uav::parse_utc(const std::string& data)
{
    if (data.empty()) return utc_time{0};
    std::stringstream ss(data);
    double num;
    ss >> num;
    utc_time u;
    u.hour = num/10000;
    num -= u.hour * 10000;
    u.minute = num/100;
    num -= u.minute * 100;
    u.second = num;
    num -= u.second;
    u.ms = num * 1000;
    return u;
}

uav::angular uav::parse_lat(const std::string& data)
{
    angular a{0};
    if (data.empty()) return a;
    a.degrees = std::stoi(data.substr(0,2));
    a.minutes = std::stoi(data.substr(2,2));
    int decimal_minutes = std::stoi(data.substr(5,4));
    int total_millis = decimal_minutes * 6;
    a.seconds = total_millis / 1000;
    a.milliseconds = total_millis % 1000;
    return a;
}

uav::angular uav::parse_lon(const std::string& data)
{
    angular a{0};
    if (data.empty()) return a;
    a.degrees = std::stoi(data.substr(0,3));
    a.minutes = std::stoi(data.substr(3,2));
    int decimal_minutes = std::stoi(data.substr(6,4));
    int total_millis = decimal_minutes * 6;
    a.seconds = total_millis / 1000;
    a.milliseconds = total_millis % 1000;
    return a;
}

uav::gpgga uav::parse_gpgga(const std::string& data)
{
    std::stringstream ss(data);
    std::string token;

    gpgga ret;
    std::getline(ss, token, ',');
    ret.utc = parse_utc(token);
    std::getline(ss, token, ',');
    ret.pos.lat = parse_lat(token);
    std::getline(ss, token, ',');
    ret.pos.lat.sign = token == "N";
    std::getline(ss, token, ',');
    ret.pos.lon = parse_lon(token);
    std::getline(ss, token, ',');
    ret.pos.lon.sign = token == "E";
    std::getline(ss, token, ',');
    ret.fix_quality = token.empty() ? 0 : std::stoi(token);
    std::getline(ss, token, ',');
    ret.num_sats = token.empty() ? 0 : std::stoi(token);
    std::getline(ss, token, ',');
    ret.hdop = token.empty() ? 0 : std::stof(token);
    std::getline(ss, token, ',');
    ret.altitude = token.empty() ? 0 : std::stof(token);
    std::getline(ss, token, ',');
    ret.alt_unit = token.empty() ? '?' : token[0];
    std::getline(ss, token, ',');
    ret.undulation = token.empty() ? 0 : std::stof(token);
    std::getline(ss, token, ',');
    ret.und_unit = token.empty() ? '?' : token[0];

    std::getline(ss, token, ',');
    // no this is not a typo - should be an assignent
    if ((ret.has_dgps = token.length()))
    {
        ret.corr_age = token.empty() ? 0 : std::stoi(token);
        std::getline(ss, ret.base_ID, ',');
    }
    else
    {
        ret.corr_age = 0;
        ret.base_ID = "";
    }

    return ret;
}

uav::gpgsa uav::parse_gpgsa(const std::string& nmea)
{
    gpgsa ret;
    char ch;
    std::string token;
    std::stringstream ss(nmea);
    ss >> ret.mode_char;
    ss >> ch;
    std::getline(ss, token, ',');
    ret.mode_num = std::stoi(token);
    for (int i = 0; i < 12; i++)
    {
        std::getline(ss, token, ',');
        if (token.length())
            ret.sat_prns.push_back(std::stoi(token));
    }
    ss >> ret.pdop;
    ss >> ch;
    ss >> ret.hdop;
    ss >> ch;
    ss >> ret.vdop;
    return ret;
}

uav::gprmc uav::parse_gprmc(const std::string& nmea)
{
    gprmc ret;
    std::string token;
    std::stringstream ss(nmea);
    char ch;

    std::getline(ss, token, ',');
    ret.utc = parse_utc(token);
    ss >> ret.pos_status;
    ss >> ch;
    std::getline(ss, token, ',');
    ret.pos.lat = parse_lat(token);
    std::getline(ss, token, ',');
    ret.pos.lat.sign = token == "N";
    std::getline(ss, token, ',');
    ret.pos.lon = parse_lon(token);
    std::getline(ss, token, ',');
    ret.pos.lon.sign = token == "E";

    ss >> ret.ground_speed;
    ss >> ch;
    ss >> ret.track_angle;
    ss >> ch;
    std::getline(ss, token, ',');
    ret.day = std::stoi(token.substr(0,2));
    ret.month = std::stoi(token.substr(2,2));
    ret.year = std::stoi(token.substr(4,2));

    ss >> ret.mag_var;
    ss >> ch;
    ss >> ret.var_dir;
    ss >> ch;
    ss >> ret.pos_mode;

    return ret;
}

uav::gpgsv uav::parse_gpgsv(const std::string& nmea)
{
    gpgsv ret;
    std::stringstream ss(nmea);
    std::string token;
    std::getline(ss, token, ',');
    ret.num_msgs = std::stoi(token);
    std::getline(ss, token, ',');
    ret.msg_num = std::stoi(token);
    std::getline(ss, token, ',');
    ret.sats_in_view = std::stoi(token);

    while (std::getline(ss, token, ','))
    {
        gpgsv::sat_info s;
        s.PRN = std::stoi(token);
        std::getline(ss, token, ',');
        s.elevation = token.empty() ? 0 : std::stoi(token);
        std::getline(ss, token, ',');
        s.azimuth = token.empty() ? 0 : std::stoi(token);
        std::getline(ss, token, ',');
        s.SNR = token.empty() ? 0 : std::stoi(token);
        ret.sats.push_back(s);
    }
    return ret;
}

uav::gpvtg uav::parse_gpvtg(const std::string& nmea)
{
    std::stringstream ss(nmea);
    std::string token;
    gpvtg ret;

    std::getline(ss, token, ',');
    ret.track_true = token.empty() ? 0 : std::stod(token);
    std::getline(ss, token, ',');
    ret.track_indicator = token.empty() ? '!' : token[0];
    std::getline(ss, token, ',');
    ret.track_mag = token.empty() ? 0 : std::stod(token);
    std::getline(ss, token, ',');
    ret.mag_indicator = token.empty() ? '!' : token[0];

    std::getline(ss, token, ',');
    ret.ground_speed_knots = token.empty() ? 0 : std::stod(token);
    std::getline(ss, token, ',');
    ret.speed_ind_knots = token.empty() ? '!' : token[0];
    std::getline(ss, token, ',');
    ret.ground_speed_kph = token.empty() ? 0 : std::stod(token);
    std::getline(ss, token, ',');
    ret.speed_ind_kph = token.empty() ? '!' : token[0];
    std::getline(ss, token, ',');
    ret.mode_ind = token.empty() ? '!' : token[0];

    return ret;
}
