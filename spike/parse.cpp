#include <iostream>
#include <string>
#include <sstream>
#include <array>
#include <vector>

struct utc_time
{
    uint8_t hour, minute, second;
    uint16_t ms;
};

std::ostream& operator << (std::ostream& os, const utc_time& u)
{
    os << "hr: " << (int) u.hour
       << " min: " << (int) u.minute
       << " sec: " << (int) u.second
       << " ms: " << u.ms;
    return os;
}

// contains time, position and fix related data of the GNSS receiver
struct gpgga
{
    utc_time utc;

    uint32_t latitude, longitude;
    char latdir, londir;
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

std::ostream& operator << (std::ostream& os, const gpgga& g)
{
    os << g.utc << std::endl
       << "lat: " << g.latitude << " " << g.latdir
       << " lon: " << g.longitude << " " << g.londir
       << " fix: " << (int) g.fix_quality << std::endl
       << "sats: " << (int) g.num_sats << " hdop: " << g.hdop
       << " alt: " << g.altitude << " " << g.alt_unit
       << " und: " << g.undulation << " " << g.und_unit
       << " dgps: " << g.has_dgps
       << " " << (int) g.corr_age << " s " << g.base_ID;
    return os; 
}

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

std::ostream& operator << (std::ostream& os, const gpgsa& g)
{
    os << "mode: " << g.mode_char << " " << (int) g.mode_num
       << std::endl << "sat prns: [";
    for (int i = 0; i < g.sat_prns.size(); i++)
    {
        os << (int) g.sat_prns[i];
        if (i < g.sat_prns.size() - 1) os << " ";
    }
    os << "]\ndops: " << g.pdop << " " << g.hdop << " "
       << g.vdop;
    return os;
}

// contains time, date, position, track made good and speed data
struct gprmc
{
    utc_time utc;

    char pos_status;
    uint32_t latitude, longitude;
    char latdir, londir;
    double ground_speed;
    double track_angle;

    uint8_t day, month, year;

    double mag_var;
    char var_dir;
    char pos_mode;
};

std::ostream& operator << (std::ostream& os, const gprmc& g)
{
    os << g.utc << std::endl
       << "stat: " << g.pos_status << " pos: "
       << g.latitude << " " << g.latdir << " "
       << g.longitude << " " << g.londir << " "
       << " spd/angl: " << g.ground_speed << " "
       << g.track_angle << std::endl
       << "d/m/y: " << (int) g.day << " " << (int) g.month << " "
       << (int) g.year << std::endl
       << "magv: " << g.mag_var << " " << g.var_dir << " "
       << g.pos_mode;
    return os;
}

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

utc_time parse_utc(const std::string& data)
{
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

uint32_t parse_lat(const std::string& data)
{
    uint32_t degrees = std::stoi(data.substr(0,2));
    double minutes = std::stod(data.substr(3,data.length()));
    return (degrees + minutes/60) * 100000;
}

uint32_t parse_lon(const std::string& data)
{
    uint32_t degrees = std::stoi(data.substr(0,3));
    double minutes = std::stod(data.substr(4,data.length()));
    return (degrees + minutes/60) * 100000;
}

gpgga parse_gpgga(const std::string& data)
{
    std::stringstream ss(data);
    std::string token;

    gpgga ret;
    char ch;
    std::getline(ss, token, ',');
    ret.utc = parse_utc(token);
    std::getline(ss, token, ',');
    ret.latitude = parse_lat(token);
    ss >> ret.latdir;
    ss >> ch;
    std::getline(ss, token, ',');
    ret.longitude = parse_lon(token);
    ss >> ret.londir;
    ss >> ch;
    std::getline(ss, token, ',');
    ret.fix_quality = std::stoi(token);
    std::getline(ss, token, ',');

    ret.num_sats = std::stoi(token);
    ss >> ret.hdop;
    ss >> ch;
    ss >> ret.altitude;
    ss >> ch;
    ss >> ret.alt_unit;
    ss >> ch;
    ss >> ret.undulation;
    ss >> ch;
    ss >> ret.und_unit;
    ss >> ch;

    std::getline(ss, token, ',');
    // no this is not a typo - should be an assignent
    if (ret.has_dgps = token.length())
    {
        ret.corr_age = std::stoi(token);
        std::getline(ss, ret.base_ID, ',');
    }

    return ret;
}

gpgsa parse_gpgsa(const std::string& nmea)
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

gprmc parse_gprmc(const std::string& nmea)
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
    ret.latitude = parse_lat(token);
    ss >> ret.latdir;
    ss >> ch;
    std::getline(ss, token, ',');
    ret.longitude = parse_lon(token);
    ss >> ret.londir;
    ss >> ch;

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

void parse(const std::string& nmea)
{
    std::stringstream ss(nmea);
    std::string header, data, token, checksum;
    std::getline(ss, header, ',');
    std::getline(ss, data, '*');
    std::getline(ss, checksum);
    
    ss.str(data);
    ss.clear();
    
    const char* headers[] = {"$GPGGA", "$GPGSA", "$GPGSV", "$GPRMC", "$GPVTG"};
    int id = -1;
    for (int i = 0; i < 5; i++) if (header == headers[i]) id = i;

    std::cout << "(" << header << ")[" << data << "]" << std::endl;
    if (id == 0) std::cout << parse_gpgga(data) << std::endl;
    if (id == 1) std::cout << parse_gpgsa(data) << std::endl;
    if (id == 3) std::cout << parse_gprmc(data) << std::endl;
}

int main()
{
    auto strings =
        {"$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60",
         "$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,,*60",
         "$GPGSA,M,3,17,02,30,04,05,10,09,06,31,12,,,1.2,0.8,0.9*35",
         "$GPGSV,3,1,11,18,87,050,48,22,56,250,49,21,55,122,49,03,40,284,47*78",
         "$GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,3.24,E,A*20",
         "$GPVTG,172.516,T,155.295,M,0.049,N,0.090,K,D*2B"};

    for (auto e : strings)
    {
        parse(e);
    }
}
