#include "coordinate.h"

#include <iomanip>
#include <sstream>

namespace uav
{

coordinate::coordinate() : _latitude(0), _longitude(0), _altitude(0) { }

coordinate::coordinate(const coordinate& c) :
    coordinate(c.latitude(), c.longitude(), c.altitude()) { }

coordinate::coordinate(const angle& N, const angle& E, double alt) :
    _latitude(N), _longitude(E), _altitude(alt) { }

coordinate::coordinate(double dN, double dE, double alt) :
    coordinate(dN, 0, 0, 0, dE, 0, 0, 0, alt) { }

const angle& coordinate::latitude() const
{
    return _latitude;
}

angle& coordinate::latitude()
{
    return _latitude;
}

const angle& coordinate::longitude() const
{
    return _longitude;
}

angle& coordinate::longitude()
{
    return _longitude;
}

double coordinate::altitude() const
{
    return _altitude;
}

double& coordinate::altitude()
{
    return _altitude;
}

coordinate& coordinate::operator = (const coordinate& c)
{
    _latitude = c.latitude();
    _longitude = c.longitude();
    _altitude = c.altitude();
    return *this;
}

coordinate coordinate::operator + (const coordinate& c) const
{
    return coordinate(_latitude + c.latitude(),
                      _longitude + c.longitude(),
                      _altitude + c.altitude());
}

coordinate coordinate::operator - (const coordinate& c) const
{
    return coordinate(_latitude - c.latitude(),
                      _longitude - c.longitude(),
                      _altitude - c.altitude());
}

coordinate& coordinate::operator += (const coordinate& c)
{
    return *this = *this + c;
}

coordinate& coordinate::operator -= (const coordinate& c)
{
    return *this = *this - c;
}

bool coordinate::operator == (const coordinate& c) const
{
    return _latitude == c.latitude() &&
           _longitude == c.longitude() &&
           _altitude == c.altitude();
}

std::ostream& operator << (std::ostream& os, const coordinate& c)
{
    uint64_t uN = std::abs(c.latitude().micros()),
             uE = std::abs(c.longitude().micros());
    
    int dN = uN/angle::us_deg;
    int dE = uE/angle::us_deg;

    int mN = uN/angle::us_min - dN * 60;
    int mE = uE/angle::us_min - dE * 60;

    int sN = uN/angle::us_sec - (dN * 3600 + mN * 60);
    int sE = uE/angle::us_sec - (dE * 3600 + mE * 60);

    uN -= (dN * angle::us_deg + mN * angle::us_min + sN * angle::us_sec);
    uE -= (dE * angle::us_deg + mE * angle::us_min + sE * angle::us_sec);

    char cN = c.latitude() >= angle(0) ? 'N' : 'S';
    char cE = c.longitude() >= angle(0) ? 'E' : 'W';

    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << dN << "*"
       << std::setw(2) << std::setfill('0') << mN << "'"
       << std::setw(2) << std::setfill('0') << sN << "."
       << std::setw(6) << std::setfill('0') << uN << " " << cN << " "
       << std::setw(2) << std::setfill('0') << dE << "*"
       << std::setw(2) << std::setfill('0') << mE << "'"
       << std::setw(2) << std::setfill('0') << sE << "."
       << std::setw(6) << std::setfill('0') << uE << " " << cE << " "
       << std::fixed << std::setprecision(2) << c.altitude();
    return os << ss.str();
}

} // namespace uav
