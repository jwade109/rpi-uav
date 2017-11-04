#ifndef RAW_DATA_H
#define RAW_DATA_H

#include <uav/math>
#include <uav/logging>
#include <uav/hardware>

namespace uav
{

archive& operator << (archive& a, const angle& b);

archive& operator >> (archive& a, angle& b);

archive& operator << (archive& a, const coordinate& c);

archive& operator >> (archive& a, coordinate& c);

archive& operator << (archive& a, const imu::Vector<3>& v);

archive& operator >> (archive& a, imu::Vector<3>& v);

archive& operator << (archive& a, const arduino_data& d);

archive& operator >> (archive& a, arduino_data& d);

archive& operator << (archive& a, const utc_time& u);

archive& operator >> (archive& a, utc_time& g);

archive& operator << (archive& a, const gpgga& g);

archive& operator >> (archive& a, gpgga& g);

archive& operator << (archive& a, const gpgsa& g);

archive& operator >> (archive& a, gpgsa& g);

archive& operator << (archive& a, const gprmc& g);

archive& operator >> (archive& a, gprmc& g);

archive& operator << (archive& a, const gpvtg& g);

archive& operator >> (archive& a, gpvtg& g);

archive& operator << (archive& a, const gpgsv::sat_info& g);

archive& operator >> (archive& a, gpgsv::sat_info& g);

archive& operator << (archive& a, const gpgsv& g);

archive& operator >> (archive& a, gpgsv& g);

archive& operator << (archive& a, const gps_data& g);

archive& operator >> (archive& a, gps_data& g);

archive& operator << (archive& a, const bmp085_data& b);

archive& operator >> (archive& a, bmp085_data& b);

archive& operator << (archive& a, const raw_data& r);

archive& operator >> (archive& a, raw_data& r);

}

#endif // RAW_DATA_H
