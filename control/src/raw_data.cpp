#include "raw_data.h"

namespace uav
{

archive& operator << (archive& a, const angle& b)
{
    return a << b.micros();
}

archive& operator >> (archive& a, angle& b)
{
    return a >> b.micros();
}

archive& operator << (archive& a, const coordinate& c)
{
    return a << c.latitude() << c.longitude() << c.altitude();
}

archive& operator >> (archive& a, coordinate& c)
{
    return a >> c.latitude() >> c.longitude() >> c.altitude();
}

archive& operator << (archive& a, const imu::Vector<3>& v)
{
    return a << v.x() << v.y() << v.z();
}

archive& operator >> (archive& a, imu::Vector<3>& v)
{
    return a >> v.x() >> v.y() >> v.z();
}

archive& operator << (archive& a, const arduino_data& d)
{
    return a << d.millis << d.euler << d.calib << d.pres << d.acc;
}

archive& operator >> (archive& a, arduino_data& d)
{
    return a >> d.millis >> d.euler >> d.calib >> d.pres >> d.acc;
}

archive& operator << (archive& a, const utc_time& u)
{
    return a << u.hour << u.minute << u.second << u.ms;
}

archive& operator >> (archive& a, utc_time& u)
{
    return a >> u.hour >> u.minute >> u.second >> u.ms;
}

archive& operator << (archive& a, const gpgga& g)
{
    return a << g.utc << g.pos << g.fix_quality << g.num_sats
             << g.hdop << g.alt_unit << g.undulation << g.und_unit
             << g.has_dgps << g.corr_age << g.base_ID;
}

archive& operator >> (archive& a, gpgga& g)
{
    return a >> g.utc >> g.pos >> g.fix_quality >> g.num_sats
             >> g.hdop >> g.alt_unit >> g.undulation >> g.und_unit
             >> g.has_dgps >> g.corr_age >> g.base_ID;
}
    
archive& operator << (archive& a, const gpgsa& g)
{
    return a << g.mode_char << g.mode_num << g.sat_prns
             << g.pdop << g.hdop << g.vdop;
}

archive& operator >> (archive& a, gpgsa& g)
{
    return a >> g.mode_char >> g.mode_num >> g.sat_prns
             >> g.pdop >> g.hdop >> g.vdop;
}

archive& operator << (archive& a, const gprmc& g)
{
    return a << g.utc << g.pos_status << g.pos << g.ground_speed
             << g.track_angle << g.day << g.month << g.year
             << g.mag_var << g.var_dir << g.pos_mode;
}

archive& operator >> (archive& a, gprmc& g)
{
    return a >> g.utc >> g.pos_status >> g.pos >> g.ground_speed
             >> g.track_angle >> g.day >> g.month >> g.year
             >> g.mag_var >> g.var_dir >> g.pos_mode;
}

archive& operator << (archive& a, const gpvtg& g)
{
    return a << g.track_true << g.track_indicator << g.track_mag
             << g.mag_indicator << g.ground_speed_knots
             << g.speed_ind_knots << g.ground_speed_kph
             << g.speed_ind_kph << g.mode_ind;
}

archive& operator >> (archive& a, gpvtg& g)
{
    return a >> g.track_true >> g.track_indicator >> g.track_mag
             >> g.mag_indicator >> g.ground_speed_knots
             >> g.speed_ind_knots >> g.ground_speed_kph
             >> g.speed_ind_kph >> g.mode_ind;
}

archive& operator << (archive& a, const gpgsv::sat_info& g)
{
    return a << g.PRN << g.elevation << g.azimuth << g.SNR;
}

archive& operator >> (archive& a, gpgsv::sat_info& g)
{
    return a >> g.PRN >> g.elevation >> g.azimuth >> g.SNR;
}
    
archive& operator << (archive& a, const gpgsv& g)
{
    return a << g.num_msgs << g.msg_num << g.sats_in_view << g.sats;
}

archive& operator >> (archive& a, gpgsv& g)
{
    return a >> g.num_msgs >> g.msg_num >> g.sats_in_view >> g.sats;
}
    
archive& operator << (archive& a, const gps_data& g)
{
    return a << g.gga << g.gsa << g.rmc << g.vtg << g.gsv;
}

archive& operator >> (archive& a, gps_data& g)
{
    return a >> g.gga >> g.gsa >> g.rmc >> g.vtg >> g.gsv;
}

archive& operator << (archive& a, const bmp085_data& b)
{
    return a << b.temp << b.pressure;
}

archive& operator >> (archive& a, bmp085_data& b)
{
    return a >> b.temp >> b.pressure;
}

archive& operator << (archive& a, const raw_data& r)
{
    return a << r.ard << r.gps << r.bmp;
}

archive& operator >> (archive& a, raw_data& r)
{
    return a >> r.ard >> r.gps >> r.bmp;
}

} // namespace uav
