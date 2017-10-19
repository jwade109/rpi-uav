#ifndef UAV_CORE_H
#define UAV_CORE_H

#include <fstream>
#include <sstream>
#include <deque>
#include <array>
#include <limits>

namespace uav
{

// data representation typedefs ////////////////////////////////////////////

using timestamp_t   = uint64_t;
using pres_t        = float;
using temp_t        = float;
using pos_t         = float;
using euler_t       = float;
using calib_t       = uint8_t;
using target_t      = float;
using pid_ov_t      = float;
using motor_t       = float;
using error_t       = uint16_t;
using status_t      = uint8_t;

using freq_t        = uint8_t;
using home_pres_t   = double;
using pid_gain_t    = double;
using lpf_tau_t     = double;
using wavg_t        = double;
using mrate_t       = uint16_t;
using wgt_frac_t    = double;

enum : freq_t
{
    f1hz = 1, f10hz = 10, f20hz = 20, f25hz = 25, f40hz = 40, f50hz = 50,
    f100hz = 100, f125hz = 125, f200hz = 200, f250hz = 250,

    fdefault = f100hz
};

enum : status_t
{
    null_status, align, no_vel, pos_seek, pos_hold, high_tilt, upside_down
};

// formatting bitmasks /////////////////////////////////////////////////////

namespace fmt
{
    using bitmask_t = uint16_t;

    enum : bitmask_t
    {
        time            = 1,
        time_full       = (1 << 1) | time,
        pressure        = 1 << 2,
        position        = 1 << 3,
        attitude        = 1 << 4,
        config          = position | attitude,
        calib           = 1 << 5,
        quat            = 1 << 6,
        targets         = 1 << 7,
        pid             = 1 << 8,
        motors          = 1 << 9,
        error           = 1 << 10,
        status          = 1 << 11,

        all = std::numeric_limits<bitmask_t>::max(),
        standard = time | config | motors | status
    };
}

// uav::state //////////////////////////////////////////////////////////////

struct state
{
    timestamp_t     t, t_abs, comptime; // time in millis, computation time
    std::array<pres_t, 2> pres;         // pressure from imu/bmp

    std::array<pos_t, 6> pos;
    calib_t         calib;              // calibration status
    std::array<target_t, 6> targets;    // targets for 6 degrees of freedom
    std::array<pid_ov_t, 6> pidov;      // respective pid response
    std::array<motor_t, 4> motors;
    error_t         err;                // bitmask for storing error codes
    uint8_t         status;

    static std::string header(fmt::bitmask_t);
    bool operator==(const state& other);
    bool operator!=(const state& other);

    const static size_t fields = 30;
    const static size_t size = 3 * sizeof(timestamp_t) + 2 * sizeof(pres_t) +
        3 * sizeof(pos_t) + 3 * sizeof(euler_t) +
        sizeof(calib_t) + 6 * sizeof(target_t) + 6 * sizeof(pid_ov_t) +
        4 * sizeof(motor_t) + sizeof(error_t) + sizeof(uint8_t);

    using bin = std::array<uint8_t, size>;
};

// uav::param //////////////////////////////////////////////////////////////

struct param
{
    freq_t          freq;               // frequency of updates in hz

    // pid gains for altitude, heading, pitch, roll
    std::array<pid_gain_t, 4> spidg, zpidg, hpidg, ppidg, rpidg;

    pos_t           tilt95;
    pos_t           maxtilt;
    wgt_frac_t      mg;                 // vehicle weight/max thrust * 100

    static std::string header();
    bool operator==(const param& other);
    bool operator!=(const param& other);

    const static size_t fields = 24;
    const static size_t size = sizeof(freq_t) + 20 * sizeof(pid_gain_t) +
        sizeof(pos_t) * 2 + sizeof(wgt_frac_t);

    using bin = std::array<uint8_t, size>;
};

// conversion functions ////////////////////////////////////////////////////

param::bin serialize(const param& p);

state::bin serialize(const state& s);

param deserialize(const param::bin& b);

state deserialize(const state::bin& b);

template <size_t N, typename T> std::array<uint8_t, N> wrap(T *ptr)
{
    std::array<uint8_t, N> bin;
    uint8_t* src = reinterpret_cast<uint8_t*>(ptr);
    std::copy(src, src + N, begin(bin));
    return bin;
}

template <typename T> std::array<uint8_t, sizeof(T)> bin(T var)
{
    uint8_t* b = reinterpret_cast<uint8_t*>(&var);
    std::array<uint8_t, sizeof(T)> array;
    std::copy(b, b + sizeof(T), begin(array));
    return array;
}

template <typename T, size_t N>
std::array<uint8_t, sizeof(T) * N> bin(const std::array<T, N>& vars)
{
    std::array<uint8_t, sizeof(T) * N> array;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(vars.data());
    std::copy(src, src + sizeof(T) * N, begin(array));
    return array;
}

template <typename T>
void bin(const uint8_t* src, size_t& rptr, T& dest)
{
    dest = *reinterpret_cast<const T*>(src + rptr);
    rptr += sizeof(T);
}

template <size_t N, size_t M> std::array<uint8_t, N + M>
operator + (const std::array<uint8_t, N>& a, const std::array<uint8_t, M>& b)
{
    std::array<uint8_t, N + M> c;
    std::copy(begin(a), end(a), begin(c));
    std::copy(begin(b), end(b), begin(c) + N);
    return c;
}

template <size_t N> bool
operator == (const std::array<uint8_t, N>& a, const std::array<uint8_t, N>& b)
{
    for (size_t i = 0; i < N; i++)
        if (a[i] != b[i]) return false;
    return true;
}

// std::string functions ///////////////////////////////////////////////////

std::string to_string(const param& prm);

std::string to_string(const state& it, fmt::bitmask_t mask);

std::string timestamp();

// logging functions ///////////////////////////////////////////////////////

void reset();

void include(param p);

void include(state s);

void flush();

class logstream
{
    std::stringstream ss;
    const std::string name;

    public:

    logstream(const std::string& name);

    template <typename T>
    logstream& operator << (const T& t)
    {
        ss << t;
        return *this;
    }

    void operator () (const std::string& s);

    logstream& operator << (std::ostream& (*)(std::ostream&));
};

extern logstream debug, info, error;

} // namespace uav

#endif // UAV_CORE_H
