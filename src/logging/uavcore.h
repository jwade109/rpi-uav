#ifndef UAV_CORE_H
#define UAV_CORE_H

#include <fstream>
#include <sstream>
#include <deque>
#include <array>
#include <limits>

namespace uav
{

class angle
{
    public:

    static angle from_degrees(double degs);
    static angle from_radians(double rads);
    static angle from_rotations(int rots);

    angle();
    angle(double rads, int rots = 0);
    angle(const angle& a);

    double rad() const;
    double deg() const;
    int rot() const;

    angle operator - ();
    angle& operator = (const angle& a);
    angle operator + (const angle& a) const;
    angle operator - (const angle& a) const;
    angle& operator += (const angle& a);
    angle& operator -= (const angle& a);

    angle& operator = (double rads);

    angle operator * (double scalar) const;
    angle operator / (double divisor) const;
    angle& operator *= (double scalar);
    angle& operator /= (double divisor);

    double operator / (const angle& divisor) const;

    bool operator == (const angle& a) const;
    bool operator != (const angle& a) const;
    bool operator < (const angle& a) const;
    bool operator > (const angle& a) const;
    bool operator <= (const angle& a) const;
    bool operator >= (const angle& a) const;

    operator double () const;

    private:

    int rotations;
    double radians;
};

angle target_azimuth(angle current, angle desired);

std::ostream& operator << (std::ostream& os, const angle& a);

namespace angle_literals
{

angle operator "" _rad(unsigned long long radians);

angle operator "" _deg(unsigned long long degrees);

angle operator "" _rad(long double radians);

angle operator "" _deg(long double degrees);

} // namespace uav::angle_literals

// takes pressure in atm
double altitude(double atm);

// data representation typedefs ////////////////////////////////////////////

enum : uint8_t
{
    f1hz = 1, f10hz = 10, f20hz = 20, f25hz = 25, f40hz = 40, f50hz = 50,
    f100hz = 100, f125hz = 125, f200hz = 200, f250hz = 250,

    fdefault = f100hz
};

enum : uint8_t
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
        position        = 1 << 3,
        attitude        = 1 << 4,
        config          = position | attitude,
        quat            = 1 << 5,
        targets         = 1 << 6,
        motors          = 1 << 7,
        error           = 1 << 8,
        status          = 1 << 9,

        all = std::numeric_limits<bitmask_t>::max(),
        standard = time | config | motors | status
    };
}

// uav::state //////////////////////////////////////////////////////////////

struct state
{
    std::array<uint64_t, 3> time;
    std::array<double, 3> position;
    std::array<double, 3> attitude;
    std::array<double, 4> targets;
    std::array<double, 4> motors;
    uint16_t error;
    uint8_t status;

    static std::string header(fmt::bitmask_t);
    bool operator==(const state& other);
    bool operator!=(const state& other);

    const static size_t fields = 30;
    const static size_t size = 3*sizeof(uint64_t) +
        14*sizeof(double) + sizeof(uint16_t) + sizeof(uint8_t);

    using bin = std::array<uint8_t, size>;
};

// uav::param //////////////////////////////////////////////////////////////

struct param
{
    uint8_t freq; // frequency of updates in hz
    double mass;
    // pid gains for altitude, heading, pitch, roll
    std::array<double, 20> pid_gains;

    static std::string header();
    bool operator==(const param& other);
    bool operator!=(const param& other);

    const static size_t size = sizeof(uint8_t) + 21*sizeof(double);

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
