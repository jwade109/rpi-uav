#ifndef UAV_CORE_H
#define UAV_CORE_H

#include <fstream>
#include <deque>
#include <array>
#include <limits>

namespace uav
{
    // data representation typedefs ////////////////////////////////////////////

    using timestamp_t   = uint64_t;
    using pres_t        = float;
    using temp_t        = float;
    using alt_t         = float;
    using attitude_t    = float;
    using calib_t       = uint8_t;
    using target_t      = int16_t;
    using pid_ov_t      = float;
    using motor_t       = uint8_t;
    using error_t       = uint16_t;

    using freq_t        = uint8_t;
    using home_pres_t   = double;
    using pid_gain_t    = double;
    using lpf_tau_t     = double;
    using wavg_t        = double;
    using mrate_t       = uint16_t;
    using wgt_frac_t    = double;

    using byte = unsigned char;

    enum : freq_t
    {
        f1hz = 1, f10hz = 10, f20hz = 20, f25hz = 25, f40hz = 40, f50hz = 50,
        f100hz = 100, f125hz = 125, f200hz = 200, f250hz = 250,

        fdefault = f100hz
    };

    // formatting bitmasks /////////////////////////////////////////////////////

    namespace fmt
    {
        using bitmask_t = uint32_t;

        enum : bitmask_t
        {
            time            = 1,
            time_full       = 7,
            pressure        = 3 << 3,
            temperature     = 3 << 5,
            altitude        = 1 << 7,
            attitude        = 7 << 8,
            attitude_full   = 15 << 8,
            targets         = 15 << 12,
            pid             = 15 << 16,
            motors          = 15 << 20,
            error           = 1 << 24,

            all = std::numeric_limits<bitmask_t>::max(),
            standard = time | altitude | attitude | motors | error
        };
    }

    // uav::state //////////////////////////////////////////////////////////////

    struct state
    {
        timestamp_t     t, t_abs, comptime; // time in millis, computation time
        pres_t          pres[2];            // pressure from imu/bmp
        temp_t          temp[2];            // temperature from above
        alt_t           dz;                 // altitude from home point

        attitude_t      h, p, r;            // heading, pitch, roll
        calib_t         calib;              // calibration status
        target_t        tz, th, tp, tr;     // targets for 4 degrees of freedom
        pid_ov_t        zov, hov, pov, rov; // respective pid response
        motor_t         motors[4];
        error_t         err;                // bitmask for storing error codes

        static std::string header(fmt::bitmask_t);
        bool operator==(const state& other);
        bool operator!=(const state& other);

        const static size_t fields = 25;
        const static size_t size = 3 * sizeof(timestamp_t) + 2 * sizeof(pres_t) +
            2 * sizeof(temp_t) + sizeof(alt_t) + 3 * sizeof(attitude_t) +
            sizeof(calib_t) + 4 * sizeof(target_t) + 4 * sizeof(pid_ov_t) +
            4 * sizeof(motor_t) + sizeof(error_t);

        using bin = std::array<byte, size>;
    };

    static_assert(state::fields == 25, "CONST_STATE_FIELDS_NOT_25");
    static_assert(state::size == 87, "CONST_STATE_SIZE_NOT_87");

    // uav::param //////////////////////////////////////////////////////////////

    struct param
    {
        freq_t          freq;               // frequency of updates in hz
        home_pres_t     p1h, p2h;           // home point pres from imu/bmp

        // pid gains for altitude, heading, pitch, roll
        pid_gain_t      zpidg[4], hpidg[4], ppidg[4], rpidg[4];

        lpf_tau_t       gz_rc;              // RC time constant for alt lpf
        wavg_t          gz_wam;             // weighted average gain towards z1
        mrate_t         maxmrate;           // max thrust d/dt in hz
        wgt_frac_t      mg;                 // vehicle weight/max thrust * 100

        static std::string header();
        bool operator==(const param& other);
        bool operator!=(const param& other);

        const static size_t fields = 23;
        const static size_t size = sizeof(freq_t) + 2 * sizeof(home_pres_t) +
            16 * sizeof(pid_gain_t) + sizeof(lpf_tau_t) + sizeof(wavg_t) +
            sizeof(mrate_t) + sizeof(wgt_frac_t);

        using bin = std::array<byte, size>;
    };

    static_assert(param::fields == 23, "CONST_PARAM_FIELDS_NOT_23");
    static_assert(param::size == 171, "CONST_PARAM_SIZE_NOT_171");

    // conversion functions ////////////////////////////////////////////////////

    param::bin serialize(const param& p);

    state::bin serialize(const state& s);

    param deserialize(const param::bin& b);

    state deserialize(const state::bin& b);

    template<class T, class U> std::array<uint8_t, T::size> wrap(U *ptr)
    {
        std::array<byte, T::size> bin;
        byte* src = reinterpret_cast<byte*>(ptr);
        std::copy(src, src + T::size, bin.begin());
        return bin;
    }

    // std::string functions ///////////////////////////////////////////////////

    std::string to_string(const param& prm);

    std::string to_string(const state& it, fmt::bitmask_t mask);

    std::string timestamp();

    // logging functions ///////////////////////////////////////////////////////

    void reset();

    void include(param p);

    void include(state s);

    void debug(std::string s);

    void info(std::string s);

    void error(std::string s);

    void flush();

    namespace tests
    {
        int uavcore();
    }
}

#endif // UAV_CORE_H
