#ifndef UAV_CORE_H
#define UAV_CORE_H

#include <fstream>
#include <deque>
#include <array>

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

    using home_pres_t   = double;
    using pid_gain_t    = double;
    using lpf_tau_t     = double;
    using wavg_t        = double;
    using mrate_t       = uint16_t;
    using wgt_frac_t    = double;

    enum freq_t : uint8_t
    {
        f10hz = 10, f20hz = 20, f25hz = 25, f40hz = 40, f50hz = 50,
        f100hz = 100, f125hz = 125, f200hz = 200, f250hz = 250,

        fdefault = f100hz
    };

    // uav::state //////////////////////////////////////////////////////////////

    struct state
    {
        timestamp_t     t, comptime;        // time in millis, computation time
        pres_t          pres[2];            // pressure from imu/bmp
        temp_t          temp[2];            // temperature from above
        alt_t           dz;                 // altitude from home point

        attitude_t      h, p, r;            // heading, pitch, roll
        calib_t         calib;              // calibration status
        target_t        tz, th, tp, tr;     // targets for 4 degrees of freedom
        pid_ov_t        zov, hov, pov, rov; // respective pid response
        motor_t         motors[4];
        error_t         err;                // bitmask for storing error codes
    };

    const size_t state_fields = 24;
    
    const size_t state_size = 2*sizeof(timestamp_t) + 2*sizeof(pres_t) +
        2*sizeof(temp_t) + sizeof(alt_t) + 3*sizeof(attitude_t) +
        sizeof(calib_t) + 4*sizeof(target_t) + 4*sizeof(pid_ov_t) +
        4*sizeof(motor_t) + sizeof(error_t);

    static_assert(state_fields == 24, "CONST_STATE_FIELDS_NOT_24");
    static_assert(state_size == 79, "CONST_STATE_SIZE_NOT_71");

    using state_bin = std::array<uint8_t, state_size>;

    // uav::param //////////////////////////////////////////////////////////////

    struct param
    {
        freq_t          freq;               // frequency of updates in hz
        home_pres_t     p1h, p2h;           // home point pres from imu/bmp

        // pid gains for altitude, heading, pitch, roll
        pid_gain_t      zpidg[4], hpidg[4], ppidg[4], rpidg[4];
        
        lpf_tau_t       gz_rc;              // RC time constant for alt low-pass filter
        wavg_t          gz_wam;             // weighted average gain towards z1
        mrate_t         maxmrate;           // max motor thrust rate of change in hz
        wgt_frac_t      mg;                 // vehicle weight/max thrust * 100
    };

    const size_t param_fields = 23;

    const size_t param_size = sizeof(freq_t) + 2*sizeof(home_pres_t) +
        16*sizeof(pid_gain_t) + sizeof(lpf_tau_t) + sizeof(wavg_t) +
        sizeof(mrate_t) + sizeof(wgt_frac_t);

    static_assert(param_fields == 23, "CONST_PARAM_FIELDS_NOT_23");
    static_assert(param_size == 171, "CONST_PARAM_SIZE_NOT_171");

    using param_bin = std::array<uint8_t, param_size>;
    
    // conversion functions ////////////////////////////////////////////////////

    param_bin to_binary(const param& p);

    state_bin to_binary(const state& s);

    param from_binary(const param_bin& b);

    state from_binary(const state_bin& b);

    template<size_t N, class T> std::array<uint8_t, N> wrap(T *ptr)
    {
        std::array<uint8_t, N> bin;
        uint8_t *src = reinterpret_cast<uint8_t*>(ptr);
        std::copy(src, src + N, bin.begin());
        return bin;
    }

    // file i/o functions //////////////////////////////////////////////////////

    std::string pheader();

    std::string sheader(uint64_t mask);

    std::string to_string(const param& prm);

    std::string to_string(const state& it, uint64_t mask);

    std::string ts(uint64_t ms);

    extern std::deque<std::string> debug, info, error;

    void debug_write(const std::string& s);

    void info_write(const std::string& s);

    void error_write(const std::string& s);
}

#endif // UAV_CORE_H
