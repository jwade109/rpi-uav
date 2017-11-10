#include "state.h"

#include <bitset>
#include "raw_data.h"

namespace uav
{

std::string state::header()
{
    using namespace std;
    stringstream line;
    line << left;

    line << setw(15) << "time";
    line << setw(15) << "t_abs" << setw(15) << "comp_us";
    line << setw(55) << "pos";
    line << setw(15) << "hdg" << setw(15) << "pitch"
         << setw(15) << "roll";
    line << setw(15) << "tx" << setw(15) << "ty"
         << setw(15) << "tz" << setw(15) << "th";
    line << setw(15) << "m1" << setw(15) << "m2"
         << setw(15) << "m3" << setw(15) << "m4";
    line << setw(20) << "err";
    line << setw(15) << "status";
    return line.str();
}

std::ostream& operator << (std::ostream& os, const state& s)
{
    using namespace std;
    stringstream line;
    line << left << fixed << setprecision(3);

    line << setw(15) << s.time[0]/1000.0
         << setw(15) << s.time[1]/1000.0
         << setw(15) << s.time[2]/1000.0
         << setw(55) << s.position;
    for (angle e : s.attitude) line << setw(15) << e;
    for (double e : s.targets) line << setw(15) << e;
    for (double e : s.motors) line << setw(15) << e;
    line << setw(20) << std::bitset<16>(s.error)
         << (int) s.status << "-";
    const char *str[] = {"NULL_STATUS", "ALIGNING", "NO_VEL",
        "POS_SEEK", "POS_HOLD", "UPSIDE_DOWN"};
    line << str[s.status];
    return os << line.str();
}

archive& operator << (archive& a, const state& s)
{
    return a << s.time << s.position << s.attitude << s.targets
             << s.motors << s.error << s.status;
}

archive& operator >> (archive& a, state& s)
{
    return a >> s.time >> s.position >> s.attitude >> s.targets
             >> s.motors >> s.error >> s.status;
}

} // namespace uav
