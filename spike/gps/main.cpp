#include <iostream>
#include <iomanip>
#include <chrono>
#include <filters.h>
#include <gps.h>

struct east_north
{
    double e, n;
};

struct gps_tuple
{
    uint64_t ms;
    east_north pos, vel, accel;
};

std::ostream& operator << (std::ostream& os, const east_north& en)
{
    return os << en.e << ", " << en.n;
}

std::ostream& operator << (std::ostream& os, const gps_tuple& gt)
{
    return os << "<" << gt.ms << " |  " << gt.pos << " | "
        << gt.vel << " | " << gt.accel << ">";
}

void use(const uav::gpgga& gga)
{
    using namespace std::chrono;

    const static uav::gpgga home(gga);
    const static steady_clock::time_point first(steady_clock::now());
    static uav::derivative<1> v_e, v_n, a_e, a_n;

    gps_tuple tuple;
    static uint64_t last(0);
    tuple.ms = duration_cast<milliseconds>
        (steady_clock::now() - first).count();

    double dt = (tuple.ms - last)/1000.0;
    last = tuple.ms;

    auto rel = gga.pos - home.pos;
    tuple.pos.e = rel.x();
    tuple.pos.n = rel.y();

    tuple.vel.e = v_e(tuple.pos.e, dt);
    tuple.vel.n = v_n(tuple.pos.n, dt);

    tuple.accel.e = a_e(tuple.vel.e, dt);
    tuple.accel.n = a_n(tuple.vel.n, dt);

    std::cout << tuple << std::endl;
}

int main()
{
    using namespace std::chrono_literals;

    std::cout << std::fixed << std::setprecision(3);

    uav::gps gps;
    int ret;
    if ((ret = gps.begin()) > 0)
    {
        std::cerr << ret << std::endl;
        return 1;
    }

    auto old_data = gps.get().gga;
    while (1)
    {
        if (gps.isnew())
        {
            auto new_data = gps.get().gga;
            if (old_data.utc.second != new_data.utc.second)
            {
                use(new_data);
                old_data = new_data;
            }
        }
        std::this_thread::sleep_for(50ms);
    }

    return 0;
}
