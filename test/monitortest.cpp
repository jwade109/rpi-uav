#include <iostream>
#include <sstream>
#include <chrono>
#include <assert.h>

#include <monitor.h>

int main()
{
    using namespace uav;
    namespace chrono = std::chrono;

    Monitor m;
    m.open();

    auto start = chrono::steady_clock::now();
    for (size_t i = 0; i < 2500000; i++)
    {
        std::stringstream line;
        auto now = chrono::steady_clock::now();
        auto ts = chrono::duration_cast<chrono::milliseconds>
            (now - start).count();
        line << ts << " " << i << "\n";

        m.states.put(line.str());
        m.params.put(line.str());
        m.events.put(line.str());
    }

    m.flush();
    m.close();
    
    return 0;
}
