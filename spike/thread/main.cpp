#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <vector>

class Background
{
    public:

    Background();
    ~Background();
    const std::vector<int>& getdata() const; // read-only

    private:

    std::vector<int> data;
    std::thread worker;
    bool cont;

    void work();
};

Background::Background() : cont(true)
{
    worker = std::thread(&Background::work, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

Background::~Background()
{
    cont = false;
    worker.join();
}

const std::vector<int>& Background::getdata() const
{
    return data;
}

void Background::work()
{
    using namespace std::chrono;
    static steady_clock::time_point last;
    static int count(0);
    while (cont)
    {
        auto now = steady_clock::now();
        if (now - last > milliseconds(200))
        {
            data.push_back(count);
            last = now;
            count++;
        }
    }
}

int main()
{
    Background bg;
    for (int i = 0; i < 30; i++)
    {
        for (auto e : bg.getdata()) std::cout << e << " " << std::flush;
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
