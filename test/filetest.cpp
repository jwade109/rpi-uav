#include <filebuffer.h>
#include <stdio.h>
#include <timeutil.h>
#include <string>
#include <sstream>

int main()
{
    int wait = 5;
    unit_t prec = milli;
    
    FileBuffer fb("log/data.txt"), hd("log/hexdump.bin");
    fb.begin();
    hd.begin();

    uint64_t start = unixtime(prec);
    for (int i = 0; i < 100; i++)
    {
        uint64_t t = unixtime(prec);
        std::string xs("Well this here logging daemon contraption "
                       "is truly a remarkable invention which will allow "
                       "for much more reliable logging\n");
        std::string is = std::to_string(i);
        std::string ts = std::to_string(t);
        fb.push(is + " " + ts + " " + xs);
        waituntil(start+=wait, prec);
    }

    char buf[256];
    for (int i = 0; i < 256; i++)
        buf[i] = i;
    for (int i = 0; i < 10; i++)
    {
        hd.push(buf, 256);
        waitfor(1, milli);
    }
    waitfor(10, milli);
    fb.flush();
    hd.flush();

    return 0;
}
