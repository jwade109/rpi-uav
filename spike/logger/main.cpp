#include <filebuffer.h>
#include <stdio.h>
#include <timeutil.h>
#include <string.h>

int main()
{
    int wait = 1;
    unit_t prec = milli;
    
    FileBuffer fb("data.txt"), hd("hexdump.bin");
    fb.begin();
    hd.begin();

    uint64_t start = unixtime(prec);
    for (int i = 0; i < 100; i++)
    {
        uint64_t t = unixtime(prec);
        std::string xs("Well this here logging daemon contraption is truly a remarkable invention which will allow for much more reliable logging\n");
        std::string is = std::to_string(i);
        std::string ts = std::to_string(t);
        fb.push(is + " " + ts + " " + xs);
        waituntil(start + (i+1)*wait, prec);
    }

    char buf[256];
    for (int i = 0; i < 256; i++)
        buf[i] = i;
    hd.push(buf, 256);
    waitfor(10, milli);

    return 0;
}
