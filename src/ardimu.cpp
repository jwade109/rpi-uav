#include <iostream>
#include <cstring>
#include <termios.h>
#include <fcntl.h>

#include <ardimu.h>

namespace uav
{
    const speed_t baud = B115200;

    Arduino::Arduino(): data{0}, cont(true), init(-1) { }

    Arduino::~Arduino()
    {
        cont = false;
        if (parser.joinable()) parser.join();
        in.close();
    }

    int Arduino::begin()
    {
        int fd = open("/dev/ttyACM0", O_RDWR);
        if (fd < 0)
        {
            std::cerr << "Arduino: Could not generate "
                         "file descriptor" << std::endl;
            return 1;
        }
        struct termios attr;
        int rt = -tcgetattr(fd, &attr);
        rt -= cfsetispeed(&attr, baud);
        rt -= cfsetospeed(&attr, baud);
        rt -= tcsetattr(fd, TCSANOW, &attr);
        if (rt < 0)
        {
            std::cerr << "Arduino: Failed to set "
                         "baud rate" << std::endl;
            return 2;
        }

        in.open("/dev/ttyACM0");
        if (!in)
        {
            std::cerr << "Arduino: Could not open "
                         "/dev/ttyACM0" << std::endl;
            return 3;
        }

        parser = std::thread(&Arduino::parse, this);

        while (init < 0);
        if (init) return 4;

        return 0;
    }

    const Message& Arduino::get()
    {
        return data;
    }

    void Arduino::parse()
    {
        size_t ptr = 0;
        bool message = false;
        char ch;
        char buffer[msg_len];

        while (in.get(ch) && cont)
        {
            if (ch == '#') init = 0;
            else if (ch == '!')
            {
                memset(buffer, 0, msg_len);
                buffer[0] = '!';
                in.get(ch);
                buffer[1] = ch;
                for (int i = 2; i < msg_len && ch != '!'; i++)
                {
                    in.get(ch);
                    buffer[i] = ch;
                }
                fprintf(stderr, "Arduino: Reporting error: "
                        "\"%s\"\n", buffer);
                cont = false;
                init = 1;
            }
            else if (ch == '<')
            {
                message = true;
                in.get(ch);
            }
            else if (ch == '>')
            {
                message = false;
                Message d;
                char* cursor;
                d.millis = strtol(buffer, &cursor, 10);
                d.heading = strtod(cursor, &cursor);
                d.pitch = strtod(cursor, &cursor);
                d.roll = strtod(cursor, &cursor);
                d.calib = strtol(cursor, &cursor, 10);
                d.pres = strtod(cursor, &cursor);
                d.temp = strtod(cursor, &cursor);
                data = d;
                memset(buffer, 0, msg_len);
                ptr = 0;
            }

            if (message)
            {
                buffer[ptr] = ch;
                ++ptr;
            }
        }
    }
}
