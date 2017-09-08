#include <ardimu.h>

#include <cstring>
#include <iostream>
#include <fcntl.h>

namespace uav
{
    arduino::arduino(): data{0}, cont(true), status(-1) { }

    arduino::~arduino()
    {
        cont = false;
        if (parser.joinable()) parser.join();
        in.close();
    }

    int arduino::begin()
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
            std::cerr << "arduino: Failed to set "
                         "baud rate" << std::endl;
            return 2;
        }

        in.open("/dev/ttyACM0");
        if (!in)
        {
            std::cerr << "arduino: Could not open "
                         "/dev/ttyACM0" << std::endl;
            return 3;
        }

        parser = std::thread(&arduino::parse, this);

        while (status < 0);
        if (status) return 4;

        return 0;
    }

    const imu_packet& arduino::get() const
    {
        return data;
    }

    void arduino::parse()
    {
        size_t ptr = 0;
        bool recieved = false;
        char ch;
        std::array<char, buffer_size> buffer;

        while (in.get(ch) && cont)
        {
            if (ch == '#') status = 0;
            else if (ch == '!')
            {
                buffer.fill(0);
                buffer[0] = '!';
                in.get(ch);
                buffer[1] = ch;
                for (size_t i = 2; i < buffer_size && ch != '!'; i++)
                {
                    in.get(ch);
                    buffer[i] = ch;
                }
                fprintf(stderr, "Arduino: Reporting error: "
                        "\"%s\"\n", buffer.data());
                cont = false;
                status = 1;
            }
            else if (ch == '<')
            {
                recieved = true;
                in.get(ch);
            }
            else if (ch == '>')
            {
                recieved = false;
                imu_packet d;
                char* cursor;
                d.millis = strtol(buffer.data(), &cursor, 10);
                d.heading = strtod(cursor, &cursor);
                d.pitch = strtod(cursor, &cursor);
                d.roll = strtod(cursor, &cursor);
                d.calib = strtol(cursor, &cursor, 10);
                d.pres = strtod(cursor, &cursor);
                d.temp = strtod(cursor, &cursor);
                data = d;
                buffer.fill(0);
                ptr = 0;
            }

            if (recieved)
            {
                buffer[ptr] = ch;
                ++ptr;
            }
        }
    }
}
