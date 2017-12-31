#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

void append_checksum(std::string& msg)
{
    char check = std::accumulate(begin(msg), end(msg), 0,
        [](char sum, char ch) { return sum ^ ch; });
    std::stringstream checksum;
    checksum << "*" << std::hex << std::setw(2)
        << std::setfill('0') << (int) check;
    msg += checksum.str();
}

int main()
{
    std::cerr << "Enter a message: " << std::flush;
    std::string message;
    std::getline(std::cin, message);
    append_checksum(message);
    std::cerr << "$" << message << std::endl;

    return 0;
}
