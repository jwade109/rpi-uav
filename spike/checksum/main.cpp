#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

uint8_t calculate_checksum(const std::string& msg)
{
    return std::accumulate(begin(msg), end(msg), 0,
            [] (uint8_t sum, char ch) { return sum^ch; });
}

bool verify_checksum(const std::string& msg, uint8_t checksum)
{
    return checksum == calculate_checksum(msg);
}

void make_message(std::string& msg)
{
    int check = calculate_checksum(msg);
    std::stringstream checksum;
    checksum << "*" << std::hex << std::setw(2)
        << std::setfill('0') << check;
    msg += checksum.str();
}

int main()
{
    std::cerr << "Enter a message: " << std::flush;
    std::string message;
    std::getline(std::cin, message);
    std::cerr << "Enter the checksum: " << std::flush;
    int checkguess;
    std::cin >> checkguess;

    std::cerr << "You guessed " << checkguess << "." << std::endl;
   
    if (verify_checksum(message, checkguess))
    {
        std::cerr << "Correct!" << std::endl;
    }
    else
    {
        std::cerr << "Incorrect." << std::endl;
    }

    make_message(message);
    std::cerr << "$" << message << std::endl;

    return 0;
}
