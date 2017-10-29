#include "data_frame.h"

namespace uav
{

data_frame::data_frame(const std::string& name,
        std::vector<uint8_t> bits) : _name(name), _bits(bits) { }

uint16_t data_frame::size() const
{
    return 10 + _bits.size();
}

const std::string& name() const
{
    return _name;
}

std::string& name()
{
    return _name;
}

const std::vector<uint8_t>& bits() const
{
    return _bits;
}

std::vector<uint8_t>& bits()
{
    return _bits;
}

std::vector<uint8_t> raw()
{
    uint16_t s = size();
    return std::vector<uint8_t>();
}
