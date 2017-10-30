#include "data_frame.h"

namespace uav
{

data_frame data_frame::enframe(std::vector<uint8_t> raw)
{
    uint16_t size = *reinterpret_cast<uint16_t*>(&raw[0]);
    if (size != raw.size()) return data_frame("", {});
    std::string name;
    for (int i = 0; raw[i+2] != 0; i++)
        name.push_back(raw[i+2]);
    uint16_t data_start = 3 + name.length();
    std::vector<uint8_t> data(size - data_start, 0);
    for (int i = 0; i < size - data_start; i++)
        data[i] = raw[i+data_start];
    return data_frame(name, data);
}

data_frame::data_frame(const std::string& name) : _name(name) { }

data_frame::data_frame(const std::string& name,
    std::vector<uint8_t> bytes) : _name(name), _bytes(bytes) { }

uint16_t data_frame::size() const
{
    return 3 + _name.length() + _bytes.size();
}

const std::string& data_frame::name() const
{
    return _name;
}

std::string& data_frame::name()
{
    return _name;
}

const std::vector<uint8_t>& data_frame::bytes() const
{
    return _bytes;
}

std::vector<uint8_t>& data_frame::bytes()
{
    return _bytes;
}

std::vector<uint8_t> data_frame::raw() const
{
    std::vector<uint8_t> raw;
    uint16_t s = size();
    raw.resize(s);

    uint8_t* sbytes = reinterpret_cast<uint8_t*>(&s);
    raw[0] = sbytes[0];
    raw[1] = sbytes[1];

    for (int i = 0; i < _name.length(); i++)
    {
        raw[i+2] = _name[i];
    }
    uint16_t data_start = 3 + _name.length();
    raw[data_start - 1] = 0;
    for (int i = 0; i < _bytes.size(); i++)
        raw[i + data_start] = _bytes[i];
    return raw;
}

} // namespace uav
