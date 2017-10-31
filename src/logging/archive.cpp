#include "archive.h"

namespace uav
{

archive archive::package(std::vector<uint8_t> raw)
{
    uint16_t size = *reinterpret_cast<uint16_t*>(&raw[0]);
    if (size != raw.size()) return archive("", {});
    std::string name;
    for (int i = 0; raw[i+2] != 0; i++)
        name.push_back(raw[i+2]);
    uint16_t data_start = 3 + name.length();
    std::vector<uint8_t> data(size - data_start, 0);
    for (int i = 0; i < size - data_start; i++)
        data[i] = raw[i+data_start];
    return archive(name, data);
}

archive archive::package(uint8_t* raw)
{
    uint16_t size = *reinterpret_cast<uint16_t*>(raw);
    return package(std::vector<uint8_t>(raw, raw + size));
}

archive::archive(const std::string& name) :
    _name(name), _fail_flag(false), _read_pointer(0) { }

archive::archive(const std::string& name, std::vector<uint8_t> bytes) :
    _name(name), _bytes(bytes), _fail_flag(false), _read_pointer(0) { }

uint16_t archive::size() const
{
    return 3 + _name.length() + _bytes.size();
}

const std::string& archive::name() const
{
    return _name;
}

std::string& archive::name()
{
    return _name;
}

const std::vector<uint8_t>& archive::bytes() const
{
    return _bytes;
}

std::vector<uint8_t>& archive::bytes()
{
    return _bytes;
}

std::vector<uint8_t> archive::raw() const
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

void archive::seekp(signed pos)
{
    _read_pointer = pos > _bytes.size() ? _bytes.size() :
                    pos < 0 ? 0 : pos;
}

size_t archive::tellp()
{
    return _read_pointer;
}

void archive::empty()
{
    _bytes.clear();
    _read_pointer = 0;
    _fail_flag = false;
}

bool archive::fail() const
{
    return _fail_flag;
}

void archive::clear()
{
    _fail_flag = false;
}

archive& archive::operator << (const std::string& s)
{
    _bytes.insert(_bytes.end(), begin(s), end(s));
    _bytes.push_back(0);
    return *this;
}

archive& archive::operator >> (std::string& s)
{
    s = std::string(reinterpret_cast<char*>
        (_bytes.data() + _read_pointer));
    _read_pointer += s.length() + 1;
    return *this;
}

} // namespace uav
