#ifndef DATA_FRAME_H
#define DATA_FRAME_H

#include <string>
#include <vector>

class data_frame
{
    public:

    data_frame(const std::string& name,
               std::vector<uint8_t> bits);

    uint16_t size() const;
    const std::string& name() const;
    std::string& name();
    const std::vector<uint8_t>& bits() const;
    std::vector<uint8_t>& bits();

    std::vector<uint8_t>& raw();

    private:

    std::string _name;
    std::vector<uint8_t> _bits;
}

#endif // DATA_FRAME_H
