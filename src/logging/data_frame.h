#ifndef DATA_FRAME_H
#define DATA_FRAME_H

#include <string>
#include <vector>

namespace uav
{

class data_frame
{
    public:

    static data_frame enframe(std::vector<uint8_t> raw);

    data_frame(const std::string& name);
    data_frame(const std::string& name,
               std::vector<uint8_t> bytes);

    uint16_t size() const;
    const std::string& name() const;
    std::string& name();
    const std::vector<uint8_t>& bytes() const;
    std::vector<uint8_t>& bytes();

    std::vector<uint8_t> raw() const;

    private:

    std::string _name;
    std::vector<uint8_t> _bytes;
};

} // namespace uav

#endif // DATA_FRAME_H
