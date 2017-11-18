#ifndef ARCHIVE_H
#define ARCHIVE_H

#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <type_traits>

namespace uav
{

class archive
{
    public:

    static archive package(std::vector<uint8_t> raw);
    static archive package(uint8_t* raw);

    archive(const std::string& name = "Archive");
    archive(const std::string& name,
            std::vector<uint8_t> bytes);

    uint16_t size() const;
    const std::string& name() const;
    std::string& name();
    const std::vector<uint8_t>& bytes() const;
    std::vector<uint8_t>& bytes();
    std::vector<uint8_t> raw() const;

    void seekp(signed pos);
    size_t tellp();
    void empty();
    bool fail() const;
    void clear();

    template <typename T, typename = typename
        std::enable_if<std::is_fundamental<T>::value, T>::type>
        archive& operator << (T x);
    
    template <typename T, typename = typename
        std::enable_if<std::is_fundamental<T>::value, T>::type>
        archive& operator >> (T& x);

    template <typename T, size_t N>
        archive& operator << (const std::array<T, N>& a);

    template <typename T, size_t N>
        archive& operator >> (std::array<T, N>& a);

    template <typename T>
        archive& operator << (const std::vector<T>& v);

    template <typename T>
        archive& operator >> (std::vector<T>& v);
    
    archive& operator << (const std::string& s);
    archive& operator >> (std::string& s);

    private:

    std::string _name;
    std::vector<uint8_t> _bytes;
    bool _fail_flag;
    size_t _read_pointer;
};

template <typename T, typename U> archive& archive::operator << (T x)
{
    uint8_t* b = reinterpret_cast<uint8_t*>(&x);
    _bytes.insert(_bytes.end(), b, b + sizeof(T));
    return *this;
}

template <typename T, typename U> archive& archive::operator >> (T& x)
{
    _fail_flag |= (_read_pointer + sizeof(T) > _bytes.size());
    if (_fail_flag)
    {
        x = T();
        return *this;
    }
    x = *reinterpret_cast<T*>(&_bytes[_read_pointer]);
    _read_pointer += sizeof(T);
    return *this;
}

template <typename T, size_t N>
archive& archive::operator << (const std::array<T, N>& a)
{
    for (auto& e : a) *this << e;
    return *this;
}

template <typename T, size_t N>
archive& archive::operator >> (std::array<T, N>& a)
{
    for (auto& e : a) *this >> e;
    return *this;
}

template <typename T>
archive& archive::operator << (const std::vector<T>& v)
{
    for (auto& e : v) *this << e;
    return *this;
}

template <typename T>
archive& archive::operator >> (std::vector<T>& v)
{
    for (auto& e : v) *this >> e;
    return *this;
}

} // namespace uav

#endif // ARCHIVE_H