#ifndef ARCHIVE_H
#define ARCHIVE_H

#include <string>
#include <vector>
#include <array>
#include <iostream>

namespace uav
{

class archive
{
    public:

    static archive package(std::vector<uint8_t> raw);

    archive(const std::string& name = "Archive");
    archive(const std::string& name,
            std::vector<uint8_t> bytes);

    uint16_t size() const;
    const std::string& name() const;
    std::string& name();
    const std::vector<uint8_t>& bytes() const;
    std::vector<uint8_t>& bytes();
    std::vector<uint8_t> raw() const;
    bool fail() const;
    void clear();

    template <typename T, typename U =
        std::enable_if_t<std::is_fundamental<T>::value, T>>
        archive& operator << (T x);
    
    template <typename T, typename U =
        std::enable_if_t<std::is_fundamental<T>::value, T>>
        archive& operator >> (T& x);

    template <typename T, size_t N, typename U =
        std::enable_if_t<std::is_fundamental<T>::value, T>>
        archive& operator << (const std::array<T, N>& a);

    template <typename T, size_t N, typename U =
        std::enable_if_t<std::is_fundamental<T>::value, T>>
        archive& operator >> (std::array<T, N>& a);

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

template <typename T, size_t N, typename U>
archive& archive::operator << (const std::array<T, N>& a)
{
    auto src = reinterpret_cast<const uint8_t*>(a.data());
    _bytes.insert(_bytes.end(), src, src + sizeof(T) * N);
    return *this;
}

template <typename T, size_t N, typename U>
archive& archive::operator >> (std::array<T, N>& a)
{
    for (auto& e : a) *this >> e;
    return *this;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// converts an array of bytes to a vector of bytes
template <size_t N> std::vector<uint8_t>
    array2vector(const std::array<uint8_t, N> array);

// converts a contiguous block of primitives into a byte array
template <size_t N, typename T> std::array<uint8_t, N>
    wrap(T *ptr);

// converts a single primitive into a byte array
template <typename T> std::array<uint8_t, sizeof(T)>
    bytes(T var);

// converts an array of primitives into a byte array
template <typename T, size_t N> std::array<uint8_t, sizeof(T) * N>
    bytes(const std::array<T, N>& vars);

// converts a contiguous block of bytes into a primitive,
// and increments the passed read pointer
template <typename T> void
    debytes(const uint8_t* src, size_t& rptr, T& dest);

// concatenates two arrays of bytes
template <size_t N, size_t M> std::array<uint8_t, N + M>
    operator + (const std::array<uint8_t, N>& a, const std::array<uint8_t, M>& b);

// checks two byte arrays for equality
template <size_t N> bool
    operator == (const std::array<uint8_t, N>& a, const std::array<uint8_t, N>& b);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

template <size_t N>
std::vector<uint8_t> array2vector(const std::array<uint8_t, N> array)
{
    std::vector<uint8_t> vector(N,0);
    for (size_t i = 0; i < vector.size(); i++)
        vector[i] = array[i];
    return vector;
}

template <size_t N, typename T> std::array<uint8_t, N> wrap(T *ptr)
{
    static_assert(std::is_fundamental<T>::value,
        "This function should only be used for primitives");
    std::array<uint8_t, N> bin;
    uint8_t* src = reinterpret_cast<uint8_t*>(ptr);
    std::copy(src, src + N, begin(bin));
    return bin;
}

template <typename T> std::array<uint8_t, sizeof(T)> bytes(T var)
{
    static_assert(std::is_fundamental<T>::value,
        "This function should only be used for primitives");
    uint8_t* b = reinterpret_cast<uint8_t*>(&var);
    std::array<uint8_t, sizeof(T)> array;
    std::copy(b, b + sizeof(T), begin(array));
    return array;
}

template <typename T, size_t N>
std::array<uint8_t, sizeof(T) * N> bytes(const std::array<T, N>& vars)
{
    static_assert(std::is_fundamental<T>::value,
        "This function should only be used for primitives");
    std::array<uint8_t, sizeof(T) * N> array;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(vars.data());
    std::copy(src, src + sizeof(T) * N, begin(array));
    return array;
}

template <typename T>
void debytes(const uint8_t* src, size_t& rptr, T& dest)
{
    static_assert(std::is_fundamental<T>::value,
        "This function should only be used for primitives");
    dest = *reinterpret_cast<const T*>(src + rptr);
    rptr += sizeof(T);
}

template <size_t N, size_t M> std::array<uint8_t, N + M>
operator + (const std::array<uint8_t, N>& a, const std::array<uint8_t, M>& b)
{
    std::array<uint8_t, N + M> c;
    std::copy(begin(a), end(a), begin(c));
    std::copy(begin(b), end(b), begin(c) + N);
    return c;
}

template <size_t N> bool
operator == (const std::array<uint8_t, N>& a, const std::array<uint8_t, N>& b)
{
    for (size_t i = 0; i < N; i++)
        if (a[i] != b[i]) return false;
    return true;
}

} // namespace uav

#endif // ARCHIVE_H
