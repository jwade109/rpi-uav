#ifndef SERIAL_H
#define SERIAL_H

#include <array>

namespace uav
{

template <size_t N, typename T> std::array<uint8_t, N> wrap(T *ptr)
{
    std::array<uint8_t, N> bin;
    uint8_t* src = reinterpret_cast<uint8_t*>(ptr);
    std::copy(src, src + N, begin(bin));
    return bin;
}

template <typename T> std::array<uint8_t, sizeof(T)> bin(T var)
{
    uint8_t* b = reinterpret_cast<uint8_t*>(&var);
    std::array<uint8_t, sizeof(T)> array;
    std::copy(b, b + sizeof(T), begin(array));
    return array;
}

template <typename T, size_t N>
std::array<uint8_t, sizeof(T) * N> bin(const std::array<T, N>& vars)
{
    std::array<uint8_t, sizeof(T) * N> array;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(vars.data());
    std::copy(src, src + sizeof(T) * N, begin(array));
    return array;
}

template <typename T>
void bin(const uint8_t* src, size_t& rptr, T& dest)
{
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

#endif // SERIAL_H
