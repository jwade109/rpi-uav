#ifndef SERIAL_H
#define SERIAL_H

#include <vector>
#include <array>

namespace uav
{

// converts an array of bytes to a vector of bytes
template <size_t N>
std::vector<uint8_t> array2vector(const std::array<uint8_t, N> array)
{
    std::vector<uint8_t> vector(N,0);
    for (size_t i = 0; i < vector.size(); i++)
        vector[i] = array[i];
    return vector;
}

// converts a contiguous block of primitives into a byte array
template <size_t N, typename T> std::array<uint8_t, N> wrap(T *ptr)
{
    static_assert(std::is_fundamental<T>::value,
        "This function should only be used for primitives");
    std::array<uint8_t, N> bin;
    uint8_t* src = reinterpret_cast<uint8_t*>(ptr);
    std::copy(src, src + N, begin(bin));
    return bin;
}

// converts a single primitive into a byte array
template <typename T> std::array<uint8_t, sizeof(T)> bytes(T var)
{
    static_assert(std::is_fundamental<T>::value,
        "This function should only be used for primitives");
    uint8_t* b = reinterpret_cast<uint8_t*>(&var);
    std::array<uint8_t, sizeof(T)> array;
    std::copy(b, b + sizeof(T), begin(array));
    return array;
}

// converts an array of primitives into a byte array
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

// converts a contiguous block of bytes into a primitive,
// and increments the passed read pointer
template <typename T>
void debytes(const uint8_t* src, size_t& rptr, T& dest)
{
    static_assert(std::is_fundamental<T>::value,
        "This function should only be used for primitives");
    dest = *reinterpret_cast<const T*>(src + rptr);
    rptr += sizeof(T);
}

// concatenates two arrays of bytes
template <size_t N, size_t M> std::array<uint8_t, N + M>
operator + (const std::array<uint8_t, N>& a, const std::array<uint8_t, M>& b)
{
    std::array<uint8_t, N + M> c;
    std::copy(begin(a), end(a), begin(c));
    std::copy(begin(b), end(b), begin(c) + N);
    return c;
}

// checks two byte arrays for equality
template <size_t N> bool
operator == (const std::array<uint8_t, N>& a, const std::array<uint8_t, N>& b)
{
    for (size_t i = 0; i < N; i++)
        if (a[i] != b[i]) return false;
    return true;
}

} // namespace uav

#endif // SERIAL_H
