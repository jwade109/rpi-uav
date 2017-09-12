#include <iostream>
#include <array>
#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

template<class T, size_t A, size_t B>
std::array<T, A + B> concat(std::array<T, A>& a, std::array<T, B>& b)
{
    std::array<T, A + B> c;
    auto midc = std::copy(begin(a), end(a), begin(c));
    std::copy(begin(b), end(b), midc);
    return c;
}

template<class IIter>
double rms(IIter begin, IIter end)
{
    double init = 0.0;
    return sqrt(std::inner_product(begin, end, begin, init)/(end - begin));
}

template<int M, int N, class T>
std::array<T, M*N> mat2array(Eigen::Matrix<T, M, N>& m)
{
    std::array<T, M*N> a;
    std::copy(m.data(), m.data() + m.size(), begin(a));
    return a;
}

template<int M, int N, class T>
std::array<uint8_t, M*N*sizeof(T)> mat2bin(Eigen::Matrix<T, M, N>& m)
{
    const size_t bytes = M * N * sizeof(T);
    std::array<uint8_t, bytes> bin;
    const uint8_t *matdata = reinterpret_cast<uint8_t*>(m.data());
    std::copy(matdata, matdata + m.size() * sizeof(T), begin(bin));
    return bin;
}

template<class T, int M, int N>
Eigen::Matrix<T, M, N> bin2mat(std::array<uint8_t, M*N*sizeof(T)>& bin)
{
    const size_t bytes = M * N * sizeof(T);
    Eigen::Matrix<T, M, N> m;
    uint8_t *matdata = reinterpret_cast<uint8_t*>(m.data());
    std::copy(begin(bin), begin(bin) + bytes, matdata);
    return m;
}

int main()
{
    Eigen::Matrix<double, 4, 7> m = Eigen::Matrix<double, 4, 7>::Random();
    auto array = mat2array(m);

    std::cout << m << std::endl << std::endl;
    for (auto e : array) std::cout << e << " " << std::flush;
    std::cout << std::endl;

    std::cout << rms(m.data(), m.data() + m.size()) << std::endl;

    auto bin = mat2bin(m);
    for (auto e : bin) std::cout << std::hex << (int) e << " " << std::flush;
    std::cout << std::dec << std::endl;
    auto newmat = bin2mat<double, 4, 7>(bin);
    std::cout << newmat << std::endl;

    std::cout << rms(newmat.data(), newmat.data() + newmat.size())
              << std::endl;

    std::array<int, 5> arr1{1, 2, 3, 4, 5}, arr2{10, 9, 8, 7, 6};
    auto con = concat(arr1, arr2);
    for (auto e : con) std::cout << e << " " << std::flush;
    std::cout << std::endl;

    return 0;
}
