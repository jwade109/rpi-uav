#include <iostream>
#include <string>
#include <sstream>
#include <tuple>
#include <vector>

template<typename T, typename U, typename... Args>
std::string print(T arg1, U arg2, Args... args)
{
    return to_string(arg1) + " " + print(arg2, args...);
}

template<size_t N, typename... Types>
void print_tuple(std::tuple<Types...> t)
{
    std::cout << std::get<N>(t) << std::endl;
}

template<typename... Types>
void print_last(std::tuple<Types...> t)
{
    std::cout << std::get<
        std::tuple_size<decltype(t)>::value - 1
    >(t) << std::endl;
}

template<size_t N, class... Types>
struct tuple_iter
{
    static std::string print_recurring(std::tuple<Types...> t)
    {
        std::stringstream s;
        s << tuple_iter<N-1, Types...>::print_recurring(t)
          << " " << std::get<N>(t);
        return s.str();
    }

    static size_t size_of(std::tuple<Types...> t)
    {
        return sizeof(std::get<N>(t)) +
            tuple_iter<N-1, Types...>::size_of(t);
    }

    static std::vector<uint8_t> to_binary(std::tuple<Types...> t)
    {
        auto data = std::get<N>(t);
        std::vector<uint8_t> v;

        uint8_t *bytes = reinterpret_cast<uint8_t*>(&data);
        for (int i = 0; i < sizeof(data); i++) v.push_back(bytes[i]);

        std::vector<uint8_t> other = tuple_iter<N-1, Types...>::to_binary(t);
        other.insert(other.end(), v.begin(), v.end());
        return other;
    }
};

template<class... Types>
struct tuple_iter<0, Types...>
{
    static std::string print_recurring(std::tuple<Types...> t)
    {
        std::stringstream s;
        s << std::get<0>(t);
        return s.str();
    }

    static size_t size_of(std::tuple<Types...> t)
    {
        return sizeof(std::get<0>(t));
    }

    static std::vector<uint8_t> to_binary(std::tuple<Types...> t)
    {
        auto data = std::get<0>(t);
        std::vector<uint8_t> v;
        uint8_t *bytes = reinterpret_cast<uint8_t*>(&data);
        for (int i = 0; i < sizeof(data); i++) v.push_back(bytes[i]);
        return v;
    }
};

template<class... Types>
std::string to_string(std::tuple<Types...> t)
{
    return tuple_iter<std::tuple_size<decltype(t)>::value - 1,
        Types...>::print_recurring(t);
}

template<class... Types>
size_t size_of(std::tuple<Types...> t)
{
    return tuple_iter<std::tuple_size<decltype(t)>::value - 1,
        Types...>::size_of(t);
}

template<class... Types>
std::vector<uint8_t> to_binary(std::tuple<Types...> t)
{
    return tuple_iter<std::tuple_size<decltype(t)>::value - 1,
        Types...>::to_binary(t);
}

int main()
{
    auto t = std::make_tuple((uint16_t) 0x7d22, (uint32_t) 0x11d4abcf);
    std::cout << to_string(t) << std::endl;
    std::cout << "[" << size_of(t) << "]" << std::endl;
    for (int e : to_binary(t))
        std::cout << std::hex << e << " ";
    std::cout << std::endl;
}
