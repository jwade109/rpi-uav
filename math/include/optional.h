#ifndef OPTIONAL_H
#define OPTIONAL_H

#include <iostream>

namespace uav
{

template <typename T> class optional
{
    public:

    optional() : flag(true) { }
    optional(T init, bool flag = true) :
        value(init), flag(flag) { }

    bool is() const { return flag; }
    bool& is() { return flag; }

    T get() const { return value; }
    T& get() { return value; }

    operator T() const { return value; }
    operator T&() { return value; }

    template <typename U>
    explicit operator U() const { return (U) value; }

    template <typename U>
    optional<T>& operator = (const optional<U>& o)
    {
        value = o.get();
        flag = o.is();
        return *this;
    }

    private:

    T value;
    bool flag;
};

template <typename T>
optional<T> make_optional(T value, bool flag = true)
{
    return optional<T>(value, flag);
}

template <typename T>
std::ostream& operator << (std::ostream& os, const optional<T>& op)
{
    return os << op.get();
}

} // namespace uav

#endif // OPTIONAL_H
