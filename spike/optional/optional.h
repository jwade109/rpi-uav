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

    private:

    T value;
    bool flag;
};

template <typename T>
optional<T> make_optional(T value, bool flag = true)
{
    return optional<T>(value, flag);
}

} // namespace uav
