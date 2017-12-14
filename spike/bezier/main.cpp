#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>

template <size_t N> using point = Eigen::Matrix<double, N, 1>;
template <size_t N> using pvector =
    std::vector<point<N>, Eigen::aligned_allocator<point<N>>>;

template <size_t N = 2> class bezier_curve
{
    static_assert(N > 1, "Curve dimension must be at least 2");

    public:

    bezier_curve(const bezier_curve<N>& curve);
    bezier_curve(const pvector<N>& handles);

    const pvector<N>& handles() const;
    pvector<N>& handles();

    point<N> sample(double t);

    private:

    pvector<N> _handles;
};

template <size_t N>
bezier_curve<N>::bezier_curve(const bezier_curve<N>& curve)
    : _handles(curve.handles()) { };

template <size_t N>
bezier_curve<N>::bezier_curve(const pvector<N>& handles)
    : _handles(handles) { };

template <size_t N>
const pvector<N>& bezier_curve<N>::handles() const
{
    return _handles;
}

template <size_t N>
pvector<N>& bezier_curve<N>::handles()
{
    return _handles;
}

template <size_t N> point<N> bezier_curve<N>::sample(double t)
{
    t = t > 1 ? 1 : t < 0 ? 0 : t;
    
    auto calculate = [t] (const pvector<N>& handles)
    {
        pvector<N> ret(handles.size() - 1);
        for (size_t i = 0; i < ret.size(); ++i)
            ret[i] = handles[i] + t*(handles[i+1] - handles[i]);
        return ret;
    };
    
    auto condensed = calculate(_handles);
    while (condensed.size() > 1)
        condensed = calculate(condensed);
    return condensed[0];
}

int main()
{
    point<2> p0{0, 0}, p1{1, 0}, p2{0, 1}, p3{1, 1};
    bezier_curve<2> bc({p0, p1, p2, p3});
    for (double t = 0; t <= 1; t += 0.001)
    {
        auto x = bc.sample(t);
        std::cout << x(0) << " " << x(1) << std::endl;
    }
    return 0;
}
