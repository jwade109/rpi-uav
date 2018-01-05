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

template <size_t N>
bezier_curve<N> make_bezier(const point<N>& p0, const point<N>& d0,
    const point<N>& p1, const point<N>& d1, double radius)
{
    pvector<N> ret(5);
    ret[0] = p0;
    ret[1] = p0 + d0.normalized()*radius;
    ret[2] = (p0 + p1)*0.5;
    ret[3] = p1 - d1.normalized()*radius;
    ret[4] = p1;
    return bezier_curve<N>(ret);
}

template <size_t N> class bezier_path
{
    public:

    bezier_path(std::vector<bezier_curve<N>> curves);

    double limit() const;

    point<N> sample(double t);

    private:

    std::vector<bezier_curve<N>> _curves;
};

template <size_t N>
bezier_path<N>::bezier_path(std::vector<bezier_curve<N>> curves)
    : _curves(curves) { };

template <size_t N>
double bezier_path<N>::limit() const { return _curves.size(); }

template <size_t N>
point<N> bezier_path<N>::sample(double t)
{
    t = t < 0 ? 0 : t;
    if (t >= limit()) return _curves[limit()-1].sample(1);
    return _curves[std::floor(t)].sample(std::fmod(t, 1));
}

int main()
{
    auto b1 = make_bezier<2>({0,0}, {1,0}, {8,5}, {1,-0.5}, 2);
    auto b2 = make_bezier<2>({8,5}, {1,-0.5}, {12,-1}, {0,-1}, 2);
    auto b3 = make_bezier<2>({12,-1}, {0,-1}, {1,4}, {-1,2}, 2);

    bezier_path<2> bp({b1, b2, b3});

    for (double t = -1; t < bp.limit() + 1; t += 0.001)
    {
        auto x = bp.sample(t);
        std::cout << t << " " << x(0) << " " << x(1) << std::endl;
    }

    return 0;
}
