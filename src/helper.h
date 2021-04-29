#pragma once
#include <tuple>
#include <cmath>
namespace PPCHelper
{
    template <typename type>
    std::tuple<type, type, type, type> fit_plane(const type &x1, const type &x2, const type &x3,
                                                 const type &y1, const type &y2, const type &y3,
                                                 const type &z1, const type &z2, const type &z3)
    {
        const auto A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        const auto B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        const auto C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        const auto D = -(A * x1 + B * y1 + C * z1);
        return std::make_tuple(A, B, C, D);
    }

    template<typename type>
    float distance_3d(const type& A, const type&  B, const type& C, 
                    const type& D, const type& x, const type& y, const type& z)
    {
        return std::fabs(A * x + B * y + C * z + D) / (float)sqrt(A * A + B * B + C * C);
    }
}