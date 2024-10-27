#pragma once
#include "MotionProfile.hpp"
CubicBezier::CubicBezier(const Point2D &p0, const Point2D &p1, const Point2D &p2, const Point2D &p3, int length)
{
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
    // this->P << p0.x, p1.x, p2.x, p3.x,
    //     p0.y, p1.y, p2.y, p3.y;
    this->P << p0.x, p0.y,
        p1.x, p1.y,
        p2.x, p2.y,
        p3.x, p3.y;

    this->len = length;
    this->lengths = std::vector<double>(this->len + 1, 0);

    this->coefficients << -1, 3, -3, 1,
        3, -6, 3, 0,
        -3, 3, 0, 0,
        1, 0, 0, 0;
    this->derivativeCoef << -3, 9, -9, 3,
        6, -12, 6, 0,
        -3, 3, 0, 0;
    this->second_derivative_coef << -6, 18, -18, 6,
        6, -12, 6, 0;
}
