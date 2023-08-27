#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
//#include <algorithm>

#include "matplot/matplot.h"

// Represents a 3d vector with its tail at the origin
struct Vec
{
    double x, y, z;
};

// Represents a point in 3d space. Same implementation as a vector, but different semantic meaning
struct Point
{
    double x, y, z;
};

// Represents a plane in 3d space using the coefficients of the plane equation az + by + cz + d = 0
struct Plane
{
    double a, b, c, d;
};

Plane computePlane(Vec v1, Vec v2, Point p)
{
    Vec v3;
    v3.x = v1.y * v2.z - v1.z * v2.y;
    v3.y = v1.z * v2.x - v1.x * v2.z;
    v3.z = v1.x * v2.y - v1.y * v2.x;

    //std::cout << "Normal vec: x=" << v3.x << ", y=" << v3.y << ", z=" << v3.z << std::endl;

    const auto& [a, b, c] = v3;

    double d = -(a * p.x + b * p.y + c * p.z);

    return {a, b, c, d};
}

double distanceFromPlane(Point point, Plane plane)
{
    const auto& [a, b, c, d] = plane;
    double num = std::abs(a * point.x + b * point.y + c * point.z + d);
    double denom = std::sqrt(a*a + b*b + c*c);
    return num / denom;
}

int main()
{
    // Set up the ground truth "ground plane"
    Vec v1 = {1, 0, 0};
    Vec v2 = {0, 1, 0};
    Point p = {0, 0, -1};

    Plane ground_plane = computePlane(v1, v2, p);
    const auto& [a, b, c, d] = ground_plane;

    std::cout << "coefficients of the *real* plane equation: a="
              << a << ", b=" << b << ", c=" << c << ", d=" << d << std::endl;

    // generate the data points

    unsigned int num_points = 100;
    double mean = 0;
    double stddev = 5;
    std::vector<double> xs = matplot::randn(num_points, mean, stddev);
    std::vector<double> ys = matplot::randn(num_points, mean, stddev);

    std::vector<double> zs;
    zs.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        zs.push_back(-(a * xs.at(i) + b * ys.at(i) + d) / c);
    }

    matplot::scatter3(xs, ys, zs, "filled");
    matplot::zlim({-2, 2});
    matplot::show();

    return 0;
}
