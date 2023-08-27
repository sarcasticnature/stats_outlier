#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
//#include <algorithm>

#include "Eigen/Dense"

#include "matplot/matplot.h"

// Represents a 3d vector with its tail at the origin
using Vec = Eigen::Vector3d;
//struct Vec
//{
//    double x, y, z;
//};

// Represents a point in 3d space. Same implementation as a vector, but different semantic meaning
using Point = Eigen::Vector3d;
//struct Point
//{
//    double x, y, z;
//};

// Represents a plane in 3d space using the coefficients of the plane equation az + by + cz + d = 0
struct Plane
{
    double a, b, c, d;
};

Plane computePlane(Vec v1, Vec v2, Point p)
{
    Vec v = v1.cross(v2);

    double d = -(v(0) * p(0) + v(1) * p(1) + v(2) * p(2));

    return {v(0), v(1), v(2), d};
}

double distanceFromPlane(Point point, Plane plane)
{
    const auto& [a, b, c, d] = plane;
    double num = std::abs(a * point(0) + b * point(1) + c * point(2) + d);
    double denom = std::sqrt(a*a + b*b + c*c);
    return num / denom;
}

//// Fn for running the RANSAC alg.
//// points: input data
//// k: number of iterations
//// t: inlier threshold
//// d: minimum number of inliers to
//Plane ransac(std::vector<Point> points, unsigned k, unsigned t, unsigned d)

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

    unsigned num_points = 100;
    double mean = 0.0;
    double stddev = 5.0;
    std::vector<double> xs = matplot::randn(num_points, mean, stddev);
    std::vector<double> ys = matplot::randn(num_points, mean, stddev);

    std::vector<double> zs;
    zs.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        zs.push_back(-(a * xs.at(i) + b * ys.at(i) + d) / c);
    }

    // turn some data points into outliers
    double outlier_ratio = 0.50;
    unsigned outlier_cnt = num_points * outlier_ratio;

    for (unsigned i = 0; i < outlier_cnt; ++i) {
        zs.at(i) += matplot::rande(0.1);
    }

    matplot::scatter3(xs, ys, zs, "filled");
    //matplot::zlim({-2, 2});
    matplot::show();

    //Plane model = ransac(xs, ys, zs);

    return 0;
}
