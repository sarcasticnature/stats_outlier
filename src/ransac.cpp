#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <optional>
#include <sstream>
//#include <algorithm>

#include "Eigen/Dense"

#include "matplot/matplot.h"

// Represents a 3d vector with its tail at the origin
using Vec = Eigen::Vector3d;

// Represents a point in 3d space. Same implementation as a vector, but different semantic meaning
using Point = Eigen::Vector3d;

// Represents a nx3 matrix of points
using PointMatrix = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

// Represents a plane in 3d space using the coefficients of the plane equation az + by + cz + d = 0
struct Plane
{
    double a, b, c, d;
};

// Compute a plane using two vectors and a point
Plane computePlaneVec(Vec v1, Vec v2, Point p)
{
    Vec n = v1.cross(v2);

    double d = -(n(0) * p(0) + n(1) * p(1) + n(2) * p(2));

    return {n(0), n(1), n(2), d};
}

// Compute a plane using three points (same signature as above, but with different semantics)
Plane computePlanePoints(Point p1, Point p2, Point p3)
{
    Vec v1 = p1 - p3;
    Vec v2 = p2 - p3;

    Vec n = v1.cross(v2);

    double d = -(n(0) * p3(0) + n(1) * p3(1) + n(2) * p3(2));

    return {n(0), n(1), n(2), d};
}

// Compute a plane using the plane's normal vector and a point
Plane computePlaneNormal(Vec n, Point p)
{
    double d = -(n(0) * p(0) + n(1) * p(1) + n(2) * p(2));

    return {n(0), n(1), n(2), d};
}

double distanceFromPlane(Point point, Plane plane)
{
    const auto& [a, b, c, d] = plane;
    double num = std::abs(a * point(0) + b * point(1) + c * point(2) + d);
    double denom = std::sqrt(a*a + b*b + c*c);
    return num / denom;
}

PointMatrix zipPoints(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs)
{
    // There's probably a better way of doing this with Eigen::Map, but this is (conceptually)
    // simpler

    size_t n = std::min(xs.size(), ys.size());
    n = std::min(n, zs.size());
    PointMatrix mat(n, 3);

    for (size_t i = 0; i < n; ++i) {
        mat.row(i) << xs.at(i), ys.at(i), zs.at(i);
    }

    return mat;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
unzipPoints(PointMatrix points)
{
    std::vector<double> xs, ys, zs;
    int num_points = points.rows();
    xs.reserve(num_points);
    ys.reserve(num_points);
    zs.reserve(num_points);

    for (int i = 0; i < num_points; ++i) {
        xs.push_back(points.row(i)(0));
        ys.push_back(points.row(i)(1));
        zs.push_back(points.row(i)(2));
    }
    return {xs, ys, zs};
}

std::tuple<int, int, int> pick3(int min, int max)
{
    return {matplot::randi(min, max), matplot::randi(min, max), matplot::randi(min, max)};
}

// Fn for running the RANSAC alg.
// points: input data (nx3 matrix)
// k: number of iterations
// t: inlier threshold
// d: minimum number of inliers to be considered a 'good' model
//
// TODO: create an alternate implementation that doesn't append inliers to an empty array and
// instead does something else more... efficient
std::optional<Plane> ransac(PointMatrix points, unsigned k, unsigned t, unsigned d)
{
    std::optional<Plane> best_model = std::nullopt;
    int best_inliers = 0;
    int num_points = points.rows();
    Plane model;
    PointMatrix inliers;
    int inlier_cnt;

    while (k > 0) {
        auto [fst, snd, thrd] = pick3(0, num_points - 1);
        model = computePlanePoints(points.row(fst), points.row(snd), points.row(thrd));
        inliers.resize(0, 3);

        for (auto i = 0; i < num_points; ++i) {
            if (distanceFromPlane(points.row(i), model) < t) {
                inliers.conservativeResize(inliers.rows() + 1, Eigen::NoChange);
                inliers.bottomRows(1) = points.row(i);
            }
        }

        if (inliers.rows() > d) {
            Eigen::RowVector3d mean = inliers.colwise().mean();   // TODO: move this out of the loop
            inliers.rowwise() -= mean;
            Eigen::BDCSVD<PointMatrix> svd(inliers, Eigen::ComputeFullV);
            Vec normal = svd.matrixV().rightCols(1);
            model = computePlaneNormal(normal, mean.transpose());
            inlier_cnt = 0;

            for (auto i = 0; i < num_points; ++i) {
                if (distanceFromPlane(points.row(i), model) < t) {
                    ++inlier_cnt;
                }
            }

            if (inlier_cnt > best_inliers) {
                best_model = model;
                best_inliers = inlier_cnt;
            }
        }

        --k;
    }
    return best_model;
}

int main()
{
    // Set up the ground truth "ground plane"

    // xy plane at -10 z
    Vec v1 = {1, 0, 0};
    Vec v2 = {0, 1, 0};
    Point p = {0, 0, -10};
    Plane ground_plane = computePlaneVec(v1, v2, p);

    // rando plane
    //Point p1 = {1, 0, 0};
    //Point p2 = {0, 1, 0};
    //Point p3 = {0, 0, 1};
    //Plane ground_plane = computePlanePoints(p1, p2, p3);

    const auto& [a, b, c, d] = ground_plane;
    std::cout << "coefficients of the *real* plane equation: a="
              << a << ", b=" << b << ", c=" << c << ", d=" << d << std::endl;

    Vec n = {a, b, c};
    double norm = n.norm();
    double a_ = a / norm;
    double b_ = b / norm;
    double c_ = c / norm;
    double d_ = d / norm;

    std::cout << "After normalizing the original plane normal vector, we get: "
              << a_ << ", " << b_ << ", " << c_ << ", " << d_ << std::endl;

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
    double outlier_ratio = 0.20;
    unsigned outlier_cnt = num_points * outlier_ratio;

    for (unsigned i = 0; i < outlier_cnt; ++i) {
        zs.at(i) += matplot::rande(0.1);
    }

    PointMatrix point_mat = zipPoints(xs, ys, zs);

    // TODO: remove hardcoding? esp. given 0.1 doesn't match the actual outlier ratio
    //unsigned k = std::log(1.0 - 0.99) / std::log(1 - std::pow(1 - .1, num_points));
    //std::cout << "k = " << k << std::endl;
    unsigned k = 10000;

    // TODO: futz with the parameters, esp. t & d
    auto model_opt = ransac(point_mat, k, 1.0, num_points / 5);
    Plane model = model_opt.value();
    std::cout << "Best model has coefficients: a="
              << model.a << ", b=" << model.b << ", c=" << model.c << ", d=" << model.d
              << std::endl;

    // TODO: move this into its own function (or return the inliers from the ransac function)
    PointMatrix inliers;
    for (size_t i = 0; i < num_points; ++i) {
        if (distanceFromPlane(point_mat.row(i), model) < 1.0) { // TODO: hardcoded `t` (1.0)
            inliers.conservativeResize(inliers.rows() + 1, Eigen::NoChange);
            inliers.bottomRows(1) = point_mat.row(i);
        }
    }

    auto [xs_, ys_, zs_] = unzipPoints(inliers);

    auto f = matplot::figure();
    matplot::subplot(2, 2, 0);
    matplot::scatter3(xs, ys, zs, "filled");
    matplot::xlim({-20, 20});
    matplot::ylim({-20, 20});
    matplot::zlim({-10, 40});

    matplot::subplot(2, 2, 1);
    matplot::scatter3(xs, ys, zs, "filled");
    matplot::view(-25, 0, 0);
    matplot::xlim({-20, 20});
    matplot::ylim({-20, 20});
    matplot::zlim({-10, 40});

    matplot::subplot(2, 2, 2);
    matplot::scatter3(xs_, ys_, zs_, "filled");
    matplot::view();
    matplot::xlim({-20, 20});
    matplot::ylim({-20, 20});
    matplot::zlim({-10, 40});

    matplot::subplot(2, 2, 3);
    matplot::scatter3(xs_, ys_, zs_, "filled");
    matplot::view(-25, 0, 0);
    matplot::xlim({-20, 20});
    matplot::ylim({-20, 20});
    matplot::zlim({-10, 40});

    f->width(1920);
    f->height(1080);
    matplot::show();

    return 0;
}
