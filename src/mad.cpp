#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "matplot/matplot.h"

int main()
{
    std::vector<double> xs = matplot::rand(990, 150, 175);
    std::vector<double> ys = matplot::rand(5, 225, 255);
    xs.insert(xs.end(), ys.begin(), ys.end());
    ys = matplot::rand(5, 0, 100);
    xs.insert(xs.end(), ys.begin(), ys.end());
    std::vector<double> zs = xs;

    auto f1 = matplot::figure();
    matplot::subplot(2, 2, 0);
    auto b1 = matplot::boxplot(xs);

    matplot::subplot(2, 2, 2);
    auto h1 = matplot::hist(xs);

    size_t mid = zs.size() / 2;
    std::nth_element(zs.begin(), zs.begin() + mid, zs.end());
    double median = zs.at(mid);
    std::cout << "median element is: " << median << std::endl;

    auto mad_fn = [&median](double x) { return std::abs(x - median); };
    std::transform(zs.begin(), zs.end(), zs.begin(), mad_fn);
    std::nth_element(zs.begin(), zs.begin() + mid, zs.end());
    double mad = zs.at(mid);
    std::cout << "Median absolute deviation is: " << mad << std::endl;

    double outlier_bound = 5 * mad;
    std::erase_if(xs, [&](double x) { return std::abs(x - median) > outlier_bound; });

    matplot::subplot(2, 2, 1);
    auto b2 = matplot::boxplot(xs);

    matplot::subplot(2, 2, 3);
    auto h2 = matplot::hist(xs);

    f1->width(1920);
    f1->height(1080);
    matplot::show();

    return 0;
}
