#include "concave_hull.hpp"

#include <boost/geometry/geometries/geometries.hpp>

#include <iostream>

using namespace boost::geometry;

int main(int argc, char **argv)
{
    typedef model::point<double, 2, cs::cartesian> Point;
    model::ring<Point> foo;
    model::polygon<Point> bar;
    bar.outer() = foo;
}
