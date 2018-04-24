#include "concave_hull.hpp"
#include "centre_of_circle.hpp"

#include <boost/assign/list_of.hpp>

#include <boost/geometry/algorithms/equals.hpp>

#include <boost/geometry/geometries/geometries.hpp>

#include <boost/geometry/io/wkt/wkt.hpp>

#include <gtest/gtest.h>

#include <utility>

using namespace boost::geometry;
using boost::assign::list_of;

typedef model::point<double, 2, cs::cartesian> Point;
typedef model::multi_point<Point> MultiPoint;
typedef model::ring<Point> Ring;

typedef std::pair<MultiPoint, Ring> Foo;

struct ConcaveHull : ::testing::TestWithParam<Foo>
{
    MultiPoint input;
};

TEST_P(ConcaveHull, empty)
{
    Ring hull = list_of(Point(0, 0))(Point(0, 1))(Point(1, 1))(Point(1, 0)), hull_copy(hull);
    concave_hull(input, 0, hull);
    EXPECT_TRUE(equals(hull, hull_copy));
}

const std::vector<Foo> data = list_of(std::make_pair(MultiPoint(), Ring()));

INSTANTIATE_TEST_CASE_P(Bar, ConcaveHull, ::testing::ValuesIn(data));

TEST(Circle, Centre)
{
    Point a(1, 1), b(2, 4), c(5, 3);
    Point centre = centre_of_circle(a, b, c);
    EXPECT_TRUE(equals(Point(3, 2), centre));
}
