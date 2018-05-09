
#define BOOST_TEST_MODULE Concave Hull
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "concave_hull.hpp"
#include "centre_of_circle.hpp"

#include <boost/assign/list_of.hpp>

#include <boost/geometry/algorithms/equals.hpp>

#include <boost/geometry/geometries/geometries.hpp>

#include <boost/geometry/io/wkt/wkt.hpp>

#include <utility>

using namespace boost::geometry;
using boost::assign::list_of;

typedef model::point<double, 2, cs::cartesian> Point;
typedef model::multi_point<Point> MultiPoint;
typedef model::ring<Point> Ring;

typedef std::pair<MultiPoint, Ring> Foo;

struct ConcaveHull
{
    MultiPoint input;
};

BOOST_FIXTURE_TEST_CASE(empty, ConcaveHull)
{
    Ring hull = list_of(Point(0, 0))(Point(0, 1))(Point(1, 1))(Point(1, 0)), hull_copy(hull);
    concave_hull(input, 0, hull);
    BOOST_CHECK(equals(hull, hull_copy));
}

const std::vector<Foo> data = list_of(std::make_pair(MultiPoint(), Ring()));

BOOST_AUTO_TEST_CASE(Circle_Centre)
{
    Point a(1, 1), b(2, 4), c(5, 3);
    Point centre = centre_of_circle(a, b, c);
    BOOST_CHECK(equals(Point(3, 2), centre));
}
