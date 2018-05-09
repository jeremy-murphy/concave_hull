
#define BOOST_TEST_MODULE Hemisphere
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "semicircle.hpp"

using namespace boost::geometry;

BOOST_AUTO_TEST_CASE(construct)
{
    typedef model::point<double, 2, cs::cartesian> Point;
    typedef model::segment<Point> Segment;
    typedef model::semicircle<Point> Semicircle;

    Segment const s(Point(0, 0), Point(1, 0));
    Semicircle const a(s, Semicircle::left);

}
