#ifndef GEOMETRY_HEMISPHERE_HPP
#define GEOMETRY_HEMISPHERE_HPP

#include <boost/geometry/geometries/box.hpp>

namespace boost { namespace geometry {

namespace model {
    template <typename Point>
    struct semicircle;
}

template <typename Point, typename HemispherePoint>
inline bool intersects(Point const& p, model::semicircle<HemispherePoint> const& hs)
{
    return hs.intersects_point(p);
}

template <typename Point, typename HemispherePoint>
inline bool intersects(model::box<Point> const& b, model::semicircle<HemispherePoint> const& hs)
{
    return hs.intersects_box(b);
}

}}

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>


namespace boost { namespace geometry {
namespace model {

template <typename Point>
struct semicircle
{
    enum side_t {left, right};

    template <typename Segment>
    semicircle(Segment const &s, side_t side)
        : s(s.first, s.second), side(side == right ? 1 : -1) {}

    template <typename OtherPoint>
    bool intersects_point(OtherPoint const &p) const
    {
        return strategy::side::side_by_triangle<>::apply(s.first, s.second, p) == side
        && geometry::distance(return_centroid<Point>(s), p) <= geometry::length(s) / 2;
    }


    template <typename OtherSegment>
    bool intersects_segment(OtherSegment const &os) const
    {
        return (strategy::side::side_by_triangle<>::apply(s.first, s.second, os.first) == side
        || strategy::side::side_by_triangle<>::apply(s.first, s.second, os.second) == side)
        && geometry::distance(return_centroid<Point>(s), os) <= geometry::length(s) / 2;
    }

    template <typename Box>
    bool intersects_box(Box const &b) const
    {
        if (geometry::intersects(s, b))
            return true;
        typedef typename point_type<Box>::type BoxPoint;
        typedef segment<BoxPoint> BoxSegment;
        BoxSegment bs(b.min_corner(),
                        BoxPoint(get<min_corner, 0>(b), get<max_corner, 1>(b)));
        if (intersects_segment(bs))
            return true;
        bs.first = b.max_corner();
        if (intersects_segment(bs))
            return true;
        set<0>(bs.second, get<max_corner, 0>(b));
        set<1>(bs.second, get<min_corner, 1>(b));
        if (intersects_segment(bs))
            return true;
        bs.first = b.min_corner();
        if (intersects_segment(bs))
            return true;
        return false;
    }

private:
    model::segment<Point> s;
    short side;
};

}

struct semicircle_tag : single_tag, areal_tag {};

namespace traits
{
    template <typename Point>
    struct tag<model::semicircle<Point> >
    {
        typedef semicircle_tag type;
    };

    template <typename Point>
    struct point_type<model::semicircle<Point> >
    {
        typedef Point type;
    };
}

}}

#endif

