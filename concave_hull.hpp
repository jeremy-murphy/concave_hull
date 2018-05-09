#ifndef CONCAVE_HULL_HPP
#define CONCAVE_HULL_HPP

#include "semicircle.hpp"
#include "centre_of_circle.hpp"

#include <boost/bind.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <boost/heap/priority_queue.hpp>

#include <cassert>

namespace boost { namespace geometry {

namespace detail {

    template <typename Point>
    struct edge_survey
    {
        Point p;
        double survey;
        
        friend
        bool operator==(const edge_survey &x, const edge_survey &y)
        {
            return x.p == y.p && x.survey == y.survey;
        }
        
        friend
        bool operator!=(const edge_survey &x, const edge_survey &y)
        {
            return !(x == y);
        }
        
        friend
        bool operator<(const edge_survey &x, const edge_survey &y)
        {
            return x.survey < y.survey;
        }
        
        // etc
    };
    


    /**
     * @brief Survey an edge wrt an indexed point set.
     *
     * This function is only defined when there is a point that satisfies.
     *
     */
    template <typename Index, typename Segment>
    edge_survey<typename Index::value_type> 
    survey(Index const &points, Segment const &edge)
    {
        typedef typename Index::value_type Point;
        edge_survey<Point> result;
        typedef model::semicircle<typename point_type<Segment>::type> Hemisphere;
        Hemisphere const inner_hs(edge, Hemisphere::right);
        Point p_nearest = *qbegin(points,
                                  index::intersects(inner_hs) // NOTE: Current cause of pain.
                                  && index::nearest(edge, 1));
        Point p_next;

        // The information generated by this loop could be saved for later reuse.
        // It may, however, not be worth it.
        do
        {
            Point centre = centre_of_circle(edge.first, edge.second, p_nearest);
            p_next = *qbegin(points, index::nearest(centre, 1));
        } while(!geometry::equals(p_next, p_nearest));

        return result;
    }


}

template <typename Geometry, typename OutputGeometry>
void concave_hull(Geometry const& input, int N, OutputGeometry& hull)
{
    if (is_empty(input))
        return;

    assert(within(input, hull));

    typedef typename point_type<Geometry>::type InputPoint;
    typedef typename point_type<OutputGeometry>::type HullPoint;
    typedef detail::edge_survey<InputPoint> EdgeSurvey;
    typedef index::rtree< InputPoint, index::rstar<16> > PointIndex;
    PointIndex I_P(input);
    
    typedef heap::priority_queue<EdgeSurvey> SurveyPriorityQueue;
    SurveyPriorityQueue Q;
    typedef model::referring_segment<HullPoint> ReferringSegment;

    // TODO: Refactor the binds away.
    for_each_segment(hull, bind(&SurveyPriorityQueue::push, &Q,
        bind(detail::survey<PointIndex, ReferringSegment>, I_P, _1)));
    
}

}}

#endif
