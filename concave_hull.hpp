#ifndef CONCAVE_HULL_HPP
#define CONCAVE_HULL_HPP

#include <boost/bind.hpp>

#include <boost/geometry.hpp>
/*
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/is_empty.hpp>

#include <boost/geometry/strategies/strategies.hpp>
*/

#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <boost/heap/priority_queue.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <cassert>

namespace boost { namespace geometry {

namespace detail {

    int determinant_sign(const boost::numeric::ublas::permutation_matrix<std ::size_t>& pm)
    {
        namespace bnu = boost::numeric::ublas;

        int pm_sign = 1;
        std::size_t size = pm.size();
        for (std::size_t i = 0; i < size; ++i)
            if (i != pm(i))
                pm_sign = -pm_sign; // swap_rows would swap a pair of rows here, so we change sign
        return pm_sign;
    }

    template <typename T>
    T determinant( boost::numeric::ublas::matrix<T>& m )
    {
        namespace bnu = boost::numeric::ublas;

        bnu::permutation_matrix<std ::size_t> pm(m.size1());
        T det;
        if( bnu::lu_factorize(m, pm) )
            det = 0.0;
        else
        {
            det = determinant_sign( pm );
            for(typename boost::numeric::ublas::matrix<T>::size_type i = 0; i < m.size1(); i++)
                det *= m(i,i); // multiply by elements on diagonal
        }
        return det;
    }

    template <typename Point>
    Point centre_of_circle(Point const &a, Point const &b, Point const &c)
    {
        typedef typename coordinate_type<Point>::type Coordinate;
        boost::numeric::ublas::matrix<Coordinate> M11(3, 3), M12(3, 3), M13(3, 3);
        // Initialize 3rd column in all matrices to ones.
        for (int i = 0; i != 3; i++)
            M11(i, 2) = M12(i, 2) = M13(i, 2) = 1;
        // Initialize first column.
        M12(0, 0) = M13(0, 0) = get<0>(a) * get<0>(a) + get<1>(a) * get<1>(a);
        M12(1, 0) = M13(1, 0) = get<0>(b) * get<0>(b) + get<1>(b) * get<1>(b);
        M12(2, 0) = M13(2, 0) = get<0>(c) * get<0>(c) + get<1>(c) * get<1>(c);
        // Initialize second column of M11 and M12.
        M11(0, 1) = M12(0, 1) = get<1>(a);
        M11(1, 1) = M12(1, 1) = get<1>(b);
        M11(2, 1) = M12(2, 1) = get<1>(c);
        // Initialize first column of M11 and second column of M13.
        M11(0, 0) = M13(0, 1) = get<0>(a);
        M11(1, 0) = M13(1, 1) = get<0>(b);
        M11(2, 0) = M13(2, 1) = get<0>(c);

        Coordinate detM11 = determinant(M11);
        Coordinate detM12 = determinant(M12);
        Coordinate detM13 = determinant(M13);

        return Point( detM12 / detM11 / Coordinate(2), detM13 / detM11 / -Coordinate(2));
    }
    
    /**
     * @brief Determine whether a geometry is within the area enclosed by a ring.
     * 
     */
    template <typename Geometry, typename Ring>
    bool within_ring(Geometry const& geometry, Ring const& hull)
    {
        typedef typename point_type<Ring>::type Point;
        model::polygon<Point> solid_hull;
        solid_hull.outer() = hull;
        return geometry::within(geometry, solid_hull);
    }
    
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
    
    template <typename Index, typename Segment>
    edge_survey<typename Index::value_type> 
    survey(const Index &points, const Segment &edge)
    {
        typedef typename Index::value_type Point;
        edge_survey<typename Index::value_type> result;
        Point p_nearest = *qbegin(points, index::nearest(edge, 1));
        Point p_next;

        // The loop with the sad worst-case complexity.
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

    assert(detail::within_ring(input, hull));

    typedef typename point_type<Geometry>::type InputPoint;
    typedef typename point_type<OutputGeometry>::type HullPoint;
    typedef detail::edge_survey<InputPoint> EdgeSurvey;
    typedef index::rtree< InputPoint, index::rstar<16> > PointIndex;
    PointIndex I_P(input);
    
    typedef heap::priority_queue<EdgeSurvey> SurveyPriorityQueue;
    SurveyPriorityQueue Q;
    typedef model::referring_segment<HullPoint> ReferringSegment;
    
    for_each_segment(hull, boost::bind(&SurveyPriorityQueue::push, &Q, 
        bind(detail::survey<PointIndex, ReferringSegment>, I_P, _1)));
    
    
}

}}

#endif
