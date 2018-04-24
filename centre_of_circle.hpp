#ifndef CENTRE_OF_CIRCLE_HPP
#define CENTRE_OF_CIRCLE_HPP

#include <boost/geometry.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <cstdlib>

namespace boost { namespace geometry {

    int determinant_sign(boost::numeric::ublas::permutation_matrix<std ::size_t> const &pm)
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
    T determinant( boost::numeric::ublas::matrix<T> &m )
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
}}

#endif
