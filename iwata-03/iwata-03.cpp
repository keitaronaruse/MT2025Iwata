/**
 * @file iwata-03.cpp
 * @brief Dijkstra's search
 * @date 2024-07-02
 * @copyright MIT License
 * @details */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <cassert>

template < class T >
std::ostream& operator<<( std::ostream& os, const std::vector< T >& v ) {
    for( auto it = v.begin( ); it != v.end( ); it++ ) {
        os << *it << ( it == --v.end( ) ? "" : " " );
    }
    return os;
}

//  Range of u-position [ u_min : d_u : u_max ]
const double d_u = 0.025, u_min = -1.0 - d_u / 2.0, u_max = 1.0 + d_u / 2.0;
//  Range of v-position [ v_min : d_v : v_max ]
const double d_v = 0.025, v_min = -1.0 - d_v / 2.0, v_max = 1.0 + d_v / 2.0;
//  Range of angle [ q_min : d_q : q_max ]
const double d_q = M_PI / 60.0, q_min = -M_PI - d_q / 2.0, q_max = M_PI - d_q / 2.0;

/**
 * @fn u_id
 * @brief convert u position to id
 * @param [in] u a position in meter
 * @return id
 * @details requires u_min and d_u
 */
int u_id( double u ) {
    return ( int ) std::floor( ( u - u_min ) / d_u );
}

/**
 * @fn v_id
 * @brief convert v position to id
 * @param [in] v a position in meter
 * @return id
 * @details requires v_min and d_v
 */
int v_id( double v ) {
    return ( int ) std::floor( ( v - v_min ) / d_v );
}

/**
 * @fn q_id
 * @brief convert an angle to id
 * @param [in] a an angle by radian
 * @return id
 * @details requires q_min and d_q, and M_PI in cmath
 */
int q_id( double q ) {
    return ( int ) std::floor( ( q - q_min ) / d_q );
}

int main( ) {
    // for( int u = -1000; u <= 1000; u += 25 ) {
    //     double u_d = ( double ) u / 1000.0;
    //     std::cerr << std::fixed << std::setprecision( 3 ) << u_d << ": " << u_id( u_d ) << std::endl;
    // }
    // for( int v = -1000; v <= 1000; v += 25 ) {
    //     double v_d = ( double ) v / 1000.0;
    //     std::cerr << std::fixed << std::setprecision( 3 ) << v_d << ": " << v_id( v_d ) << std::endl;
    // }
    // for( int q = -180; q < 180; q += 3 ) {
    //     double q_d = ( double ) q, r = M_PI * q / 180.0;
    //     std::cerr << std::fixed << std::setprecision( 3 ) << q << " == " << r << ": " << q_id( r ) << std::endl;
    // }
    const int u_size = u_id( u_max ), v_size = v_id( v_max ), q_size = q_id( q_max );
    std::cerr << u_size << " " << v_size << " " << q_size << std::endl;

    const double INF = 1e6;
    // std::cerr << INF << std::endl;
    std::vector< std::vector< std::vector< double > > > f_cost(
        u_size, std::vector< std::vector< double > >( v_size, std::vector< double >( q_size, INF ) ) );
    for( int u = 0; u < u_size; u++ ) {
        for( int v = 0; v < v_size; v++ ) {
            for( int q = 0; q < q_size; q++ ) {
                f_cost.at( u ).at( v ).at( q ) = 0.0;
            }
        }
    }
    return 0;
}