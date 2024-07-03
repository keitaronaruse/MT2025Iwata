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
#include <tuple>
#include <queue>
#include <cassert>

template < class T >
std::ostream& operator<<( std::ostream& os, const std::vector< T >& v ) {
    for( auto it = v.begin( ); it != v.end( ); it++ ) {
        os << *it << ( it == --v.end( ) ? "" : " " );
    }
    return os;
}

//  Parameters of robot velocity
const double V = 0.1;
const std::vector< double > W = { -30.0, 0.0, 30.0 };
//  Simulation parameters
const double dT = 1.0;

//  state = ( u_id, v_id, q_id )
using state = std::tuple< int, int, int >;
std::ostream& operator<<( std::ostream& os, const state& s ) {
    auto [ u, v, q ] = s;
    os << "( " << u << ", " << v << ", " << q << " )";
    return os;
}

//  entry = ( t[s], state )
using entry = std::pair< double, state >;
std::ostream& operator<<( std::ostream& os, const entry& e ) {
    auto [ t, s ] = e;
    os << "( " << t << ", " << s << " )";
    return os;
}

//  Constants
//  INF for time
const double INF = 1e6;
//  Range of u-position [ u_min, u_max )
const double d_u = 0.025, u_min = -1.0 - d_u / 2.0, u_max = 1.0 + d_u / 2.0;
//  Range of v-position [ v_min, v_max )
const double d_v = 0.025, v_min = -1.0 - d_v / 2.0, v_max = 1.0 + d_v / 2.0;
//  Range of angle [ q_min, q_max )
const double d_q = M_PI / 60.0, q_min = 0.0 - d_q / 2.0, q_max = 2.0 * M_PI - d_q / 2.0;

/**
 * @fn u_id
 * @brief convert a position of u to an id of u
 * @param [in] u a position in meter
 * @return id
 * @details requires u_min and d_u
 */
int u_id( double u ) {
    return ( int ) std::floor( ( u - u_min ) / d_u );
}

/**
 * @fn u_val
 * @brief convert u_id to a position of u
 * @param [in] u_id
 * @return position of u [m]
 * @details requires u_min and d_u
 */
double u_val( int u_id ) {
    return ( double ) u_id * d_u + u_min + d_u / 2.0;
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
 * @fn v_val
 * @brief convert v_id to a position of v
 * @param [in] v_id
 * @return position of v [m]
 * @details requires v_min and d_v
 */
double v_val( int v_id ) {
    return ( double ) v_id * d_v + ( v_min + d_v / 2.0 );
}

/**
 * @fn q_id
 * @brief convert an angle to id
 * @param [in] q an angle by radian
 * @return id
 * @details requires q_min and d_q, and M_PI in cmath
 */
int q_id( double q ) {
    return ( int ) std::floor( ( q - q_min ) / d_q );
}

/**
 * @fn q_val
 * @brief convert q_id to an angle of q
 * @param [in] q_id
 * @return an angle of q [rad]
 * @details requires q_min and d_q
 */
double q_val( int q_id ) {
    return ( double ) q_id * d_q + ( q_min + d_q / 2.0 );
}

int main( ) {
    //  Constants
    const int u_size = u_id( u_max ), v_size = v_id( v_max ), q_size = q_id( q_max );

    //  Cost table
    std::vector< std::vector< std::vector< double > > > g_cost(
        u_size, std::vector< std::vector< double > >( v_size, std::vector< double >( q_size, INF ) ) );

    //  Start position and angle
    const double u_start = 0.000, v_start = 0.000, q_start = 0.000;
    //  Goal position and angle
    const double u_goal = 0.600, v_goal = 0.400, q_goal = 0.000;

    //  entry = ( t[s], state )
    //  state = ( u_id, v_id, q_id )
    std::priority_queue< entry, std::vector< entry >, std::greater< entry > > pri_que;
    pri_que.push( { 0.0, { u_id( u_start ), v_id( v_start ), q_id( q_start ) } } );
    g_cost.at( u_id( u_start ) ).at( v_id( v_start ) ).at( q_id( q_start ) ) = 0.0;

    while( !pri_que.empty( ) ) {
        auto e = pri_que.top( );
        auto [ t_curr, s_curr ] = e;
        auto [ u_id_curr, v_id_curr, q_id_curr ] = s_curr;
        double u_curr = u_val( u_id_curr );
        double v_curr = v_val( v_id_curr );
        double q_curr = q_val( q_id_curr );
        pri_que.pop( );
        // std::cerr << u_id_curr << " " << v_id_curr << " " << q_id_curr << std::endl;
        // std::cerr << std::fixed << std::setprecision( 6 ) << u_curr << " " << v_curr << " " << q_curr << std::endl;
        // std::cerr << u_id( u_curr ) << " " << v_id( v_curr ) << " " << q_id( q_curr ) << std::endl;
        // std::cerr << std::endl;
        //  Check if it arrives at goal
        if( u_id_curr == u_id( u_goal ) && v_id_curr == v_id( v_goal ) && q_id_curr == q_id( q_goal ) ) {
            break;
        }
        //  An entry not to be searched
        if( g_cost.at( u_id_curr ).at( v_id_curr ).at( q_id_curr ) < t_curr ) {
            continue;
        }
        for( auto w : W ) {
            double q_next = q_curr + w * dT;
            if( q_next < q_min ) {
                q_next += 2.0 * M_PI;
            } else if( q_max <= q_next ) {
                q_next -= 2.0 * M_PI;
            }
            double u_next = u_val( u_id_curr ) + V * dT * std::cos( ( q_next + q_curr ) / 2.0 );
            double v_next = v_val( v_id_curr ) + V * dT * std::sin( ( q_next + q_curr ) / 2.0 );
            //  Out of workspace
            if( u_next < u_min || u_max <= u_next || v_next < v_min || v_max <= v_next ) {
                continue;
            }
            int u_id_next = u_id( u_next );
            int v_id_next = v_id( v_next );
            int q_id_next = q_id( q_next );

            if( t_curr + 1.0 < g_cost.at( u_id_next ).at( v_id_next ).at( q_id_next ) ) {
                pri_que.push( { t_curr + 1.0, { u_id_next, v_id_next, q_id_next } } );
                g_cost.at( u_id_next ).at( v_id_next ).at( q_id_next ) = t_curr + 1.0;
            }
        }
    }

    return 0;
}