/**
 * @file iwata-03.cpp
 * @brief Dijkstra's search
 * @date 2024-07-04
 * @copyright MIT License
 * @details priority queue search (Dijkstra's search)
 * */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <tuple>
#include <queue>
#include <algorithm>
#include <cassert>

/**
 * @fn deg2rad
 * @brief convert from degree to radian
 * @param [in] deg an angle in degree
 * @return rad
 * @details
 */
double deg2rad( double deg ) { return M_PI * deg / 180.0; }

//  Constants
//  INF for time
const double INF = 1e6;

//  Data Set 1
//  Range of u-position [ u_min, u_max )
const double d_u = 0.005, u_min = 1.800 - d_u / 2.0, u_max = 2.400 + d_u / 2.0;
//  Range of v-position [ v_min, v_max )
const double d_v = 0.005, v_min = -1.200 - d_v / 2.0, v_max = 1.200 + d_v / 2.0;
//  Range of angle [ q_min, q_max )
const double d_q = M_PI / 60.0, q_min = 0.0 - d_q / 2.0, q_max = 2.0 * M_PI - d_q / 2.0;
//  Start position and angle
const double u_start = 1.800, v_start = 0.000, q_start = deg2rad( 270.0 );
//  Goal position and angle
const double u_goal = 2.400, v_goal = -0.600, q_goal = deg2rad( 0.0 );

// //  Data Set 3
// //  Range of u-position [ u_min, u_max )
// const double d_u = 0.0025, u_min = 2.400 - d_u / 2.0, u_max = 3.600 + d_u / 2.0;
// //  Range of v-position [ v_min, v_max )
// const double d_v = 0.0025, v_min = -1.200 - d_v / 2.0, v_max = 1.200 + d_v / 2.0;
// //  Range of angle [ q_min, q_max )
// const double d_q = M_PI / 60.0, q_min = 0.0 - d_q / 2.0, q_max = 2.0 * M_PI - d_q / 2.0;
// //  Start position and angle
// const double u_start = 2.400, v_start = -0.600, q_start = deg2rad( 0.0 );
// //  Goal position and angle
// const double u_goal = 3.600, v_goal = 0.000, q_goal = deg2rad( 30.0 );

// //  Data Set 4
// //  Range of u-position [ u_min, u_max )
// const double d_u = 0.005, u_min = 3.600 - d_u / 2.0, u_max = 4.800 + d_u / 2.0;
// //  Range of v-position [ v_min, v_max )
// const double d_v = 0.005, v_min = -1.200 - d_v / 2.0, v_max = 1.200 + d_v / 2.0;
// //  Range of angle [ q_min, q_max )
// const double d_q = M_PI / 60.0, q_min = 0.0 - d_q / 2.0, q_max = 2.0 * M_PI - d_q / 2.0;
// //  Start position and angle
// const double u_start = 3.600, v_start = 0.000, q_start = deg2rad( 30.0 );
// //  Goal position and angle
// const double u_goal = 4.800, v_goal = 0.600, q_goal = deg2rad( 0.0 );

//  5.400  0.000 0.161  0.000 -1.000 0.000

/**
 * @fn operator<<
 * @brief stream out of vector<T>
 * @param [in] v vector of T
 * @return ostream
 * @details
 */
template < class T >
std::ostream& operator<<( std::ostream& os, const std::vector< T >& v ) {
    for( auto it = v.begin( ); it != v.end( ); it++ ) {
        os << *it << ( it == --v.end( ) ? "" : " " );
    }
    return os;
}

//  Parameters of robot velocity
//  Robot translational velocity [m/s]
const double V = 0.1;
//  Robot rotational velocity [rad/s]
const std::vector< double > W = { -deg2rad( 30.0 ), 0.0, deg2rad( 30.0 ) };
//  Simulation parameters
const double dT = 0.1;

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

/**
 * @fn u_id
 * @brief convert a position of u to an id of u
 * @param [in] u a position in meter
 * @return id
 * @details requires u_min and d_u
 */
int u_id( double u ) { return ( int ) std::floor( ( u - u_min ) / d_u ); }

/**
 * @fn u_val
 * @brief convert u_id to a position of u
 * @param [in] u_id
 * @return position of u [m]
 * @details requires u_min and d_u
 */
double u_val( int u_id ) { return ( double ) u_id * d_u + u_min + d_u / 2.0; }

/**
 * @fn v_id
 * @brief convert v position to id
 * @param [in] v a position in meter
 * @return id
 * @details requires v_min and d_v
 */
int v_id( double v ) { return ( int ) std::floor( ( v - v_min ) / d_v ); }

/**
 * @fn v_val
 * @brief convert v_id to a position of v
 * @param [in] v_id
 * @return position of v [m]
 * @details requires v_min and d_v
 */
double v_val( int v_id ) { return ( double ) v_id * d_v + ( v_min + d_v / 2.0 ); }

/**
 * @fn q_id
 * @brief convert an angle to id
 * @param [in] q an angle by radian
 * @return id
 * @details requires q_min and d_q, and M_PI in cmath
 */
int q_id( double q ) { return ( int ) std::floor( ( q - q_min ) / d_q ); }

/**
 * @fn q_val
 * @brief convert q_id to an angle of q
 * @param [in] q_id
 * @return an angle of q [rad]
 * @details requires q_min and d_q
 */
double q_val( int q_id ) { return ( double ) q_id * d_q + ( q_min + d_q / 2.0 ); }

int main( ) {
    //  Constants
    const int u_size = u_id( u_max ), v_size = v_id( v_max ), q_size = q_id( q_max );
    std::cerr << u_size << " " << v_size << " " << q_size << std::endl;

    //  Cost table
    std::vector< std::vector< std::vector< double > > > g_cost(
        u_size, std::vector< std::vector< double > >( v_size, std::vector< double >( q_size, INF ) ) );
    //  Cost table
    std::vector< std::vector< std::vector< state > > > prev(
        u_size, std::vector< std::vector< state > >( v_size, std::vector< state >( q_size, { -1, -1, -1 } ) ) );

    //  entry = ( t[s], ( u_id, v_id, q_id ) )
    std::priority_queue< entry, std::vector< entry >, std::greater< entry > > pri_que;
    pri_que.push( { 0.0, { u_id( u_start ), v_id( v_start ), q_id( q_start ) } } );
    g_cost.at( u_id( u_start ) ).at( v_id( v_start ) ).at( q_id( q_start ) ) = 0.0;

    bool is_goal_arrived = false;
    int num_searched = 0;
    while( !pri_que.empty( ) ) {
        auto [ t_curr, s_curr ] = pri_que.top( );
        auto [ u_id_curr, v_id_curr, q_id_curr ] = s_curr;
        double u_curr = u_val( u_id_curr ), v_curr = v_val( v_id_curr ), q_curr = q_val( q_id_curr );
        pri_que.pop( );

        // Check if it arrives at goal
        if( u_id_curr == u_id( u_goal ) && v_id_curr == v_id( v_goal ) && q_id_curr == q_id( q_goal ) ) {
            is_goal_arrived = true;
            break;
        }
        //  An entry which should not to be searched
        if( g_cost.at( u_id_curr ).at( v_id_curr ).at( q_id_curr ) < t_curr ) {
            continue;
        }

        //  Take a rotation speed w out of W
        for( auto w : W ) {
            //  Next state
            double q_next = q_curr + w * dT;
            if( q_next < q_min ) {
                q_next += 2.0 * M_PI;
            } else if( q_max <= q_next ) {
                q_next -= 2.0 * M_PI;
            }
            int q_id_next = q_id( q_next );
            assert( q_min <= q_next && q_next < q_max );
            assert( 0 <= q_id_next && q_id_next < q_size );

            double u_next = u_curr + V * dT * std::cos( ( q_next + q_curr ) / 2.0 );
            double v_next = v_curr + V * dT * std::sin( ( q_next + q_curr ) / 2.0 );
            int u_id_next = u_id( u_next ), v_id_next = v_id( v_next );

            //  Out of workspace
            if( u_id_next < 0 || u_size <= u_id_next || v_id_next < 0 || v_size <= v_id_next ) {
                continue;
            }

            assert( 0 <= u_id_next && u_id_next < 480 );
            assert( 0 <= v_id_next && v_id_next < 960 );

            if( t_curr + 1.0 < g_cost.at( u_id_next ).at( v_id_next ).at( q_id_next ) ) {
                pri_que.push( { t_curr + 1.0, { u_id_next, v_id_next, q_id_next } } );
                g_cost.at( u_id_next ).at( v_id_next ).at( q_id_next ) = t_curr + 1.0;
                prev.at( u_id_next ).at( v_id_next ).at( q_id_next ) = state( u_id_curr, v_id_curr, q_id_curr );
                num_searched++;
            }
        }
    }
    std::cerr << num_searched << std::endl;

    //  Retrieve a path
    int u_id_curr = u_id( u_goal ), v_id_curr = v_id( v_goal ), q_id_curr = q_id( q_goal );
    std::vector< state > path_state;
    while( !( u_id_curr == u_id( u_start ) && ( v_id_curr == v_id( v_start ) ) && q_id_curr == q_id( q_start ) ) ) {
        path_state.push_back( { u_id_curr, v_id_curr, q_id_curr } );
        auto [ u, v, q ] = prev.at( u_id_curr ).at( v_id_curr ).at( q_id_curr );
        u_id_curr = u;
        v_id_curr = v;
        q_id_curr = q;
        assert( 0 <= u_id_curr && u_id_curr < u_size );
        assert( 0 <= v_id_curr && v_id_curr < v_size );
        assert( 0 <= q_id_curr && q_id_curr < q_size );
    }
    path_state.push_back( { u_id_curr, v_id_curr, q_id_curr } );
    std::reverse( path_state.begin( ), path_state.end( ) );
    std::cerr << path_state << std::endl;

    //  Outout a path as ( u, v, du, dv )
    for( const auto& s : path_state ) {
        auto [ u_id_curr, v_id_curr, q_id_curr ] = s;
        double u = u_val( u_id_curr ), v = v_val( v_id_curr ), q = q_val( q_id_curr );
        std::cout << std::fixed << std::setprecision( 3 ) << u << " " << v << " " << V * std::cos( q ) << " "
                  << V * std::sin( q ) << std::endl;
    }

    return 0;
}