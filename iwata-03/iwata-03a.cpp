/**
 * @file iwata-03a.cpp
 * @brief Dijkstra's search
 * @date 2024-07-06
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
 * @fn operator<< std::pair< S, T >
 * @brief stream out of std::pair< S, T >
 * @param [in] os an output stream
 * @param [in] r a reference of std::pair< S, T >
 * @return a reference of output stream
 * @details
 */
template < class S, class T >
std::ostream& operator<<( std::ostream& os, const std::pair< S, T >& r ) {
    auto [ s, t ] = r;
    os << "( " << s << ", " << t << " )";
    return os;
}

/**
 * @fn operator<< std::tuple< S, T, U >
 * @brief stream out of std::tuple< S, T, U >
 * @param [in] os a reference of output stream
 * @param [in] r a const reference of std::tuple< S, T, U >
 * @return reference of output stream
 * @details
 */
template < class S, class T, class U >
std::ostream& operator<<( std::ostream& os, const std::tuple< S, T, U >& r ) {
    auto [ s, t, u ] = r;
    os << "( " << s << ", " << t << ", " << u << " )";
    return os;
}

/**
 * @fn operator<< std::vector< T >
 * @brief stream out of vector< T >
 * @param [in] os a reference of output stream
 * @param [in] r  a const reference of std::vector< T >
 * @return a reference of output stream
 * @details
 */
template < class T >
std::ostream& operator<<( std::ostream& os, const std::vector< T >& r ) {
    for( auto it = r.begin( ); it != r.end( ); it++ ) {
        os << *it << ( it == --r.end( ) ? "" : " " );
    }
    return os;
}

/**
 * @fn deg2rad
 * @brief convert from degree to radian
 * @param [in] deg an angle in degree
 * @return rad
 * @details
 */
double deg2rad( double deg ) { return M_PI * deg / 180.0; }

//  Parameters of robot velocity
//  Simulation parameters
const double dT = 0.1;
//  Robot translational velocity [m/s]
//  V * dT = 0.01 [m/s]
const double V = 0.1;

//  Robot rotational velocity [rad/s]
//  W * dT = { -3.0 [deg/s ], 0.0 [deg/s], 3.0 [deg/s] }
const std::vector< double > W = { -deg2rad( 30.0 ), 0.0, deg2rad( 30.0 ) };

//  Data Set
//  Offset
const double d_u = 0.005;
const double offset_u = ( std::floor( V / deg2rad( 30.0 ) * 1000.0 ) + 1.0 ) / 1000.0;
//  { Start, Goal }
using uvq = std::tuple< double, double, double >;
const std::vector< std::pair< uvq, uvq > > starts_goals_uvq = {
    { { 1.863, 0.000, deg2rad( 270.0 ) }, { 2.484, -0.600, deg2rad( 0.0 ) } },
    { { 2.484, -0.600, deg2rad( 0.0 ) }, { 3.726, 0.000, deg2rad( 30.0 ) } },
    { { 3.726, 0.000, deg2rad( 30.0 ) }, { 4.968, 0.600, deg2rad( 0.0 ) } },
    { { 4.968, 0.600, deg2rad( 0.0 ) }, { 5.589, 0.000, deg2rad( 270.0 ) } },
    { { 5.589, 0.000, deg2rad( 270.0 ) }, { 4.968, -0.600, deg2rad( 180.0 ) } },
    { { 4.968, -0.600, deg2rad( 180.0 ) }, { 3.726, 0.000, deg2rad( 150.0 ) } },
    { { 3.726, 0.000, deg2rad( 150.0 ) }, { 2.484, 0.600, deg2rad( 180.0 ) } },
    { { 2.484, 0.600, deg2rad( 180.0 ) }, { 1.863, 0.000, deg2rad( 270.0 ) } } };

//  Range of v-position [ v_min, v_max )
const double d_v = 0.005;
const std::pair< double, double > v_range = { -1.200 - d_v / 2.0, 1.200 + 3.0 * d_v / 2.0 };

//  Range of angle [ q_min, q_max )
const double d_q = M_PI / 60.0;
const std::pair< double, double > q_range = { 0.0 - d_q / 2.0, 2.0 * M_PI - d_q / 2.0 };

//  Constants
//  INF for time
const double INF = 1e6;

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
// int u_id( double u ) { return ( int ) std::floor( ( u - u_min ) / d_u ); }

/**
 * @fn u_val
 * @brief convert u_id to a position of u
 * @param [in] u_id
 * @return position of u [m]
 * @details requires u_min and d_u
 */
// double u_val( int u_id ) { return ( double ) u_id * d_u + u_min + d_u / 2.0; }

/**
 * @fn v_id
 * @brief convert v position to id
 * @param [in] v a position in meter
 * @return id
 * @details requires v_min and d_v
 */
// int v_id( double v ) { return ( int ) std::floor( ( v - v_min ) / d_v ); }

/**
 * @fn v_val
 * @brief convert v_id to a position of v
 * @param [in] v_id
 * @return position of v [m]
 * @details requires v_min and d_v
 */
// double v_val( int v_id ) { return ( double ) v_id * d_v + ( v_min + d_v / 2.0 ); }

/**
 * @fn q_id
 * @brief convert an angle to id
 * @param [in] q an angle by radian
 * @return id
 * @details requires q_min and d_q, and M_PI in cmath
 */
int q_id( double q, double q_min, double d_q ) { return ( int ) std::floor( ( q - q_min ) / d_q ); }

/**
 * @fn q_val
 * @brief convert q_id to an angle of q
 * @param [in] q_id
 * @return an angle of q [rad]
 * @details requires q_min and d_q
 */
double q_val( int q_id, double q_min, double d_q ) { return ( double ) q_id * d_q + ( q_min + d_q / 2.0 ); }

int main( ) {
    //  Constants
    std::cerr << offset_u << std::endl;

    for( const auto& [ start_uvq, goal_uvq ] : starts_goals_uvq ) {
        std::cerr << std::fixed << std::setprecision( 3 ) << start_uvq << " " << goal_uvq << std::endl;

        auto [ start_u, start_v, start_q ] = start_uvq;
        auto [ goal_u, goal_v, goal_q ] = goal_uvq;
        double u_min = std::min( start_u, goal_u ) - offset_u, u_max = std::min( start_u, goal_u ) + offset_u;
        //  Range of u-position [ u_min, u_max )
        const std::pair< double, double > u_range = { u_min - d_u / 2.0, u_max + 3.0 * d_u / 2.0 };
        std::cerr << u_range << " " << v_range << " " << q_range << std::endl;
    }

    return 0;
}