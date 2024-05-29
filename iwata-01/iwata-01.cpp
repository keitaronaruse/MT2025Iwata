/**
 * @file iwata-01.cpp
 * @brief BFS (breadth first search) of a single point
 * @author Keitaro Naruse
 * @date 2024-05-29
 * @copyright MIT License
 * @details
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <queue>
#include <tuple>

template < class A, class B, class C >
std::ostream& operator<<( std::ostream& os, const std::tuple< A, B, C >& t ) {
    auto [ a, b, c ] = t;
    os << "(" << a << "," << b << "," << c << ")";
    return os;
}

const double dU = 0.010, dV = 0.010, dQ = 3.0;
//  [ Umin, Umax ), [ Vmin, Vmax ), [ Qmin, Qmax )
const double Umin = -3.735, Umax = 3.735, Vmin = -1.205, Vmax = 1.205, Qmin = -181.5, Qmax = 178.5;

//  Find an id for u, v, and q
int u_id( double u ) { return ( int ) std::floor( ( u - Umin ) / dU ); }
int v_id( double v ) { return ( int ) std::floor( ( v - Vmin ) / dV ); }
int q_id( double q ) { return ( int ) std::floor( ( q - Qmin ) / dQ ); }

int main( ) {
    std::tuple< int, int, int > state_q0 = { u_id( -2.484 ), v_id( 0.0 ), q_id( -90.0 ) },
                                state_q1 = { u_id( -1.242 ), v_id( -0.600 ), q_id( 0.0 ) };

    std::queue< std::tuple< int, int, int > > que;
    que.push( state_q0 );
    que.push( state_q1 );
    while( !que.empty( ) ) {
        auto q = que.front( );
        que.pop( );
        std::cout << q << std::endl;
    }

    return 0;
}