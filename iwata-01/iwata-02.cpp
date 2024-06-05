/**
 * @file iwata-02.cpp
 * @brief BFS (breadth first search) of a single point
 * @author Keitaro Naruse
 * @date 2024-05-30
 * @copyright MIT License
 * @details
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <queue>
#include <tuple>
#include <algorithm>

//  [ Umin, Umax ), [ Vmin, Vmax ), [ Qmin, Qmax )
//  Parameters of workspace and cell size
//  Zone 0, 1, 2
//  Precise and slow
const std::vector< std::tuple< double, double, double > > Urange = { { -3.737, -1.241, 0.002 } };
const std::vector< std::tuple< double, double, double > > Vrange = { { -1.201, -1.201, 0.002 } };
const std::vector< std::tuple< double, double, double > > Qrange = { { -180.75, -179.25, 1.5 } };

//  Unprecise and fast
// const double Umin = -3.735, Umax = 0.005, dU = 0.010;
// const double Vmin = -1.205, Vmax = 0.005, dV = 0.010;
// const double Qmin = -181.5, Qmax = 178.5, dQ = 3.0;
const double Umin = -3.737, Umax = -1.241, dU = 0.002;
const double Vmin = -1.201, Vmax = 1.201, dV = 0.002;
const double Qmin = -180.75, Qmax = 179.25, dQ = 1.5;

//  Parameters of robot velocity
const double V = 0.1;
const std::vector< double > W = { -30.0, 0.0, 30.0 };
//  Simulation parameters
const double dT = 0.1;

struct state {
    int u;
    int v;
    int q;
    state( ) : u( 0 ), v( 0 ), q( 0 ) {}
    state( int _u, int _v, int _q ) : u( _u ), v( _v ), q( _q ) {}
    state( double u_val, double v_val, double q_val )
        : u( ( int ) std::floor( ( u_val - Umin ) / dU ) ),
          v( ( int ) std::floor( ( v_val - Vmin ) / dV ) ),
          q( ( int ) std::floor( ( q_val - Qmin ) / dQ ) ){ };
    std::tuple< double, double, double > value( ) const {
        std::tuple< double, double, double > t;
        double u_val = ( double ) u * dU + Umin + dU / 2.0;
        double v_val = ( double ) v * dV + Vmin + dV / 2.0;
        double q_val = ( double ) q * dQ + Qmin + dQ / 2.0;
        return std::tuple< double, double, double >( u_val, v_val, q_val );
    }
};

bool operator==( const state& a, const state& b ) { return a.u == b.u && a.v == b.v && a.q && b.q; }

std::ostream& operator<<( std::ostream& os, const state& s ) {
    os << "(" << s.u << "," << s.v << "," << s.q << ")";
    return os;
}

template < class T >
std::ostream& operator<<( std::ostream& os, const std::vector< T >& v ) {
    for( auto it = v.begin( ); it != v.end( ); it++ ) {
        os << *it << ( it == --v.end( ) ? "" : " " );
    }
    return os;
}

double deg2rad( double deg ) { return deg / 180.0 * M_PI; }

int main( ) {
    //  Find a size of configuration space
    state q000( Umin, Vmin, Qmin ), qNNN( Umax, Vmax, Qmax );
    const int u_num = qNNN.u + 1, v_num = qNNN.v + 1, q_num = qNNN.q;

    //  Cost table: steps from a start state
    const int INF = 1 << 30;
    std::vector< std::vector< std::vector< int > > > cost(
        u_num, std::vector< std::vector< int > >( v_num, std::vector< int >( q_num, INF ) ) );
    std::vector< std::vector< std::vector< state > > > prev(
        u_num, std::vector< std::vector< state > >( v_num, std::vector< state >( q_num, state( -1, -1, -1 ) ) ) );

    //  BFS::initailize
    std::queue< state > que;
    //  Start: s_state, Goal: g_state
    const state s_state( -2.484, 0.000, -90.0 ), g_state( -1.242, -0.600, 0.0 );
    //  Push a start state
    que.push( s_state );
    //  Cost of a start state is 0
    cost.at( s_state.u ).at( s_state.v ).at( s_state.q ) = 0;
    //  BFS
    while( !que.empty( ) ) {
        //  Current state
        state s_curr = que.front( );
        //  Pop queue
        que.pop( );
        //  If goal is found, break out of loop
        if( s_curr == g_state ) {
            break;
        }
        //  Calculate a typical position of u-v and orientation q of a current state
        auto [ u_curr_val, v_curr_val, q_curr_val ] = s_curr.value( );
        //  Take all the possible actions
        for( auto w : W ) {
            //  Orientation at the next state
            double q_next_val = q_curr_val + dT * w;
            //  Normalization of orientation
            if( q_next_val < Qmin ) {
                q_next_val += 360.0;
            } else if( q_next_val >= Qmax ) {
                q_next_val -= 360.0;
            }
            //  U-position at the next state
            double u_next_val = u_curr_val + dT * V * std::cos( deg2rad( ( q_curr_val + q_next_val ) / 2.0 ) );
            //  Skip it if it is out of workspace
            if( u_next_val < Umin || Umax < u_next_val ) {
                continue;
            }
            //  V-position at the next state
            double v_next_val = v_curr_val + dT * V * std::sin( deg2rad( ( q_curr_val + q_next_val ) / 2.0 ) );
            //  Skip it if it is out of workspace
            if( v_next_val < Vmin || Vmax < v_next_val ) {
                continue;
            }
            //  Next state
            state s_next = state( u_next_val, v_next_val, q_next_val );
            //  If it is visited at first time, push it to que and update a cost
            if( cost.at( s_next.u ).at( s_next.v ).at( s_next.q ) == INF ) {
                que.push( s_next );
                cost.at( s_next.u ).at( s_next.v ).at( s_next.q ) =
                    cost.at( s_curr.u ).at( s_curr.v ).at( s_curr.q ) + 1;
                prev.at( s_next.u ).at( s_next.v ).at( s_next.q ) = s_curr;
            }
        }
    }
    // std::cout << "Arrived at goal int steps of " << cost.at( g_state.u ).at( g_state.v ).at( g_state.q ) <<
    // std::endl;

    //  Find a path of states
    std::vector< state > path_state;
    auto c_state = g_state;
    while( cost.at( c_state.u ).at( c_state.v ).at( c_state.q ) != 0 ) {
        path_state.push_back( c_state );
        auto p_state = prev.at( c_state.u ).at( c_state.v ).at( c_state.q );
        c_state = p_state;
    }
    path_state.push_back( c_state );
    std::reverse( path_state.begin( ), path_state.end( ) );
    // std::cout << path_state << std::endl;

    //  Find a path of positions and orientations
    for( const auto& s : path_state ) {
        auto [ u, v, q ] = s.value( );
        std::cout << std::fixed << std::setprecision( 3 ) << u << " " << v << " " << V << " " << q << std::endl;
    }

    return 0;
}