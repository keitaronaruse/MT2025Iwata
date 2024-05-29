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

//  [ Umin, Umax ), [ Vmin, Vmax ), [ Qmin, Qmax )
//  Workspace size parameters
const double Umin = -3.730, Umax = 0.005, dU = 0.010;
const double Vmin = -1.205, Vmax = 0.005, dV = 0.010;
const double Qmin = -181.5, Qmax = 178.5, dQ = 3.0;
//  Robot velocity parameters
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
    std::tuple< double, double, double > value( ) {
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

double deg2rad( double deg ) { return deg / 180.0 * M_PI; }

int main( ) {
    //  Find a size of configuration space
    state q000( -3.730, -1.205, -181.5 ), qNNN( 0.005, 0.005, 178.5 );
    const int u_num = qNNN.u + 1, v_num = qNNN.v + 1, q_num = qNNN.q;

    //  Cost table: steps from a start state
    const int INF = 1 << 30;
    std::vector< std::vector< std::vector< int > > > cost(
        u_num, std::vector< std::vector< int > >( v_num, std::vector< int >( q_num, INF ) ) );

    state q0( -2.484, 0.000, -90.0 ), q1( -1.242, -0.600, 0.0 );
    std::queue< state > que;
    //  Push a start state
    que.push( q0 );
    //  Cost of a start state is 0
    cost.at( q0.u ).at( q0.v ).at( q0.q ) = 0;
    //  BFS
    while( !que.empty( ) ) {
        state s_curr = que.front( );
        que.pop( );
        if( s_curr == q1 ) {
            //  Goal has been just found
            break;
        }
        auto [ u_curr_val, v_curr_val, q_curr_val ] = s_curr.value( );
        for( auto w : W ) {
            double q_next_val = q_curr_val + dT * w;
            if( q_next_val < Qmin ) {
                q_next_val += 360.0;
            } else if( q_next_val >= Qmax ) {
                q_next_val -= 360.0;
            }
            double u_next_val = u_curr_val + dT * V * std::cos( deg2rad( ( q_curr_val + q_next_val ) / 2.0 ) );
            if( u_next_val < Umin || Umax < u_next_val ) {
                continue;
            }
            double v_next_val = v_curr_val + dT * V * std::sin( deg2rad( ( q_curr_val + q_next_val ) / 2.0 ) );
            if( v_next_val < Vmin || Vmax < v_next_val ) {
                continue;
            }

            state s_next = state( u_next_val, v_next_val, q_next_val );
            if( cost.at( s_next.u ).at( s_next.v ).at( s_next.q ) == INF ) {
                que.push( s_next );
                cost.at( s_next.u ).at( s_next.v ).at( s_next.q ) =
                    cost.at( s_curr.u ).at( s_curr.v ).at( s_curr.q ) + 1;
            }
        }
    }
    std::cout << cost.at( q1.u ).at( q1.v ).at( q1.q ) << std::endl;

    return 0;
}