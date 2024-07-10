/**
 * @file pic.cpp
 * @brief
 * @date 2024-07-10
 * @copyright MIT License
 * @details
 * */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <cassert>

int main( ) {
    for( double d = 0.5; d <= 1.5; d += 0.5 ) {
        for( double p = -2.0 * M_PI / 6.0; p <= 2.0 * M_PI / 6.0; p += M_PI / 6.0 ) {
            for( double q = -2.0 * M_PI / 6.0; q <= 2.0 * M_PI / 6.0; q += M_PI / 6.0 ) {
                double vx = std::cos( p ) * std::cos( q );
                double vy = std::cos( p ) * std::sin( q );
                double vz = std::sin( p );
                double px = d * vx;
                double py = d * vy;
                double pz = d * vz;
                std::cout << std::fixed << std::setprecision( 3 ) << px << " " << py << " " << pz << " " << -0.1 * vx
                          << " " << -0.1 * vy << " " << -0.1 * vz << std::endl;
            }
        }
        std::cout << std::endl << std::endl << std::endl;
    }

    return 0;
}