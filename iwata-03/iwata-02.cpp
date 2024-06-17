/**
 * @file iwata-02.cpp
 * @brief Frame transformation
 * @author Keitaro Naruse
 * @date 2024-06-02
 * @copyright MIT License
 * @details */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <cassert>

#include <algorithm>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

std::ostream &operator<<( std::ostream &os, const std::vector< Eigen::Vector3d > &plane ) {
    if( plane.empty( ) ) {
        return os;
    }
    for( const auto &v : plane ) {
        os << std::fixed << std::setprecision( 3 ) << v.x( ) << " " << v.y( ) << " " << v.z( ) << std::endl;
    }
    os << std::fixed << std::setprecision( 3 ) << plane.front( ).x( ) << " " << plane.front( ).y( ) << " "
       << plane.front( ).z( ) << std::endl;
    return os;
}

const std::vector< std::pair< Eigen::Vector3d, Eigen::Vector3d > > uvq_range = {
    { Eigen::Vector3d( 0.000, -1.200, -180.0 ), Eigen::Vector3d( 1.242, 1.200, 180.0 ) } };

//  Origin of rotation in uvw-frame
const std::vector< Eigen::Vector3d > uvw_rot_o = {
    Eigen::Vector3d( 0.000, 0.000, 0.000 ), Eigen::Vector3d( 1.242, 0.000, 0.000 ),
    Eigen::Vector3d( 1.242, 0.000, 0.000 ), Eigen::Vector3d( 2.484, 0.000, 0.000 ),
    Eigen::Vector3d( 2.484, 0.000, 0.000 ), Eigen::Vector3d( 3.726, 0.000, 0.000 ),
    Eigen::Vector3d( 3.726, 0.000, 0.000 ), Eigen::Vector3d( 6.210, 0.000, 0.000 ),
    Eigen::Vector3d( 6.210, 0.000, 0.000 ) };

//  Origin of rotation in xyz-frame
const std::vector< Eigen::Vector3d > xyz_rot_o = {
    Eigen::Vector3d( 0.000, 0.000, 0.000 ), Eigen::Vector3d( 1.200, 0.000, 0.000 ),
    Eigen::Vector3d( 1.200, 0.000, 0.000 ), Eigen::Vector3d( 2.400, 0.000, 0.000 ),
    Eigen::Vector3d( 2.400, 0.000, 0.000 ), Eigen::Vector3d( 3.600, 0.000, 0.00 ),
    Eigen::Vector3d( 3.600, 0.000, 0.000 ), Eigen::Vector3d( 6.000, 0.000, 0.000 ),
    Eigen::Vector3d( 6.000, 0.000, 0.000 ) };

const std::vector< Eigen::Vector3d > uvq_subgoals = {
    Eigen::Vector3d( 1.863, 0.000, -90.0 ), Eigen::Vector3d( 2.484, 0.000, 0.0 ),
    Eigen::Vector3d( 3.726, 0.000, 30.0 ),  Eigen::Vector3d( 4.968, 0.000, 0.0 ),
    Eigen::Vector3d( 5.589, 0.000, -90.0 ), Eigen::Vector3d( 4.968, 0.000, -180.0 ),
    Eigen::Vector3d( 3.726, 0.000, 120.0 ), Eigen::Vector3d( 2.484, 0.000, -90.0 ),
};

bool is_range( const Eigen::Vector3d &p, const Eigen::Vector3d &min, const Eigen::Vector3d &max ) {
    if( min( 0 ) <= p( 0 ) && p( 0 ) < max( 0 ) && min( 1 ) <= p( 1 ) && p( 1 ) < max( 1 ) && min( 2 ) <= p( 2 ) &&
        p( 2 ) < max( 2 ) ) {
        ;
    }

    return true;
}

int main( ) {
    for( const auto &p_uvq : uvq_subgoals ) {
        int idx = -1;
        for( const auto &[ uvw_min, uvw_max ] : uvw_range ) {
            if( uvw_min( 0 ) <= p_uvq( 0 ) && p_uvq( 0 ) < uvw_max( 0 ) ) {
                ;
            };
        }
        assert( idx >= 0 );
    }
    // const std::vector< Eigen::Vector3d > uvw_plane = {
    //     Eigen::Vector3d( 0.000, 1.200, 0.000 ), Eigen::Vector3d( 0.000, -1.200, 0.000 ),
    //     Eigen::Vector3d( 1.242, -1.200, 0.000 ), Eigen::Vector3d( 1.242, 1.200, 0.000 ) };
    // std::cout << uvw_plane << std::endl;

    // Eigen::Translation3d trans( 0.000, 0.000, 0.000 );
    // Eigen::Matrix3d rot( Eigen::AngleAxisd( -15.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY( ) ) );

    // std::vector< Eigen::Vector3d > xyz_plane;
    // for( const auto& v : uvw_plane ) {
    //     xyz_plane.push_back( affine * v );
    // }
    // std::cout << xyz_plane << std::endl;

    return 0;
}