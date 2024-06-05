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
#include <algorithm>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

std::ostream& operator<<( std::ostream& os, const std::vector< Eigen::Vector3d >& plane ) {
    if( plane.empty( ) ) {
        return os;
    }
    for( const auto& v : plane ) {
        os << std::fixed << std::setprecision( 3 ) << v.x( ) << " " << v.y( ) << " " << v.z( ) << std::endl;
    }
    os << std::fixed << std::setprecision( 3 ) << plane.at( 0 ).x( ) << " " << plane.at( 0 ).y( ) << " "
       << plane.at( 0 ).z( ) << std::endl;
    return os;
}

int main( ) {
    const std::vector< Eigen::Vector3d > xyz_plane = {
        Eigen::Vector3d( 0.000, 1.200, 0.000 ), Eigen::Vector3d( 0.000, -1.200, 0.000 ),
        Eigen::Vector3d( 1.200, -1.200, 0.322 ), Eigen::Vector3d( 1.200, 1.200, 0.322 ) };
    std::cout << xyz_plane << std::endl;

    //  Frame transfromation from XYZ to UVW
    const Eigen::Translation3d t_xyz_uvw( 0.000, 0.000, 0.000 );
    const Eigen::Matrix3d R_xyz_uvw( Eigen::AngleAxisd( 15.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY( ) ) );
    const Eigen::Affine3d T_xyz_uvw = t_xyz_uvw * R_xyz_uvw;
    // std::cerr << T_xyz_uvw.translation( ) << std::endl;
    // std::cerr << T_xyz_uvw.rotation( ) << std::endl;
    const Eigen::Translation3d t_uvw_xyz( 0.000, 0.000, 0.000 );
    const Eigen::Matrix3d R_uvw_xyz( Eigen::AngleAxisd( -15.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY( ) ) );
    const Eigen::Affine3d T_uvw_xyz = R_uvw_xyz * t_uvw_xyz;

    std::vector< Eigen::Vector3d > uvw_plane;
    for( const auto& v : xyz_plane ) {
        uvw_plane.push_back( T_xyz_uvw * v );
    }
    std::cout << uvw_plane << std::endl;

    std::vector< Eigen::Vector3d > xyz_plane_2;
    for( const auto& v : uvw_plane ) {
        xyz_plane_2.push_back( T_uvw_xyz * v );
    }
    std::cout << xyz_plane_2 << std::endl;

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