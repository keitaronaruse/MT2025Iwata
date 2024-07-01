/**
 * @file iwata-02.cpp
 * @brief Frame transformation
 * @author Keitaro Naruse
 * @date 2024-06-16
 * @copyright MIT License
 * @details
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <cassert>
#include <algorithm>

#include "Eigen/Core"
#include "Eigen/Geometry"

/**
 * @fn deg2rad
 * @brief convert from degree to radian
 * @param [in] deg an angle by degree
 * @return radian an angle by radian
 * @details requires M_PI in cmath
 */
double deg2rad( double deg ) {
    return deg / 180.0 * M_PI;
}

/**
 * @fn operator<<
 * @brief output a 3D-vector to a given output stream
 * @param [in] os output stream
 * @param [in] p Eigen::Vector3d object
 * @return os output stream
 * @details output is formatted
 */
std::ostream &operator<<( std::ostream &os, const Eigen::Vector3d &p ) {
    os << std::fixed << std::showpos << std::setprecision( 3 ) << "( " << p( 0 ) << ", " << p( 1 ) << ", " << p( 2 )
       << " )";
    return os;
}

/**
 * @fn operator<<
 * @brief output a 3*3 matrix to a given output stream
 * @param [in] os output stream
 * @param [in] m Eigen::Matrix3d object
 * @return os output stream
 * @details output is formatted
 */
std::ostream &operator<<( std::ostream &os, const Eigen::Matrix3d &m ) {
    os << std::fixed << std::showpos << std::setprecision( 3 ) << "[ " << m( 0, 0 ) << " " << m( 0, 1 ) << " "
       << m( 0, 2 ) << std::endl
       << "  " << m( 1, 0 ) << " " << m( 1, 1 ) << " " << m( 1, 2 ) << std::endl
       << "  " << m( 2, 0 ) << " " << m( 2, 1 ) << " " << m( 2, 2 ) << " ]" << std::endl;
    return os;
}

int main( ) {
    //  Translation in uvw-frame
    //  uvw座標系での並進変換
    Eigen::Translation3d t_uvw( -1.242, 0.000, 0.000 );
    // std::cerr << "t_uvw" << std::endl;
    // std::cerr << t_uvw.translation( ) << std::endl << std::endl;

    //  Rotation from uvw-fame to xyz-frame
    //  uvw座標系からxyz座標系への回転変換
    Eigen::Matrix3d R_uvw_xyz;
    R_uvw_xyz = Eigen::AngleAxisd( deg2rad( 15.0 ), Eigen::Vector3d::UnitY( ) );
    // std::cerr << "R_uvw_xyz" << std::endl;
    // std::cerr << R_uvw_xyz << std::endl << std::endl;

    //  Translation in xyz-frame
    //  xyz座標系での並進変換
    Eigen::Translation3d t_xyz( 1.200, 0.000, 0.000 );
    // std::cerr << "t_xyz" << std::endl;
    // std::cerr << t_xyz.translation( ) << std::endl << std::endl;

    //  Affine transform from uvw-frame to xyz-frame
    //  translation in uvw-frame -> rotation from uvw-frame to xyz-frame -> translation in xyz-frame
    //  uvw座標系からxyz座標系へのアフィン変換
    //  uvw座標系で並進変換 -> uvw座標系からxyz座標系へ回転変換 -> xyz座標系で並進変換
    Eigen::Affine3d A_uvw_xyz;
    A_uvw_xyz = t_xyz * R_uvw_xyz * t_uvw;
    // std::cerr << "A_uvw_xyz translation" << std::endl;
    // std::cerr << A_uvw_xyz.translation( ) << std::endl << std::endl;
    // std::cerr << "A_uvw_xyz rotations" << std::endl;
    // std::cerr << A_uvw_xyz.rotation( ) << std::endl << std::endl;

    //  Four corner points in uvw-frame
    //  uvw座標系で表現されたの４つの頂点
    const std::vector< Eigen::Vector3d > P_uvw = {
        Eigen::Vector3d( 0.621, -1.200, 0.000 ), Eigen::Vector3d( 0.621, 1.200, 0.000 ),
        Eigen::Vector3d( 1.242, 1.200, 0.000 ), Eigen::Vector3d( 1.242, -1.200, 0.000 ) };

    //  Print a position of the four corner points both in uvw-frame and xyz-frame
    //  ４つの頂点のuvw座標系とxyz座標系での位置を出力
    for( const auto &p_uvw : P_uvw ) {
        std::cerr << "uvw: " << p_uvw << ", " << "xyz: " << A_uvw_xyz * p_uvw << std::endl;
    }

    //  Print a position of the four corner points both in uvw-frame and xyz-frame
    const std::vector< Eigen::Matrix3d > R_uvw = {
        Eigen::AngleAxisd( deg2rad( -90.0 ), Eigen::Vector3d::UnitZ( ) ).toRotationMatrix( ),
        Eigen::AngleAxisd( deg2rad( 0.0 ), Eigen::Vector3d::UnitZ( ) ).toRotationMatrix( ),
        Eigen::AngleAxisd( deg2rad( 90.0 ), Eigen::Vector3d::UnitZ( ) ).toRotationMatrix( ),
        Eigen::AngleAxisd( deg2rad( -180.0 ), Eigen::Vector3d::UnitZ( ) ).toRotationMatrix( ) };
    for( const auto &r_uvw : R_uvw ) {
        std::cerr << r_uvw << std::endl;
    };
    return 0;
}