#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <iostream>

using namespace Eigen;

/// you need eigen lib: sudo apt-get install libeigen3-dev
Eigen::Vector3d equirectangle_proj(Eigen::Vector2d pixel_coord, Eigen::Vector2d image_size) {


    std::cout << "pixel coordinate :\n" << pixel_coord << std::endl;
    std::cout << "image_size :\n" << image_size << std::endl;

    
    Vector2d normalized_coords;
    normalized_coords[0]=(pixel_coord[0] - (image_size[0]/2.0)) / image_size[0];
    normalized_coords[1]=(pixel_coord[1] - (image_size[1]/2.0)) / image_size[1];

    std::cout << "lon lat (normalized):\n" << normalized_coords << std::endl;

    Vector2d lon_lat;
    lon_lat[0]= normalized_coords[0]*(EIGEN_PI);
    lon_lat[1]= normalized_coords[1]*(-EIGEN_PI / 2.0);

    std::cout << "lon lat (in radians):\n" << lon_lat << std::endl;

    /// direction in car coordinate system
    Vector3d direction = Vector3d(
       -cos(lon_lat[1])*sin(lon_lat[0]),
        -cos(lon_lat[1])*cos(lon_lat[0]), sin(lon_lat[1]));

    std::cout << "direction vector :\n" << direction << std::endl;

    return direction;

}

Eigen::Matrix3d calc_rotation(double yaw, double pitch, double roll){
    /// direction in world coord system
   // direction * rotation matrix.transpose!
   // pan=heading=yaw=z tilt=pitch=y roll=roll=x (right-hand side)
    // yaw = y pitch =x, roll=z (openGL)
   // order: yaw-pitch-roll (from applanix)
       const auto to_deg = (180.0 / EIGEN_PI);  //from Rad to Deg
       const auto to_rad = (EIGEN_PI / 180.0);  //from Deg to Rad

       Matrix3d rotation; // rotation matrix
       Vector3d YawPitchRoll = {yaw, pitch, roll}; //{128.1, 1.4, 0.8};
       Vector3d UpVector = {1.0, 0.0, 0.0};
       Vector3d DirVector = {0.0, 0.0, 1.0};

       std::cout << "Yaw, Pitch, Roll :\n" << YawPitchRoll << '\n';
       std::cout << "Up Vector :\n" << UpVector << '\n';
       std::cout << "Direction Vector :\n" << DirVector << '\n';
       std::cout << '\n';
       std::cout <<  "Importing the Applanix export into Orbit      \n";
       std::cout <<  "Rotation Order is Yaw * Pitch * negative Roll \n";
       rotation = AngleAxisd(YawPitchRoll[0] * to_rad, Vector3d::UnitZ()) *
                  AngleAxisd(YawPitchRoll[1] * to_rad, Vector3d::UnitY()) *
                  AngleAxisd(-YawPitchRoll[2] * to_rad, Vector3d::UnitX());
       // openGL
//       rotation = AngleAxisd(YawPitchRoll[0] * to_rad, Vector3d::UnitY()) *
//                  AngleAxisd(YawPitchRoll[1] * to_rad, Vector3d::UnitX()) *
//                  AngleAxisd(-YawPitchRoll[2] * to_rad, Vector3d::UnitZ());

       std::cout << "Rotation matrix :\n" << rotation << "\n\n";

       std::cout << "Rotation matrix * Up Vector :\n" << (rotation * UpVector) << "\n\n";

       return rotation;
}
