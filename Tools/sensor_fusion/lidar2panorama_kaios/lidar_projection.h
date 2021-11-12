#include <Eigen/Core>

int lidar2panorama();

Eigen::Vector3d equirectangle_proj(Eigen::Vector2d pixel_coord, Eigen::Vector2d image_size);

Eigen::Matrix3d calc_rotation(double yaw, double pitch, double roll);
