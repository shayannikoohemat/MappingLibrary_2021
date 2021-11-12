#include <iostream>
#include "LaserPoints.h"
#include "Plane.h"
#include "lidar_projection.h"
#include <Eigen/Core>
#include <Line3D.h>

using namespace std;

int main()
{
    LaserPoints lpoints;
    lpoints.Read("/mnt/DataPartition/pano2lidar/crop1_pc.laser");
    cout << lpoints.size() << endl;

    double lineseg_len = 20.0;
    double camera_height = 2.39; // not used
    Eigen::Vector2d image_size(4096,2048);

    /// create the true line3d for the target object
    Position3D obj_pos_true(338040.44, 5896253.23, 44.38);
    Position3D image_pos(338038.40041671, 5896258.09347395, 43.706684);
    Vector3D ray_dir_true(obj_pos_true - image_pos);
//                obj_pos_true[0] - image_pos[0],
//                obj_pos_true[1] - image_pos[1],
//                obj_pos_true[2] - image_pos[2]);
    Line3D line3d_true(image_pos, ray_dir_true);
    Position3D pos_inf_true(line3d_true.Position(lineseg_len));
    LineSegment3D lin3d_seg_true(image_pos, pos_inf_true);
    /// create the line to output
    ObjectPoints line_vertices;
    LineTopologies line_top;
    lin3d_seg_true.PointsWithTopology(line_vertices, line_top, 1);

    /// calculate the direction and the line3d from the projection in 3D
    Eigen::Vector2d pixel_coord(2790, 936);
    Eigen::Vector3d dir_eigen;
    dir_eigen =  equirectangle_proj(pixel_coord, image_size);
    // calculate the rotation matrix
    Eigen::Matrix3d rotation;
    rotation = calc_rotation()

    Vector3D ray_dir(dir_eigen.x(), dir_eigen.y(), dir_eigen.z());

    Line3D line3d(image_pos, ray_dir);
    // create a point on the line with a distance of 20 meters to create a line segment
    Position3D pos_inf(line3d.Position(lineseg_len));
    LineSegment3D lin3d_seg(image_pos, pos_inf);
    //cout << "line length: " << lin3d_seg.Length() << "m" << endl;
    lin3d_seg.PointsWithTopology(line_vertices, line_top, 1);

    line_vertices.Write("/mnt/DataPartition/pano2lidar/out/linesegment.objpts");
    line_top.Write("/mnt/DataPartition/pano2lidar/out/linesegment.top", false);

    /// find the points near the predicted line
    LaserPoints object_points;
    for (auto &p : lpoints){
        double p_line_dist;
        //p_line_dist = lin3d_seg.DistanceToPoint();
        p_line_dist = lin3d_seg.Distance(p.Position3DRef());
        if (p_line_dist < 0.50){
            p.Label(3);
            object_points.push_back(p);
        }
    }
    object_points.Write("/mnt/DataPartition/pano2lidar/out/obj_points.laser", false);
    return 0;
}
