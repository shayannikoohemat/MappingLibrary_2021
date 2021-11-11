#include <iostream>
#include "planar_ransac.h"
#include <CGAL/Shape_detection/Efficient_RANSAC.h>


int main(){

    std::string ascii_in="/mnt/DataPartition/threed_modeling/test.txt";
    std::string ascii_out="/mnt/DataPartition/threed_modeling/out/points_normal_segment.txt";
    //calculate_normal_wrapper(ascii_in, ascii_out);
    double  probability     = 0.05 ;    // Set probability to miss the largest primitive at each iteration.
    int     min_points      = 10  ;    // Detect shapes with at least n-min points.
    double  epsilon         = 0.002 ;    // Set maximum Euclidean distance between a point and a shape.
    double  cluster_epsilon = 0.08;     //0.08 ;  // Set maximum Euclidean distance between points to be clustered.
    double  normal_thresh   = 0.087;    // Set maximum normal deviation.// 0.9 < dot(surface_normal, point_normal);
    Efficient_ransac::Parameters ransac_parameters;
    set_parameters(ransac_parameters, probability, min_points, epsilon, cluster_epsilon, normal_thresh);
    Point_set point_set;
    planar_ransac(ascii_in, ransac_parameters, point_set);
    save_point_set(point_set, ascii_out);

    return 0;
}
