#ifndef PLANAR_SEGMENTATION_H
#define PLANAR_SEGMENTATION_H

#include <iostream>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

#include <LaserPoints.h>


// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits
        <Kernel, Pwn_vector, Point_map, Normal_map>             Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT                                           FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Point_set_3<Point> Point_set;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;


//class planar_segmentation
//{
//public:
//    planar_segmentation();
//};

// Set parameters for shape detection.
int set_parameters (Efficient_ransac::Parameters &parameters,
                    double probability=0.05, int min_points=200, double epsilon=0.002,
                    double cluster_epsilon=0.05, double normal_thresh=0.90);

int basic_efficient_ransac (const char *filename, std::string outdir,
                            Efficient_ransac::Parameters parameters,
                            int nb_neighbors = 20, bool estimate_normals=true);

int efficient_RANSAC_with_point_access(const char *filename, std::string outdir,
                                       Efficient_ransac::Parameters ransac_parameters,
                                       int nb_neighbors, LaserPoints &lp_seg_out, bool estimate_normals);

/// use this function for planar segmentation if you have laserpoints as input
int efficient_RANSAC_with_point_access(LaserPoints laserpoints, std::string outdir,
                                       Efficient_ransac::Parameters ransac_parameters,
                                       int nb_neighbors, LaserPoints &lp_seg_out, bool estimate_normals);


int pca_normal_estimation(const char* fname, std::list<PointVectorPair> &points, double r);

int store_shapes(std::string outdir, Efficient_ransac ransac, Pwn_vector points);

#endif // PLANAR_SEGMENTATION_H
