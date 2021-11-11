#ifndef PLANAR_RANSAC_H
#define PLANAR_RANSAC_H

#endif // PLANAR_RANSAC_H

#include <iostream>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Property_map<FT> FT_map; //  FT is FieldNumberType
typedef CGAL::Shape_detection::Efficient_RANSAC_traits
        <Kernel, Point_set, Point_set::Point_map, Point_set::Vector_map> Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;


bool read_color_label_ascii(const std::string &ascii_filename, Point_set &point_set, int header_lines=1);

void save_point_set (Point_set& point_set, std::string out_ascii);

int planar_ransac(const std::string &ascii_filename, Efficient_ransac::Parameters ransac_parameters,
                  Point_set &point_set, int nb_neighbors=20);

int set_parameters (Efficient_ransac::Parameters &parameters,
                    double probability, int min_points, double epsilon,
                    double cluster_epsilon, double normal_thresh);

int calculate_normal_wrapper(std::string &ascii_in, std::string &ascii_out);
