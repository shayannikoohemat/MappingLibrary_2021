//
// Created by root on 7/22/20.
//

#ifndef TRAFFICSIGNS_SHAPEDETECTION_CGAL_H
#define TRAFFICSIGNS_SHAPEDETECTION_CGAL_H

#endif //TRAFFICSIGNS_SHAPEDETECTION_CGAL_H

#include <iostream>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>


// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT                                           FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Point_set_3<Point> Point_set;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;



int shapeDetection_cgal(const char *fname, bool estimate_normals);

int pca_normal_estimation(const char* fname, std::list<PointVectorPair> &points, double search_radius=0);

int efficient_RANSAC_with_point_access(char* fname);

int populating_pointSet(char* fname);

void print_point_set (const Point_set& point_set);

void write_point_set(char* outfname, const Point_set& point_set);