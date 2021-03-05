//
// Created by root on 7/22/20.
//

#include "shapeDetection_cgal.h"
#include <fstream>
#include <iostream>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Timer.h>
#include <CGAL/number_utils.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>

// for normal estimation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <utility> // defines std::pair
#include <list>


// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT                                           FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits
        <Kernel, Pwn_vector, Point_map, Normal_map>             Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Cone<Traits>             Cone;
typedef CGAL::Shape_detection::Cylinder<Traits>         Cylinder;
typedef CGAL::Shape_detection::Plane<Traits>            Plane;
typedef CGAL::Shape_detection::Sphere<Traits>           Sphere;
typedef CGAL::Shape_detection::Torus<Traits>            Torus;

// for adding attribtues to the points
typedef std::array<unsigned char, 3> Color;
typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Property_map<Color> Color_map;
typedef Point_set::Property_map<FT> FT_map;

// for normal estimation
typedef CGAL::Sequential_tag Concurrency_tag;

int shapeDetection_cgal(const char *fname, bool estimate_normals) {

    CGAL::Timer time;
    time.start();

    std::string rootDir = "/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/Shape_detection/";
    // Points with normals.
    Pwn_vector points;
    // Load point set from a file.
    //fname = "/Shape_detection/cube.pwn";
    // estimate normals if there is no normal
    //pca_normal_estimation(fname);
    std::ifstream stream(fname);
    if (!stream ||
        !CGAL::read_xyz_points(
                stream,
                std::back_inserter(points),
                CGAL::parameters::point_map(Point_map()).
                        normal_map(Normal_map()))) {

        std::cerr << "Error: cannot read file " << fname << "!!!" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << points.size() << " points" << std::endl;

    int nb_neighbors = 18;
    if(estimate_normals)
    {
        CGAL::pca_estimate_normals<Concurrency_tag>
                (points, nb_neighbors,
                 CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
                         normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));

        // Orients normals.
        // Note: mst_orient_normals() requires a range of points
        // as well as property maps to access each point's position and normal.
        //std::list<PointVectorPair>::iterator unoriented_points_begin =
        std::vector<Point_with_normal>::iterator unoriented_points_begin =
                CGAL::mst_orient_normals(points, nb_neighbors,
                                         CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
                                                 normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
        // Optional: delete points with an unoriented normal
        // if you plan to call a reconstruction algorithm that expects oriented normals.
        points.erase(unoriented_points_begin, points.end());
    }

    // Saves point set with normals.
    char* output_filename =
            (char*) "/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/Shape_detection/00_wNormals.xyz";
    std::ofstream out(output_filename);
    out.precision(6);
    CGAL::write_xyz_points(
            out, points,
            CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
                    normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));


    // Instantiate shape detection engine.
    Efficient_ransac ransac;
    // Provide input data.
    ransac.set_input(points);

    // Register shapes for detection.
    ransac.add_shape_factory<Plane>();
    //ransac.add_shape_factory<Sphere>();
    ransac.add_shape_factory<Cylinder>();
    //ransac.add_shape_factory<Cone>();
    //ransac.add_shape_factory<Torus>();
    // Set parameters for shape detection.
    Efficient_ransac::Parameters parameters;
    // Set probability to miss the largest primitive at each iteration.
    parameters.probability = 0.05;

    // Detect shapes with at least 500 points.
    parameters.min_points = 200;
    // Set maximum Euclidean distance between a point and a shape.
    parameters.epsilon = 0.002;

    // Set maximum Euclidean distance between points to be clustered.
    parameters.cluster_epsilon = 0.10; // 0.5

    // Set maximum normal deviation.
    // 0.9 < dot(surface_normal, point_normal);
    parameters.normal_threshold = 0.9;

    // Detect shapes.
    ransac.detect(parameters);
    // Print number of detected shapes and unassigned points.
    std::cout << ransac.shapes().end() - ransac.shapes().begin()
              << " detected shapes, "
              << ransac.number_of_unassigned_points()
              << " unassigned points." << std::endl;

    // provide data for labeling points to shape numbers
    Point_set point_set;
    point_set.add_normal_map(); // for adding normals
    FT_map shape_number;
    bool success = false;
    boost::tie (shape_number, success) = point_set.add_property_map<FT> ("shape_number", 0.);
    assert (success);
    point_set.reserve (points.size()); // For memory optimization
    //int num=0;
    for (auto &p:points){
        if(point_set.has_normal_map()){
            point_set.insert(p.first, p.second);
        }
        //shape_number[&p-&points[0]] = num++; // test to add numbers, labels, intensity values, ...
    }

    // Efficient_ransac::shapes() provides
    // an iterator range to the detected shapes.
    Efficient_ransac::Shape_range shapes = ransac.shapes();
    Efficient_ransac::Shape_range::iterator shapeIt = shapes.begin();
    int cnt=0;
    while (shapeIt != shapes.end()) {

        /// added by shayan
        boost::shared_ptr<Efficient_ransac::Shape> shape = *shapeIt;
        // Use Shape_base::info() to print the parameters of the detected shape.
        std::cout << (*shapeIt)->info();
        // Sums distances of points to the detected shapes.
        FT sum_distances = 0;
        // Iterate through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator shapeIndex_it = (*shapeIt)->indices_of_assigned_points().begin();
        while (shapeIndex_it != (*shapeIt)->indices_of_assigned_points().end()) {

            // Retrieve point.
            Pwn_vector::iterator pointsIt = points.begin() + (*shapeIndex_it);
            const Point_with_normal& p = *pointsIt;
            //const Point_with_normal& p = *(points.begin() + (*shapeIndex_it));
            int p_inx = pointsIt - points.begin();
            // Adds Euclidean distance between point and shape.
            sum_distances += CGAL::sqrt((*shapeIt)->squared_distance(p.first));
            // populate the point_set with points and properties
            //point_set.insert(p.first);   // add point
            shape_number[p_inx] = cnt;          // add shape_number this is like segment number
            // Proceed with the next point.
            shapeIndex_it++;
        }
        cnt++; // this should be outside shapeIndex loop then all the associated points to the shape get the same number

        // Compute and print the average distance.
        FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
        std::cout << " average distance: " << average_distance << std::endl;
        /// END of added later

        // Get specific parameters depending on the detected shape.
        if (Plane *plane = dynamic_cast<Plane *>(shapeIt->get())) {

            Kernel::Vector_3 normal = plane->plane_normal();
            std::cout << "Plane with normal " << normal << std::endl;

            // Plane shape can also be converted to the Kernel::Plane_3.
            std::cout << "Kernel::Plane_3: " <<
                      static_cast<Kernel::Plane_3>(*plane) << std::endl;
        } else if (Cylinder *cyl = dynamic_cast<Cylinder *>(shapeIt->get())) {

            Kernel::Line_3 axis = cyl->axis();
            FT radius = cyl->radius();

            std::cout << "Cylinder with axis "
                      << axis << " and radius " << radius << std::endl;
        } else {

            // Print the parameters of the detected shape.
            // This function is available for any type of shape.
            std::cout << (*shapeIt)->info() << std::endl;
        }

        // Proceed with the next detected shape.
        shapeIt++;
    }
    // TODO fix ply storing bug
    std::string outfname = rootDir + "/out_.ply";
    std::ofstream outPointset (outfname);
    out.precision(6); // Use sufficient precision in ASCII
    CGAL::write_ply_point_set (outPointset, point_set); // same as `out << point_set` the ply is buggy don't know why
    //CGAL::write_xyz_point_set(outPointset, point_set); // doesnt save the segment number
    time.stop();
    std::cout << "total processing: " << time.time() << "s" << std::endl;

    return EXIT_SUCCESS;
}

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

int pca_normal_estimation(const char* fname, std::list<PointVectorPair> &points, double r)
{
    //const char* fname = "data/sphere_1k.xyz";
    // Reads a .xyz point set file in points[].
    //std::list<PointVectorPair> points;
    //std::vector<Point_with_normal> points;
    std::ifstream stream(fname);
    if (!stream ||
        !CGAL::read_xyz_points(stream,
                               std::back_inserter(points),
                               CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())))
    {
        std::cerr << "Error: cannot read file " << fname<< std::endl;
        return EXIT_FAILURE;
    }
    // Estimates normals direction.
    // Note: pca_estimate_normals() requires a range of points
    // as well as property maps to access each point's position and normal.
    const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
   // if (argc > 2 && std::strcmp(argv[2], "-r") == 0) // Use a fixed neighborhood radius
   if(r==0.1)
    {
        // First compute a spacing using the K parameter
        double spacing
                = CGAL::compute_average_spacing<Concurrency_tag>
                        (points, nb_neighbors,
                         CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()));
        // Then, estimate normals with a fixed radius
        CGAL::pca_estimate_normals<Concurrency_tag>
                (points,
                 0, // when using a neighborhood radius, K=0 means no limit on the number of neighbors returns
                 CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
                         normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()).
                         neighbor_radius(2. * spacing)); // use 2*spacing as neighborhood radius
    }
   else // Use a fixed number of neighbors
    {
        CGAL::pca_estimate_normals<Concurrency_tag>
                (points, nb_neighbors,
                 CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
                         normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
    }

    // Orients normals.
    // Note: mst_orient_normals() requires a range of points
    // as well as property maps to access each point's position and normal.
    //std::list<PointVectorPair>::iterator unoriented_points_begin =
    std::list<PointVectorPair>::iterator unoriented_points_begin =
            CGAL::mst_orient_normals(points, nb_neighbors,
                                     CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
                                             normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
    // Optional: delete points with an unoriented normal
    // if you plan to call a reconstruction algorithm that expects oriented normals.
    points.erase(unoriented_points_begin, points.end());


    return EXIT_SUCCESS;
}

/// Retrieving Points Assigned To Shapes
int efficient_RANSAC_with_point_access(char* fname) {

    // Points with normals.
    Pwn_vector points;
    // Load point set from a file.
    std::ifstream stream(fname);
    if (!stream ||
        !CGAL::read_xyz_points(
                stream,
                std::back_inserter(points),
                CGAL::parameters::point_map(Point_map()).
                        normal_map(Normal_map()))) {

        std::cerr << "Error: cannot read file " << fname << "!" << std::endl;
        return EXIT_FAILURE;
    }
    // Instantiate shape detection engine.
    Efficient_ransac ransac;
    // Provide input data.
    ransac.set_input(points);
    // Register detection of planes.
    ransac.add_shape_factory<Plane>();
    ransac.add_shape_factory<Cylinder>();
    // Measure time before setting up the shape detection.
    CGAL::Timer time;
    time.start();
    // Build internal data structures.
    ransac.preprocess();
    // Measure time after preprocessing.
    time.stop();
    std::cout << "preprocessing took: " << time.time() * 1000 << "ms" << std::endl;
    // Perform detection several times and choose result with the highest coverage.
    Efficient_ransac::Shape_range shapes = ransac.shapes();

    FT best_coverage = 0;
    for (std::size_t i = 0; i < 3; ++i) {

        // Reset timer.
        time.reset();
        time.start();
        // Detect shapes.
        ransac.detect();
        // Measure time after detection.
        time.stop();
        // Compute coverage, i.e. ratio of the points assigned to a shape.
        FT coverage =
                FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
        // Print number of assigned shapes and unassigned points.
        std::cout << "time: " << time.time() * 1000 << "ms" << std::endl;
        std::cout << ransac.shapes().end() - ransac.shapes().begin()
                  << " primitives, " << coverage << " coverage" << std::endl;

        // Choose result with the highest coverage.
        if (coverage > best_coverage) {

            best_coverage = coverage;

            // Efficient_ransac::shapes() provides
            // an iterator range to the detected shapes.
            shapes = ransac.shapes();
        }
    }
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    while (it != shapes.end()) {

        boost::shared_ptr<Efficient_ransac::Shape> shape = *it;

        // Use Shape_base::info() to print the parameters of the detected shape.
        std::cout << (*it)->info();
        // Sums distances of points to the detected shapes.
        FT sum_distances = 0;
        // Iterate through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator index_it = (*it)->indices_of_assigned_points().begin();
        while (index_it != (*it)->indices_of_assigned_points().end()) {

            // Retrieve point.
            const Point_with_normal& p = *(points.begin() + (*index_it));
            // Adds Euclidean distance between point and shape.
            sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
            // Proceed with the next point.
            index_it++;
        }
        // Compute and print the average distance.
        FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
        std::cout << " average distance: " << average_distance << std::endl;
        // Proceed with the next detected shape.
        it++;
    }
    return EXIT_SUCCESS;
}

// this is a different test to populate pointSet from Point_3 points,
// to populate points from a file use: https://doc.cgal.org/latest/Point_set_3/index.html
int populating_pointSet(char* fname){
    // Points with normals.
    Pwn_vector points;
    std::ifstream stream(fname);
    if (!stream ||
        !CGAL::read_xyz_points(
                stream,
                std::back_inserter(points),
                CGAL::parameters::point_map(Point_map()).
                        normal_map(Normal_map()))) {

        std::cerr << "Error: cannot read file " << fname << "!!!" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << points.size() << " points" << std::endl;

    // populating the Point_Set
    Point_set point_set;
    point_set.add_normal_map(); // for adding normals
    FT_map segment_number;
    bool success = false;
    boost::tie (segment_number, success) = point_set.add_property_map<FT> ("segment number", 0.);
    assert (success);
    point_set.reserve (points.size()); // For memory optimization
    int i=0;
    for (auto &p:points){
        if(point_set.has_normal_map()){
            point_set.insert(p.first, p.second);
            std::cout << p.second << std::endl;
        }
        else{
            std::cerr << "Points don't have normal, nothing is written to the output!!!";
            return EXIT_FAILURE;
        }
        segment_number[i] = i++;
    }

    // print point_set
    //print_point_set(point_set);

    // write point set
    //write_point_set("out.ply" , point_set);

    return EXIT_SUCCESS;
}

void print_point_set (const Point_set& point_set)
{
    std::cerr << "Content of point set:" << std::endl;
    for (Point_set::const_iterator it = point_set.begin();
         it != point_set.end(); ++ it)
        std::cerr << "* Point " << *it
                  << ": " << point_set.point(*it) // or point_set[it]
                  << " with normal " << point_set.normal(*it)
                  << std::endl;
}

void write_point_set(char* outfname, const Point_set& point_set){
    std::ofstream out (outfname);
    out.precision(6); // Use sufficient precision in ASCII
    CGAL::write_ply_point_set (out, point_set); // same as `out << point_set`
}