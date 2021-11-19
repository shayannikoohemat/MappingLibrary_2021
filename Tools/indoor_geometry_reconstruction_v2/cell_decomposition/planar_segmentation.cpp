#include "planar_segmentation.h"

// cgal includes
#if defined (_MSC_VER) && !defined (_WIN64)
#pragma warning(disable:4244) // boost::number_distance::distance()
                              // converts 64 to 32 bits integers
#endif
#include <fstream>
#include <iostream>
#include <CGAL/Timer.h>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/draw_point_set_3.h>

// for normal estimation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <utility> // defines std::pair
#include <list>

// for mapping library
#include "LaserPoints.h"

// for reading asccii
#include <boost/algorithm/string.hpp>


// Type declarations.
//typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
//typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
//typedef std::vector<Point_with_normal>                       Pwn_vector;
//typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
//typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
//typedef CGAL::Shape_detection::Efficient_RANSAC_traits
//        <Kernel, Pwn_vector, Point_map, Normal_map>             Traits;
//typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>            Cgal_Plane;
typedef CGAL::Shape_detection::Cylinder<Traits>         Cylinder;

// for adding attribtues to the points
typedef std::array<unsigned char, 3> Color;
typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Property_map<Color> Color_map;
typedef Point_set::Property_map<FT> FT_map; //  FT is FieldNumberType

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

///The parameter cluster_epsilon defines the spacing between two cells of the regular grid,
///so that two points separated by a distance of at most 22–√ cluster_epsilon are considered adjacent.
///source: https://doc.cgal.org/latest/Shape_detection/index.html#Shape_detection_RANSACParameters

/// Set parameters for shape detection.
int set_parameters (Efficient_ransac::Parameters &parameters,
                    double probability, int min_points, double epsilon,
                    double cluster_epsilon, double normal_thresh){

    // Set probability to miss the largest primitive at each iteration.
    parameters.probability = probability; //0.05;

    // Detect shapes with at least 500 points.
    parameters.min_points = min_points; //200;
    // Set maximum Euclidean distance between a point and a shape.
    parameters.epsilon = epsilon; //0.002;

    // Set maximum Euclidean distance between points to be clustered.shape_number[&p-&points[0]]
    parameters.cluster_epsilon = cluster_epsilon; //0.10; // 0.5

    // Set maximum normal deviation.
    //%Default value is 0.9 (around 25 degrees).
      //It must belong to the interval [0, 1].
    // 0.9 < dot(surface_normal, point_normal);
    parameters.normal_threshold = normal_thresh; //0.9;

    return EXIT_SUCCESS;
}

/// use this function for planar segmentation
/// it reads space delimited files with number of points in the begining of the file
int efficient_RANSAC_with_point_access(const char *filename, std::string outdir,
                                       Efficient_ransac::Parameters ransac_parameters,
                                       int nb_neighbors, LaserPoints &lp_seg_out, bool estimate_normals) {
    std::cout << "Efficient RANSAC" << std::endl;

    // Points with normals.
    Pwn_vector points;
    // Load point set from a file.
    std::ifstream stream(filename);
    if (!stream || !CGAL::read_xyz_points(stream, std::back_inserter(points),
                                          CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()))) {
        std::cerr << "Error: cannot read the file!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << points.size() << " points" << std::endl;

    if(estimate_normals)
    {
        CGAL::pca_estimate_normals<Concurrency_tag>
                (points, nb_neighbors,
                 CGAL::parameters::point_map(Point_map()).
                         normal_map(Normal_map()));

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

        // Saves point set with normals.
        //char* output_filename = (char*) "/00_wNormals.xyz";
        std::string output_filename = outdir + "/00_wNormals.xyz";
        std::ofstream out(output_filename);
        out.precision(6);
        CGAL::write_xyz_points(
                out, points,
                CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
                        normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
    }


  // Instantiate shape detection engine.
  Efficient_ransac ransac;
  // Provide input data.
  ransac.set_input(points);
  // Register detection of planes.
  ransac.add_shape_factory<Cgal_Plane>();
  // Measure time before setting up the shape detection.
  CGAL::Timer time;
  time.start();
  // Build internal data structures.
  ransac.preprocess();
  // Measure time after preprocessing.
  time.stop();
  std::cout << "preprocessing time: " << time.time() << "s" << std::endl;
  // Perform detection several times and choose result with the highest coverage.
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  FT best_coverage = 0;
  for (std::size_t i = 0; i < 3; ++i) {
    // Reset timer.
    time.reset();
    time.start();
    // Detect shapes.
    ransac.detect(ransac_parameters);
    // Measure time after detection.
    time.stop();
    // Compute coverage, i.e. ratio of the points assigned to a shape.
    FT coverage =
    FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
    // Print number of assigned shapes and unassigned points.
    std::cout << "shape detection time: " << time.time() << "s" << std::endl;
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
  int shape_counter=0;
  while (it != shapes.end()) {
    boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
    /// Use Shape_base::info() to print the parameters of the detected shape.
    //std::cout << (*it)->info();
    // Sums distances of points to the detected shapes.
    FT sum_distances = 0;
    /// Iterate through point indices assigned to each detected shape.
    std::vector<std::size_t>::const_iterator
    index_it = (*it)->indices_of_assigned_points().begin();
    while (index_it != (*it)->indices_of_assigned_points().end()) {
      /// Retrieve point.
      const Point_with_normal& p = *(points.begin() + (*index_it));
      /// construct a laserpoint and set a segmentnumber to it and add it
      /// to laserpoints class in mapping library
      LaserPoint point(p.first.x(), p.first.y(), p.first.z());
      //point.X() = p.first.x();
      //point.Y() = p.first.y();
      //point.Z() = p.first.z();
      point.SetNormal(Vector3D(p.second.x(), p.second.y(), p.second.z()));
      point.SetAttribute(SegmentNumberTag, shape_counter);
      lp_seg_out.push_back(point);
      //point.Print();

      /// Adds Euclidean distance between point and shape.
      sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
      /// Proceed with the next point.
      index_it++;
    }
    // Compute and print the average distance.
    FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
    std::cout << " average distance: " << average_distance << std::endl;
    // Proceed with the next detected shape.
    it++;
    shape_counter++;
  }
    std::string output_seg_laser = outdir + "/lp_segmented.laser";
    lp_seg_out.Write(output_seg_laser.c_str(), false);

  return EXIT_SUCCESS;
}


/// use this function for planar segmentation if you have laserpoints as input
int efficient_RANSAC_with_point_access(LaserPoints laserpoints, std::string outdir,
                                       Efficient_ransac::Parameters ransac_parameters,
                                       int nb_neighbors, LaserPoints &lp_seg_out, bool estimate_normals) {
    std::cout << "Efficient RANSAC" << std::endl;

    // Points with normals.
    Pwn_vector points;
//    // Load point set from a file.
//    std::ifstream stream(filename);
//    if (!stream || !CGAL::read_xyz_points(stream, std::back_inserter(points), CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()))) {
//      std::cerr << "Error: cannot read the file!" << std::endl;
//      return EXIT_FAILURE;
//    }

    Point_set point_set;
    point_set.add_normal_map();
    Color black = {{0,0,0}};
    bool success = false;
    Color_map color;
    boost::tie (color, success) = point_set.add_property_map<Color> ("color", black);
    assert (success);
    /// if intensity
    FT_map intensity;
    boost::tie (intensity, success) = point_set.add_property_map<FT> ("intensity", 0.);
    assert (success);
    /// if label
    FT_map label;
    boost::tie (label, success) = point_set.add_property_map<FT> ("label");
    assert (success);
    point_set.reserve (laserpoints.size()); // For memory optimization
    Point_set::iterator pset_it = point_set.begin();
    std::cout << "create point_set and points with normal from laserpoints:" << std::endl;
    for( auto &p : laserpoints){

        Kernel::Point_3 point3(p.X(), p.Y(), p.Z());
        Kernel::Vector_3 normal3;
        Point_with_normal pwn;
        pwn = std::make_pair(point3, normal3);
        points.push_back(pwn);

        /// create a point_set with all the laserpoints properties
        //if(pset_it!= point_set.end()){
            point_set.insert(point3);
            point_set.normal(*pset_it)=normal3;
            color[*pset_it] = {{(unsigned char) p.Red(), (unsigned char) p.Green(), (unsigned char) p.Blue() }};
            intensity[*pset_it] = p.Intensity();
            label[*pset_it] = p.Label();
            //std::cout << label[*pset_it] << endl;
            pset_it++;
        //}
    }

    //print_point_set (point_set);
    std::string outascii_filename = outdir + "/input.txt";
    save_point_set(point_set, outascii_filename);
    //CGAL::draw(point_set);


    std::cout << points.size() << " points" << std::endl;

    if(estimate_normals)
    {
        CGAL::pca_estimate_normals<Concurrency_tag>
                (points, nb_neighbors,
                 CGAL::parameters::point_map(Point_map()).
                         normal_map(Normal_map()));

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
        //points.erase(unoriented_points_begin, points.end());
        //cout << points.size() << " points after erasing unoriented points!!!" << endl;

        // Saves point set with normals and properties.
        //char* output_filename = (char*) "/00_wNormals.xyz";
        /// NOTE: here I changed points to point_set to export
        std::string outnormals_filename = outdir + "/00_wNormals.xyz";
        std::ofstream out(outnormals_filename);
        out.precision(6);
        CGAL::write_xyz_points(out, points,
                CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
                        normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
    }


  // Instantiate shape detection engine.
  Efficient_ransac ransac;
  // Provide input data.
  ransac.set_input(points);
  // Register detection of planes.
  ransac.add_shape_factory<Cgal_Plane>();
  // Measure time before setting up the shape detection.
  CGAL::Timer time;
  time.start();
  // Build internal data structures.
  ransac.preprocess();
  // Measure time after preprocessing.
  time.stop();
  std::cout << "preprocessing time: " << time.time() << "s" << std::endl;
  // Perform detection several times and choose result with the highest coverage.
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  FT best_coverage = 0;
  for (std::size_t i = 0; i < 3; ++i) {
    // Reset timer.
    time.reset();
    time.start();
    // Detect shapes.
    ransac.detect(ransac_parameters);
    // Measure time after detection.
    time.stop();
    // Compute coverage, i.e. ratio of the points assigned to a shape.
    FT coverage =
    FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
    // Print number of assigned shapes and unassigned points.
    std::cout << "shape detection time: " << time.time() << "s" << std::endl;
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
  int shape_counter=0;
  Pwn_vector::iterator pwn_it = points.begin();
  Point_set::const_iterator pointset_it = point_set.begin();
  while (it != shapes.end()) {
    boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
    /// Use Shape_base::info() to print the parameters of the detected shape.
    std::cout << "shape info: " << (*it)->info() << std::endl;
    // Sums distances of points to the detected shapes.
    FT sum_distances = 0;
    /// Iterate through point indices assigned to each detected shape.
    std::vector<std::size_t>::const_iterator
    index_it = (*it)->indices_of_assigned_points().begin();
    while (index_it != (*it)->indices_of_assigned_points().end()) {
        //std::cout << "point index: "<< *index_it << std::endl;
      /// Retrieve point.
      const Point_with_normal& p = *(points.begin() + (*index_it));
      /// construct a laserpoint and set a segmentnumber to it and add it
      /// OR retireve the orginal laser point (which has other attributes like color and label)
      /// to laserpoints class in mapping library
      LaserPoint point(p.first.x(), p.first.y(), p.first.z());
      point.SetNormal(Vector3D(p.second.x(), p.second.y(), p.second.z()));
      point.SetAttribute(SegmentNumberTag, shape_counter);
      lp_seg_out.push_back(point);

//TODO: there is an error here in indexing.
      // we try to add labels and color to the laserpoints from CGAL::point_set
      // a better solution is to use point_set with normals from the begininig (see planar_ransac.cpp)
//      pwn_it = points.begin() + (*index_it);
//      pointset_it = point_set.begin() + (*index_it);
//      cout << "point: " << point;
//      cout << "point set: " << point_set.point(*pointset_it) << endl;

//      if(pwn_it != points.end()){
//          int pindex = std::distance( points.begin(), pwn_it ); // error is here
//          std::cout << pindex << std::endl;
//          std::cout << *pointset_it << std::endl;
//          int plabel = label[*pointset_it];//label[pindex];
//          point.Label(plabel);
//          point.SetColour(color[*pointset_it][0], color[*pointset_it][1], color[*pointset_it][2]);
//          lp_seg_out.push_back(point);
//          //point.Print();
//      }


      /// Adds Euclidean distance between point and shape.
      sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
      /// Proceed with the next point.
      index_it++;
    }
    // Compute and print the average distance.
    FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
    std::cout << " average distance: " << average_distance << std::endl;
    // Proceed with the next detected shape.
    it++;
    shape_counter++;
  }
    std::string output_seg_laser = outdir + "/lp_segmented.laser";
    lp_seg_out.Write(output_seg_laser.c_str(), false);

  return EXIT_SUCCESS;

}


/// this has a bug in populating points into a point_set in store_shapes()
int basic_efficient_ransac (const char *filename, std::string outdir,
                            Efficient_ransac::Parameters ransac_parameters,
                            int nb_neighbors, bool estimate_normals) {
  std::cout << "Efficient RANSAC" << std::endl;

  // Points with normals.
  Pwn_vector points;
  // Load point set from a file.
  std::ifstream stream(filename);
  if (!stream ||
    !CGAL::read_xyz_points(
      stream,
      std::back_inserter(points),
      CGAL::parameters::point_map(Point_map()).
      normal_map(Normal_map()))) {
    std::cerr << "Error: cannot read the file!" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << points.size() << " points" << std::endl;

  if(estimate_normals)
  {
      CGAL::pca_estimate_normals<Concurrency_tag>
              (points, nb_neighbors,
               CGAL::parameters::point_map(Point_map()).
                       normal_map(Normal_map()));

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

      // Saves point set with normals.
      //char* output_filename = (char*) "/00_wNormals.xyz";
      std::string output_filename = outdir + "/00_wNormals.xyz";
      std::ofstream out(output_filename);
      out.precision(6);
      CGAL::write_xyz_points(
              out, points,
              CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
                      normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
  }
  // Instantiate shape detection engine.
  Efficient_ransac ransac;
  // Provide input data.
  ransac.set_input(points);
  // Register planar shapes via template method.
  ransac.add_shape_factory<Cgal_Plane>();

  // Detect shapes.
  ransac.detect(ransac_parameters);
  // Detect registered shapes with default parameters.
  //ransac.detect();
  // Print number of detected shapes.
  std::cout << ransac.shapes().end() - ransac.shapes().begin()
  << " shapes detected." << std::endl;

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

  // TODO fix ply storing bug
  std::string outfname = outdir + "/out_.ply";
  std::ofstream outPointset (outfname);
  outPointset.precision(6); // Use sufficient precision in ASCII
  CGAL::write_ply_point_set (outPointset, point_set); // same as `out << point_set` the ply is buggy don't know why
  //CGAL::write_xyz_point_set(outPointset, point_set); // doesnt save the segment number

  store_shapes(outdir, ransac, points);

  return EXIT_SUCCESS;
}

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;


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


/// this has a bug in populating points into a point_set
int store_shapes(std::string outdir, Efficient_ransac ransac, Pwn_vector points)
{
    if(ransac.shapes().empty()){
        std::cerr << "Error: ransac shape is empty! " << std::endl;
        return EXIT_FAILURE;
    }

    if(points.empty()){
        std::cerr << "Error: no point to store. Point set is empty! " << std::endl;
        return EXIT_FAILURE;
    }
    // provide data for labeling points to shape numbers
    Point_set point_set;
    point_set.add_normal_map(); // for adding normals
    FT_map shape_number;
    bool success = false;
    boost::tie (shape_number, success) = point_set.add_property_map<FT> ("shape_number", 0.);
    assert (success);
    point_set.reserve (points.size()); // For memory optimization
    /// populate the point_set with points and normals
    //int num=0;
    for (auto &p:points){
        if(point_set.has_normal_map()){
            point_set.insert(p.first, p.second);
        }
        //shape_number[&p-&points[0]] = num++; // test to add numbers, labels, intensity values, ...
    }

    /// Efficient_ransac::shapes() provides
    /// an iterator range to the detected shapes.
    Efficient_ransac::Shape_range shapes = ransac.shapes();
    Efficient_ransac::Shape_range::iterator shapeIt = shapes.begin();
    int shape_counter = 0;
    while (shapeIt != shapes.end()) {

        /// added by shayan
       // shape_counter = shapeIt - shapes.begin(); // get the number of the shape
        boost::shared_ptr<Efficient_ransac::Shape> shape = *shapeIt;
        // Use Shape_base::info() to print the parameters of the detected shape.
        std::cout << (*shapeIt)->info();
        // Sums distances of points to the detected shapes.
        FT sum_distances = 0;
        // Iterate through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator
        index_it = (*shapeIt)->indices_of_assigned_points().begin();
        while (index_it != (*shapeIt)->indices_of_assigned_points().end()) {

          // Retrieve point.
          const Point_with_normal& p = *(points.begin() + (*index_it));
          // add the point to the point_set
          //if(point_set.has_normal_map()){
          //    point_set.insert(p.first, p.second);
          //}
          //if(point_set.has_property_map(shape_number)){
          //    shape_number[&p-&points[0]] = shape_counter;
          //}
          // Adds Euclidean distance between point and shape.
          sum_distances += CGAL::sqrt((*shapeIt)->squared_distance(p.first));
          // Proceed with the next point.
          index_it++;
        }

        /// Compute and print the average distance.
        FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
        std::cout << ", average distance: " << std::setprecision (2) << average_distance << std::endl;

        // Proceed with the next detected shape.
        shapeIt++;
    }
    // TODO fix ply storing bug
    std::string outfname = outdir + "/out_.ply";
    std::ofstream outPointset (outfname);
    outPointset.precision(6); // Use sufficient precision in ASCII
    CGAL::write_ply_point_set (outPointset, point_set); // same as `out << point_set` the ply is buggy don't know why
   //CGAL::write_xyz_point_set(outPointset, point_set); // doesnt save the segment number

    return EXIT_SUCCESS;
}

void print_point_set (const Point_set& point_set)
{
  Color_map color;
  boost::tie (color, boost::tuples::ignore) = point_set.property_map<Color>("color");
  FT_map intensity;
  boost::tie (intensity, boost::tuples::ignore) =  point_set.property_map<FT>("intensity");
  /// if label
  FT_map label;
  boost::tie (label, boost::tuples::ignore) = point_set.property_map<FT> ("label");

  std::cerr << "Content of point set:" << std::endl;
  for (Point_set::const_iterator it = point_set.begin(); it != point_set.end(); ++ it)
  {
    std::cerr << "* Point " << point_set.point(*it) // or point_set[it]
              << " with color [" << (int)(color[*it][0])
              << " " << (int)(color[*it][1])
              << " " << (int)(color[*it][2])
              << "] and intensity " << intensity[*it]
              << " Label: " << label[*it]
              << std::endl;
  }
}

/// save pointset as ascii with xyz, color, intensity and label
void save_point_set (const Point_set& point_set, std::string out_ascii)
{
  Color_map color;
  boost::tie (color, boost::tuples::ignore) = point_set.property_map<Color>("color");
  FT_map intensity;
  boost::tie (intensity, boost::tuples::ignore) =  point_set.property_map<FT>("intensity");
  /// if label
  FT_map label;
  boost::tie (label, boost::tuples::ignore) = point_set.property_map<FT> ("label");

  /// open the file to write
  std::ofstream outfile;
  outfile.open(out_ascii, std::ios_base::app); // append mode
  outfile << "x y z r g b intensity label" << std::endl;
  for (Point_set::const_iterator it = point_set.begin(); it != point_set.end(); ++ it)
  {

    outfile   << point_set.point(*it) // or point_set[it]
              << " " << (int)(color[*it][0])
              << " " << (int)(color[*it][1])
              << " " << (int)(color[*it][2])
              << " " << intensity[*it]
              << " " << label[*it]
              << std::endl;

  }
  std::cout << point_set.size() << " points written to: " << out_ascii << std::endl;
}

/// read ascii conversion to cgal Point_set
bool read_color_label_ascii(const string &ascii_filename, Point_set &point_set, int header_lines)
{
    /// prepare the point_set
    point_set.add_normal_map();
    Color black = {{0,0,0}};
    bool success = false;
    Color_map color;
    boost::tie (color, success) = point_set.add_property_map<Color> ("color", black);
    assert (success);
    /// if intensity
    FT_map intensity;
    boost::tie (intensity, success) = point_set.add_property_map<FT> ("intensity", 0.);
    assert (success);
    /// if label
    FT_map label;
    boost::tie (label, success) = point_set.add_property_map<FT> ("label");
    assert (success);
    //point_set.reserve (laserpoints.size()); // For memory optimization

    /// read the ascii file
    ifstream fs;
    fs.open (ascii_filename.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        printf("Could not open file '%s'! Error : %s\n", ascii_filename.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }

    string line;
    vector<string> st;

    /// skip headers
    /// header_lines=11 // for PCD headers
    for (int i=0; i<header_lines; i++){
        getline (fs, line);
    }

    Point_set::iterator pset_it = point_set.begin();
    while (!fs.eof ()) {
        getline(fs, line);
        // Ignore empty lines
        if (line == "")
            continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r , "), boost::token_compress_on);

        if(st.size() < 3)
            continue;

        /// construct a cgal point3 object
        Kernel::Point_3 point3( float (atof (st[0].c_str ())),
                                float (atof (st[1].c_str ())),
                                float (atof (st[2].c_str ())));
        point_set.insert(point3);
        /// NOTE: there is no check if there is color or not
        int r, g, b;
        r = atoi (st[3].c_str ());
        g = atoi (st[4].c_str ());
        b = atoi (st[5].c_str ());
        color[*pset_it] = {{(unsigned char) r, (unsigned char) g, (unsigned char) b }};
        /// reserve for normals (for later)
        Kernel::Vector_3 normal3;
        point_set.normal(*pset_it)=normal3;
        /// if intensity uncomment this line and add the index of intensity column instead of ???
        //intensity[*pset_it] = atoi (st[???].c_str ());

        /// label
        label[*pset_it] = atoi (st[6].c_str ());

        pset_it++;
    }
    fs.close();


    return(true);
}


/// this has a bug probably in the 2nd while loop
//int store_shapes(std::string outdir, Efficient_ransac ransac, Pwn_vector points)
//{
//    if(ransac.shapes().empty()){
//        std::cerr << "Error: ransac shape is empty! " << std::endl;
//        return EXIT_FAILURE;
//    }

//    if(points.empty()){
//        std::cerr << "Error: no point to store. Point set is empty! " << std::endl;
//        return EXIT_FAILURE;
//    }
//    // provide data for labeling points to shape numbers
//    Point_set point_set;
//    point_set.add_normal_map(); // for adding normals
//    FT_map shape_number;
//    bool success = false;
//    boost::tie (shape_number, success) = point_set.add_property_map<FT> ("shape_number", 0.);
//    assert (success);
//    point_set.reserve (points.size()); // For memory optimization
//    /// populate the point_set with points and normals
//    //int num=0;
//    for (auto &p:points){
//        if(point_set.has_normal_map()){
//            point_set.insert(p.first, p.second);
//        }
//        //shape_number[&p-&points[0]] = 0;//num++; // test to add numbers, labels, intensity values, ...
//    }

//    // this is from here:
//    //https://cgal.geometryfactory.com/CGAL/doc/master/Polygonal_surface_reconstruction/Polygonal_surface_reconstruction_2polyfit_example_without_input_planes_8cpp-example.html
////    Efficient_ransac::Plane_range planes = ransac.planes();
////    std::size_t num_planes = planes.size();
////    std::cout << " Done. " << num_planes << " planes extracted. Time: " << t.time() << " sec." << std::endl;
////    // Stores the plane index of each point as the third element of the tuple.
////    Point_to_shape_index_map shape_index_map(points, planes);
////    for (std::size_t i = 0; i < points.size(); ++i) {
////      // Uses the get function from the property map that accesses the 3rd element of the tuple.
////      int plane_index = get(shape_index_map, i);
////      points[i].get<2>() = plane_index;
////    }

//    /// Efficient_ransac::shapes() provides
//    /// an iterator range to the detected shapes.
//    Efficient_ransac::Shape_range shapes = ransac.shapes();
//    Efficient_ransac::Shape_range::iterator shapeIt = shapes.begin();
//    int cnt=0;
//    while (shapeIt != shapes.end()) {

//        /// added by shayan
//        boost::shared_ptr<Efficient_ransac::Shape> shape = *shapeIt;
//        // Use Shape_base::info() to print the parameters of the detected shape.
//        std::cout << (*shapeIt)->info();
//        // Sums distances of points to the detected shapes.
//        FT sum_distances = 0;
//        // Iterate through point indices assigned to each detected shape.
//        std::vector<std::size_t>::const_iterator index_it = (*shapeIt)->indices_of_assigned_points().begin();
//        while (index_it != (*shapeIt)->indices_of_assigned_points().end()) {

//            // Retrieve point.
//            Pwn_vector::iterator pointsIt = points.begin() + (*index_it);
//            const Point_with_normal& p = *pointsIt;
//            //const Point_with_normal& p = *(points.begin() + (*shapeIndex_it));
//            int p_inx = pointsIt - points.begin();
//            // Adds Euclidean distance between point and shape.
//            sum_distances += CGAL::sqrt((*shapeIt)->squared_distance(p.first));
//            // populate the point_set with points and properties
//            //point_set.insert(p.first);   // add point
//            shape_number[p_inx] = cnt;          // add shape_number this is like segment number
//            // Proceed with the next point.
//            index_it++;
//        }
//        cnt++; // this should be outside shapeIndex loop then all the associated points to the shape get the same number

//        // Compute and print the average distance.

//        FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
//        std::cout << ", average distance: " << std::setprecision (2) << average_distance << std::endl;


//        /// Get specific parameters depending on the detected shape.
//        if (Cgal_Plane *plane = dynamic_cast<Cgal_Plane *>(shapeIt->get())) {

//            Kernel::Vector_3 normal = plane->plane_normal();
//            std::cout << "Plane with normal " << normal << std::endl;

//            // Plane shape can also be converted to the Kernel::Plane_3.
//            std::cout << "Kernel::Plane_3: " <<
//                      static_cast<Kernel::Plane_3>(*plane) << std::endl;
//        } /*else if (Cylinder *cyl = dynamic_cast<Cylinder *>(shapeIt->get())) {

//            Kernel::Line_3 axis = cyl->axis();
//            FT radius = cyl->radius();

//            std::cout << "Cylinder with axis "
//                      << axis << " and radius " << radius << std::endl;
//        }*/ else {

//            // Print the parameters of the detected shape.
//            // This function is available for any type of shape.
//            std::cout << (*shapeIt)->info() << std::endl;
//        }

//        // Proceed with the next detected shape.
//        shapeIt++;
//    }
//    // TODO fix ply storing bug
//    std::string outfname = outdir + "/out_.ply";
//    std::ofstream outPointset (outfname);
//    outPointset.precision(6); // Use sufficient precision in ASCII
//    CGAL::write_ply_point_set (outPointset, point_set); // same as `out << point_set` the ply is buggy don't know why
//   //CGAL::write_xyz_point_set(outPointset, point_set); // doesnt save the segment number

//    return EXIT_SUCCESS;
//}
