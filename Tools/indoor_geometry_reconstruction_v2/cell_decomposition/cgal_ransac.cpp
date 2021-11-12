
#include <fstream>
#include <iostream>
#include <CGAL/Timer.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Point_set_3/IO.h>


#include "LaserPoints.h"


// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef Kernel::Point_3 Point;

/// use this function for planar segmentation if you have laserpoints as input
int efficient_RANSAC_with_point_access(LaserPoints laserpoints, string outdir,
                                       Efficient_ransac::Parameters ransac_parameters,
                                       int nb_neighbors, LaserPoints &lp_seg_out, bool estimate_normals) {
    cout << "Efficient RANSAC" << endl;

    // Points with normals.
    Pwn_vector points;
//    // Load point set from a file.
//    ifstream stream(filename);
//    if (!stream || !CGAL::read_xyz_points(stream, back_inserter(points), CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()))) {
//      cerr << "Error: cannot read the file!" << endl;
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
    boost::tie (label, success) = point_set.add_property_map<FT> ("label", 1000000);
    assert (success);
    point_set.reserve (laserpoints.size()); // For memory optimization
    Point_set::iterator pset_it = point_set.begin();
    cout << "create point_set and points with normal from laserpoints:" << endl;
    for( auto &p : laserpoints){

        Kernel::Point_3 point3(p.X(), p.Y(), p.Z());
        Kernel::Vector_3 normal3;
        Point_with_normal pwn;
        pwn = make_pair(point3, normal3);
        points.push_back(pwn);

        /// create a point_set with all the laserpoints properties
        //if(pset_it!= point_set.end()){
            point_set.insert(point3);
            point_set.normal(*pset_it)=normal3;
            color[*pset_it] = {{(unsigned char) p.Red(), (unsigned char) p.Green(), (unsigned char) p.Blue() }};
            intensity[*pset_it] = p.Intensity();
            label[*pset_it] = p.Label();
            //cout << label[*pset_it] << endl;
            pset_it++;
        //}
    }

    //print_point_set (point_set);
    string outascii_filename = outdir + "/input.txt";
    save_point_set(point_set, outascii_filename);
    //CGAL::draw(point_set);


    cout << points.size() << " points" << endl;

    if(estimate_normals)
    {
        CGAL::pca_estimate_normals<Concurrency_tag>
                (points, nb_neighbors,
                 CGAL::parameters::point_map(Point_map()).
                         normal_map(Normal_map()));

        // Orients normals.
        // Note: mst_orient_normals() requires a range of points
        // as well as property maps to access each point's position and normal.
        //list<PointVectorPair>::iterator unoriented_points_begin =
        vector<Point_with_normal>::iterator unoriented_points_begin =
                CGAL::mst_orient_normals(points, nb_neighbors,
                                         CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
                                                 normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
        // Optional: delete points with an unoriented normal
        // if you plan to call a reconstruction algorithm that expects oriented normals.
        points.erase(unoriented_points_begin, points.end());
        cout << points.size() << " points after erasing unoriented points!!!" << endl;

        // Saves point set with normals and properties.
        //char* output_filename = (char*) "/00_wNormals.xyz";
        /// NOTE: here I changed points to point_set to export
        string outnormals_filename = outdir + "/00_wNormals.xyz";
        ofstream out(outnormals_filename);
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
  cout << "preprocessing time: " << time.time() << "s" << endl;
  // Perform detection several times and choose result with the highest coverage.
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  FT best_coverage = 0;
  for (size_t i = 0; i < 3; ++i) {
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
    cout << "shape detection time: " << time.time() << "s" << endl;
    cout << ransac.shapes().end() - ransac.shapes().begin()
    << " primitives, " << coverage << " coverage" << endl;
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
  while (it != shapes.end()) {
    boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
    /// Use Shape_base::info() to print the parameters of the detected shape.
    cout << "shape info: " << (*it)->info() << endl;
    // Sums distances of points to the detected shapes.
    FT sum_distances = 0;
    /// Iterate through point indices assigned to each detected shape.
    vector<size_t>::const_iterator
    index_it = (*it)->indices_of_assigned_points().begin();
    while (index_it != (*it)->indices_of_assigned_points().end()) {
        cout << "point index: "<< *index_it << endl;
      /// Retrieve point.
      const Point_with_normal& p = *(points.begin() + (*index_it));
      /// construct a laserpoint and set a segmentnumber to it and add it
      /// OR retireve the orginal laser point (which has other attributes like color and label)
      /// to laserpoints class in mapping library
      LaserPoint point(p.first.x(), p.first.y(), p.first.z());
      //point.X() = p.first.x();
      //point.Y() = p.first.y();
      //point.Z() = p.first.z();
      point.SetNormal(Vector3D(p.second.x(), p.second.y(), p.second.z()));
      point.SetAttribute(SegmentNumberTag, shape_counter);

//TODO: there is an error here in indexing (also becasue of erased points).
      // we try to add labels and color to the laserpoints from CGAL::point_set
      pwn_it = points.begin() + (*index_it);
      if(pwn_it != points.end()){
          int pindex = distance( points.begin(), pwn_it ); // error is here
          cout << pindex << endl;
          int plabel = label[pindex];
          point.Label(plabel);
          lp_seg_out.push_back(point);
          //point.Print();
      }


      /// Adds Euclidean distance between point and shape.
      sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
      /// Proceed with the next point.
      index_it++;
    }
    // Compute and print the average distance.
    FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
    cout << " average distance: " << average_distance << endl;
    // Proceed with the next detected shape.
    it++;
    shape_counter++;
  }
    string output_seg_laser = outdir + "/lp_segmented.laser";
    lp_seg_out.Write(output_seg_laser.c_str(), false);

  return EXIT_SUCCESS;

}
