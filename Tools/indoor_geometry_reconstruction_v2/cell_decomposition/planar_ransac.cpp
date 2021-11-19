#include "planar_ransac.h"
#include <fstream>
#include <iostream>
#include <CGAL/Timer.h>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/jet_estimate_normals.h>
// for reading asccii
#include <boost/algorithm/string.hpp>
// for laserpoints writing
#include "LaserPoints.h"


typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef CGAL::Point_set_3<Point> Point_set;
typedef std::array<unsigned char, 3> Color;
typedef Point_set::Property_map<Color> Color_map;
typedef Point_set::Property_map<FT> FT_map; //  FT is FieldNumberType
typedef CGAL::Shape_detection::Efficient_RANSAC_traits
        <Kernel, Point_set, Point_set::Point_map, Point_set::Vector_map> Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>            CGAL_Plane;

// Point with normal vector stored in a std::pair.
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointVectorPair;


// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

using namespace std;

//bool read_color_label_ascii(const std::string &ascii_filename, Point_set &point_set, int header_lines=1);

/// Set parameters for shape detection.
int set_parameters (Efficient_ransac::Parameters &parameters,
                    double probability, int min_points, double epsilon,
                    double cluster_epsilon, double normal_thresh){

    parameters.probability = probability; //0.05;
    parameters.min_points = min_points; //200;
    parameters.epsilon = epsilon; //0.02;
    parameters.cluster_epsilon = cluster_epsilon; //0.10;
    parameters.normal_threshold = normal_thresh; //0.9;
    return EXIT_SUCCESS;
}

/// simple planar ransac
int planar_ransac(const string &ascii_filename, Efficient_ransac::Parameters ransac_parameters,
                  Point_set &point_set, LaserPoints &lp_segmented, int nb_neighbors){

    // read points with labels and color
    if(read_color_label_ascii(ascii_filename, point_set, 1)){
        std::cout << " num of read points:" << point_set.size() << std::endl;
    } else {
        std::cout << "error in reading the file!" << std::endl;
        return EXIT_FAILURE;
    }

    // access the current property maps and store them in a point_map
    Color_map color;
    boost::tie (color, boost::tuples::ignore) = point_set.property_map<Color>("color");
    //FT_map intensity;
    //boost::tie (intensity, boost::tuples::ignore) =  point_set.property_map<FT>("intensity");
    /// if label
    FT_map label;
    boost::tie (label, boost::tuples::ignore) = point_set.property_map<FT> ("label");

    // add segment_num as another property map (each segment num is a plane shape from ransac)
    bool success = false;
    FT_map segment;
    boost::tie (segment, success) = point_set.add_property_map<FT> ("segment", -1);
    assert (success);


    /// create a point_set and calculate normals with PCA
    point_set.add_normal_map();
    CGAL::pca_estimate_normals<Concurrency_tag>(point_set, nb_neighbors,
                               CGAL::parameters::point_map(point_set.point_map()).normal_map(point_set.normal_map()));

    // write points with normals for double check
    //string out_ascii_normal="/out/points_normal_label.txt";
    //save_point_set(point_set, out_ascii_normal);

    // Measure time before setting up the shape detection.
    CGAL::Timer time;
    time.start();

    // Efficient_ransac
    Efficient_ransac ransac;
    ransac.set_input(point_set,
                     point_set.point_map(), // Call built-in property map
                     point_set.normal_map()); // Call built-in property map
    ransac.add_shape_factory<CGAL_Plane>();
     // Detect shapes.
     ransac.detect(ransac_parameters);

     // access points and assign shape id to points
     Efficient_ransac::Shape_range shapes = ransac.shapes();
     Efficient_ransac::Shape_range::iterator it = shapes.begin();
     int shape_counter=0;
     Point_set::const_iterator pointset_it = point_set.begin();
     while (it != shapes.end()) {
       boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
       /// Iterate through point indices assigned to each detected shape.
       std::vector<std::size_t>::const_iterator
       index_it = (*it)->indices_of_assigned_points().begin();
       while (index_it != (*it)->indices_of_assigned_points().end()) {
         /// retrieve point_set iterator
         pointset_it = point_set.begin() + (*index_it);
         /// set the segment number to the pointset
         //cout << "shape cnt: " << it - shapes.begin() << endl;
         segment[*pointset_it] = it - shapes.begin();

         /// construct a laserpoint and set the segmentnumber to it and
         /// add it to the laserpoints class in the Mapping library
         LaserPoint laserpoint(  point_set.point(*pointset_it)[0],
                                 point_set.point(*pointset_it)[1],
                                 point_set.point(*pointset_it)[2]);
         /// set the calcualted Normal vector
         laserpoint.SetNormal(Vector3D(point_set.normal(*pointset_it)[0],
                                       point_set.normal(*pointset_it)[1],
                                       point_set.normal(*pointset_it)[2]));
         /// set the shape number to segment number
         laserpoint.SetAttribute(SegmentNumberTag, shape_counter);
         /// set the label from the original file
         laserpoint.Label(label[*pointset_it]);
         /// set the color from the original file
         laserpoint.SetColour((int)(color[*pointset_it][0]),
                             (int)(color[*pointset_it][1]),
                             (int)(color[*pointset_it][2]));
         /// add it to the output laserpoints
         lp_segmented.push_back(laserpoint);

         index_it++;
       } // end points iteration per shape
        it++;
        shape_counter++;
     }//end shape iteration

    cout << "shape count: " << it - shapes.begin() << endl;

    time.stop();
    std::cout << "ransac processing time: " << time.time() << "s" << std::endl;

    return EXIT_SUCCESS;
}

/// save pointset as ascii with xyz, color, intensity and label
void save_point_set (Point_set& point_set, string out_ascii)
{

  Color_map color;
  boost::tie (color, boost::tuples::ignore) = point_set.property_map<Color>("color");
  FT_map intensity;
  boost::tie (intensity, boost::tuples::ignore) =  point_set.property_map<FT>("intensity");
  /// if label
  FT_map label;
  boost::tie (label, boost::tuples::ignore) = point_set.property_map<FT> ("label");
  /// if segment number
  FT_map segment;
  boost::tie (segment, boost::tuples::ignore) = point_set.property_map<FT> ("segment");

//  std::vector<std::string> properties = point_set.properties();
//  std::cout << "Properties:" << std::endl;
//  for (std::size_t i = 0; i < properties.size(); ++ i)
//    std::cout << " " << properties[i];
//  std::cout << "\n";
  /// open the file to write
  std::ofstream outfile;
  outfile.precision(6);
  outfile.open(out_ascii, std::ios_base::out); // use `app` for append mode
  if (point_set.has_normal_map()){
      outfile << "x y z r g b nx ny nz intensity label segment" << std::endl;
  } else
      outfile << "x y z r g b intensity label segment" << std::endl;
  for (Point_set::const_iterator it = point_set.begin(); it != point_set.end(); ++ it)
  {
      if(point_set.has_normal_map()){
          outfile   << point_set.point(*it) // or point_set[it]
                    << " " << (int)(color[*it][0])
                    << " " << (int)(color[*it][1])
                    << " " << (int)(color[*it][2])
                    << " " << point_set.normal(*it)[0]
                    << " " << point_set.normal(*it)[1]
                    << " " << point_set.normal(*it)[2]
                    << " " << intensity[*it]
                    << " " << label[*it]
                    << " " << segment[*it]
                    << std::endl;
        } else {
          outfile   << point_set.point(*it) // or point_set[it]
                    << " " << (int)(color[*it][0])
                    << " " << (int)(color[*it][1])
                    << " " << (int)(color[*it][2])
                    << " " << intensity[*it]
                    << " " << label[*it]
                    << " " << segment[*it]
                    << std::endl;
      }
  }
  std::cout << point_set.size() << " points written to: " << out_ascii << std::endl;
}

/// read ascii conversion to cgal Point_set
/// point orders:
/// 0 1 2 3 4 5 6: x y z r g b label
bool read_color_label_ascii(const std::string &ascii_filename, Point_set &point_set, int header_lines)
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
    boost::tie (label, success) = point_set.add_property_map<FT> ("label", -1);
    assert (success);
//    /// if segment numbers (we add seg_num in the RANSAC process not here)
//    FT_map segment;
//    boost::tie (segment, success) = point_set.add_property_map<FT> ("segment", -1);
//    assert (success);
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

    /// read num of lines
    std::ifstream inFile(ascii_filename.c_str ());
    int num_lines = std::count(std::istreambuf_iterator<char>(inFile), std::istreambuf_iterator<char>(), '\n');

    string line;
    vector<string> st;
    /// skip headers
    /// header_lines=11 // for PCD headers
    for (int i=0; i<header_lines; i++){
        getline (fs, line);
    }
    point_set.reserve (num_lines+1); // For memory optimization
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
        if (st.size() > 6) // xyz rgb label
             label[*pset_it] = atoi (st[6].c_str ());

        /// segment
//        if (st.size() > 7) // xyz rgb label segment
//            segment[*pset_it] = atoi (st[7].c_str ());

        pset_it++;
    }
    fs.close();

    return(true);
}

int calculate_normal_wrapper(string &ascii_in, string &ascii_out){
    Point_set ps;
    // read points with labels and color
    if(read_color_label_ascii(ascii_in, ps, 1)){
        std::cout << " num of read points:" << ps.size() << std::endl;
    } else {
        std::cout << "error in reading the file!" << std::endl;
        return EXIT_FAILURE;
    }
    // add normal property and estimate normals
    std::list<PointVectorPair> points;
    ps.add_normal_map();
    Point_set::iterator ps_it = ps.begin();
    for (ps_it = ps.begin(); ps_it != ps.end(); ps_it++){
        Kernel::Point_3 point3(ps.point(*ps_it));
        Kernel::Vector_3 normal3;
        PointVectorPair pwn; // point with normal
        pwn = std::make_pair(point3, normal3);
        points.push_back(pwn);
    }
    CGAL::pca_estimate_normals<Concurrency_tag>
            (points, 5, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                                                        .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
    /// populate pointset with normals of PointVectorPair
    Point_set::iterator pset_it = ps.begin();
    for (auto &p : points){
        ps.normal(*pset_it) = Vector(p.second.x(), p.second.y(), p.second.z());
        pset_it++;
    }
    save_point_set(ps, ascii_out);

    return EXIT_SUCCESS;
}
