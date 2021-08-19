#include "alpha_shape.h"
#include "LaserPoints.h"


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <fstream>
#include <iostream>
#include <list>
#include <cassert>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Alpha_shape_vertex_base_3<K>               Vb;
typedef CGAL::Alpha_shape_cell_base_3<K>                 Fb;
typedef CGAL::Triangulation_data_structure_3<Vb,Fb>      Tds;
typedef CGAL::Delaunay_triangulation_3<K,Tds,CGAL::Fast_location>  Delaunay;
typedef CGAL::Alpha_shape_3<Delaunay>                    Alpha_shape_3;
typedef K::Point_3                                       Point;
typedef Alpha_shape_3::Alpha_iterator                    Alpha_iterator;
typedef Alpha_shape_3::NT                                NT;
typedef CGAL::Point_set_3<Point>                         Point_set;



// note troubleshooting: if you have below error:
//terminate called after throwing an instance of 'CGAL::Assertion_exception'
//  what():  CGAL ERROR: assertion violation!
//Expr: ! CGAL_NTS is_zero(den)
//File: /home/shayan/cgal/Cartesian_kernel/include/CGAL/constructions/kernel_ftC3.h
//Line: 177
// comment this line:
//CGAL_kernel_assertion( ! CGAL_NTS is_zero(den) );
/// this code works for 3d shapes but the alpha shape is not optimal, for example for surfaces with holes.
/// I commented project-to-plane section because otherwise 3d alpha shape cant be created optimally.
/// for flat surfaces better to use 2D alpha shape and Point_2< Kernel > 	to_2d (const Point_3< Kernel > &p) const from
/// PLane_3<Kernel> class to project points to the plane.
/// useful links:
/// https://doc.cgal.org/latest/Alpha_shapes_2/index.html
/// https://doc.cgal.org/latest/Alpha_shapes_3/index.html
/// https://stackoverflow.com/questions/15905833/saving-cgal-alpha-shape-surface-mesh
/// https://stackoverflow.com/questions/24682250/creating-3d-alpha-shapes-in-cgal-and-the-visualization?noredirect=1&lq=1
/// https://stackoverflow.com/questions/59939534/how-to-get-the-remaining-triangles-after-2d-alpha-shape-using-cgal
int alphashape_w_fastlocation(LaserPoints lpoints, const char *offfile)
{
  // fit a plane to points and project points to the plane
//    Plane plane;
//    lpoints.SetAttribute(SegmentNumberTag, 1); // 1 random number, later replace it with segment number
//    plane = lpoints.FitPlane(lpoints[0].SegmentNumber());
//    plane.Number()= lpoints[0].SegmentNumber();
//    lpoints.FitPlane(0);
//    LaserPoints projected_points;
//    projected_points = Project_points_to_Plane(lpoints, plane);
//    cout << projected_points.size() << endl;
//    //projected_points.RemoveAlmostDoublePoints(false, 0.0001); // to avoid having degnerated faces, faces with identical vertices
//    //projected_points.RemoveDoublePoints(false);
//    //cout << "points after removing duplicates: "<< projected_points.size() << endl;
//    projected_points.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/floor_crop_projected.laser", false);


  // populate dt from laserpoints
  Delaunay dt;
  std::vector<Point> points_set;
  for (auto lpoint : lpoints){

      Point p(lpoint.X(), lpoint.Y(), lpoint.Z());
      dt.insert(p);
      //points_set.push_back(p);
  }

//  const char *filename
//  std::ifstream is(filename);
//  int n;
//  is >> n;
//  Point p;
//  std::cout << n << " points read" << std::endl;
//  for( ; n>0 ; n--) {
//    is >> p;
//    dt.insert(p);
//  }
  //std::cout << "Delaunay computed." << std::endl;
  // compute alpha shape
//  double radius = 10;
//  double alpha_value = radius * radius;

  Alpha_shape_3 alpha_sh(dt);
  //Alpha_shape_3 alpha_sh(points_set.begin(),points_set.end());//, alpha_value, Alpha_shape_3::REGULARIZED);
  std::cout << "Alpha shape computed in REGULARIZED mode by defaut."
            << std::endl;
  //alpha_sh.set_alpha(alpha_value);
   // find optimal alpha values
  Alpha_shape_3::NT alpha_solid = alpha_sh.find_alpha_solid();
  Alpha_iterator opt = alpha_sh.find_optimal_alpha(1);
  std::cout << "Smallest alpha value to get a solid through data points is "
            << alpha_solid << std::endl;
  std::cout << "Optimal alpha value to get one connected component is "
            <<  *opt    << std::endl;
  printf("alpha value opt: %.2f \n", *opt);
  alpha_sh.set_alpha(*opt);
  assert(alpha_sh.number_of_solid_components() == 1);


  // write alpha_shape for visualization
  //https://stackoverflow.com/questions/15905833/saving-cgal-alpha-shape-surface-mesh
  std::vector<Alpha_shape_3::Cell_handle> cells;
  std::vector<Alpha_shape_3::Facet> facets;
  std::vector<Alpha_shape_3::Edge> edges;
  // tetrahedron = cell
  alpha_sh.get_alpha_shape_cells(std::back_inserter(cells), Alpha_shape_3::INTERIOR);
  // triangles
  alpha_sh.get_alpha_shape_facets(std::back_inserter(facets), Alpha_shape_3::REGULAR);
  // edges
  alpha_sh.get_alpha_shape_edges(std::back_inserter(edges), Alpha_shape_3::REGULAR);

  std::cout << "The alpha-complex has : " << std::endl;
  std::cout << cells.size() << " cells as tetrahedrons" << std::endl;
  std::cout << facets.size() << " triangles" << std::endl;
  std::cout << edges.size() << " edges" << std::endl;

  std::stringstream pts;
  std::stringstream ind;
  std::size_t num_facets=facets.size();
  for (std::size_t i=0;i<num_facets;++i)
  {
    //To have a consistent orientation of the facet, always consider an exterior cell
    if ( alpha_sh.classify( facets[i].first )!=Alpha_shape_3::INTERIOR )
      facets[i]=alpha_sh.mirror_facet( facets[i] );
    CGAL_assertion(  alpha_sh.classify( facets[i].first )==Alpha_shape_3::INTERIOR  );

    int indices[3]={
      (facets[i].second+1)%4,
      (facets[i].second+2)%4,
      (facets[i].second+3)%4,
    };

    /// according to the encoding of vertex indices, this is needed to get
    /// a consistent orienation
    if ( facets[i].second%2==0 ) std::swap(indices[0], indices[1]);


    pts <<
    facets[i].first->vertex(indices[0])->point() << "\n" <<
    facets[i].first->vertex(indices[1])->point() << "\n" <<
    facets[i].first->vertex(indices[2])->point() << "\n";
    ind << "3 " << 3*i << " " << 3*i+1 << " " << 3*i+2 << "\n";
  }

  std::ofstream os;
  os.open (offfile);
  os << "OFF \n";
  os << 3*num_facets << " " << num_facets << " 0 \n";
  os << pts.str();
  os << ind.str();
  os.close();

  //std::cout << "OFF "<< 3*num_facets << " " << num_facets << " 0\n";
  //std::cout << pts.str();
  //std::cout << ind.str();

  return 0;
}
