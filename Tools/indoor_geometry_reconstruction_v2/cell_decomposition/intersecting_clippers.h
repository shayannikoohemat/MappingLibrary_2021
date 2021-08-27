#include <LaserPoints.h>
#include <iostream>
#include <map>


void intersect_planes(char *laserfile, char *root_dir, int min_segment_size);

void ExtractFaces_from_3DPolygonsIntersection(LaserPoints laserPoints, Planes planes,
                                              ObjectPoints polygons_vertices, //input polygons of segments
                                              LineTopologies polygons_edges, //input polygons of segments
                                              ObjectPoints &vertices,  //output faces
                                              LineTopologies &faces);

void bounding_cube(char *off_file, LaserPoints lp, double scalefactor,
                   ObjectPoints &lp_vertices, LineTopologies &lp_faces);

Planes bounding_cube(char *root_dir, LaserPoints &lp, ObjectPoints &lp_vertices,
        LineTopologies &lp_faces, double enlarge_size, std::map<int, Positions3D> &planes_pos_map) ;

/// TODO look at Use: polygon.IntersectPolygonByLine() for a better implementation
/// intersecting a plane with the edges of a clipping rectangle (or polygon)
///  to obtain the line segment that cuts the rectangle
/// \param clipperRectangle_edges  clipping rectangle
/// \param clipperRectangle_vertices  clipping rectangle
/// \param plane  target plane which intersects the edges of the rectangle
/// \return  the line segment which is inside the plane of the clipping rectangle and is bounded by the edges
bool Intersect_Plane_3DRectnagle(LineTopology clipperRectangle_edges,
                                 ObjectPoints clipperRectangle_vertices,
                                 const Plane& plane, LineSegment3D &clipped_line, bool verbose=false);


/// intersecting a plane with 6 faces of a 3D box using Intersect_Plane_3DRectnagle() to obtain the polygon/face
/// intersection of a plane and a 3DBox can be a polygon: a triangle, rectangle, pentagon or a hexagon (6).
/// NOTE1: we drop very close vertices of the polygon <0.01m
/// NOTE2: faces of the 3Dbox DONOT need to be a closed polygon (0-1-2-3)
bool Intersect_Plane_3DBoxFaces(LineTopologies box3d_faces, const ObjectPoints &box3d_vertices, //input
                           const Plane &plane, int polygon_number, //input
                           LineTopologies &polygon_edges, ObjectPoints &polygon_vertices, bool verbose=false);

/// 1. fit planes to each segment in segments
/// 2. use Intersect_Plane_3DBoxFaces() to intersect the plane to the 3DBox
/// 3. get the extended polygons of each segment as a result of intersection with the bbox3d
/// NOTE: later we use these polygons to intersect them to each other to obtain new polygons/faces
/// this is tested, it works. use it for intersecting planes with the bounding box of the data
void Intersect_Planes_3DBoxFaces(LaserPoints segments,int min_seg_size,
                                 LineTopologies box3d_faces,const ObjectPoints box3d_vertices,
                                 LineTopologies &polygons_edges, ObjectPoints &polygons_vertices,
                                 bool close,
                                 bool verbose);

bool SplitPolygon3DByLineSegment3D(ObjectPoints &polygon_points,
                                   LineTopology &polygon_edges,
                               const LineSegment3D &line_segment,
                               double snap_dist,
                               LineTopologies &new_polygons);

/// splitting several polygons with one segment/plane
void SplitPolygons3DByPlane3D(ObjectPoints &polygon_v, LineTopologies &polygon_e,
                                        LaserPoints segment,
                                        ObjectPoints &new_polygon_v, LineTopologies &new_poly_e);

/// splitting one polygon incrementally by several segments/planes
/// CELL DECOMPOSITION
void SplitPolygons3DByPlanes3D(ObjectPoints &polygons_v, LineTopologies &polygons_e, LaserPoints segments, int min_seg_size,
                                        ObjectPoints &new_polygons_v, LineTopologies &new_polygons_e,
                               bool verbose=false);


/// test intersct a plane and a 3DBox
void test_Intersect_Plane_3DBoxFaces(LaserPoints segment, LineTopologies box3d_faces,
                                      const ObjectPoints& box3d_vertices);

void test_Intersect_Plane_3DRectnagle( LaserPoints segment, LineTopologies bbox_faces,
                                       const ObjectPoints& bbox_vertices);


// for a given list of polygons match each polygon topology and corresponding vertices and return them as vectors
// in this way by looping in the vector of polygons we can get the corresponding vertices from the vector of vertices
void TopologyAndVertices (const LineTopologies &topologies,const ObjectPoints &vertices,
                            vector<LineTopology> &topology_vec, vector<ObjectPoints> &vertices_vec);

ObjectPoints GetCorresponding_vertices(const ObjectPoints &vertices, const LineTopology &topology);

// checks if a point is inside the bounds of a linesegment
bool Point_Inside_3DLineBounds(const Position3D &point, const LineSegment3D &lineseg3D, double margin);

LineTopology ReOrderTopology(LineTopologies edges);

void RenumberTopology_and_Vertices(LineTopology edges, ObjectPoints vertices,
                                   LineTopologies &new_edges, ObjectPoints &new_vertices);


