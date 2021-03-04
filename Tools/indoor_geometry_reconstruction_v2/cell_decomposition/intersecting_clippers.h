#include <LaserPoints.h>
#include <iostream>
#include <map>


void intersect_planes(char *laserfile, char *root_dir, int min_segment_size);

void ExtractFaces_from_3DPolygonsIntersection(LaserPoints laserPoints, Planes planes,
                                              ObjectPoints polygons_vertices, //input polygons of segments
                                              LineTopologies polygons_edges, //input polygons of segments
                                              ObjectPoints &vertices,  //output faces
                                              LineTopologies &faces);


Planes bounding_cube(char *root_dir, LaserPoints &lp, ObjectPoints &lp_vertices,
        LineTopologies &lp_faces, std::map<int, Positions3D> &planes_pos_map) ;

/// intersecting a plane with the edges of a clipping rectangle
///  to obtain the line segment that cuts the rectangle
///
/// \param clipperRectangle_edges  clipping rectangle
/// \param clipperRectangle_vertices  clipping rectangle
/// \param plane  target plane which intersects the edges of the rectangle
/// \return  the line segment which is inside the plane of the clipping rectangle and is bounded by the edges
bool Intersect_Plane_3DRectnagle(LineTopology clipperRectangle_edges,
                                 ObjectPoints clipperRectangle_vertices,
                                 const Plane& plane, LineSegment3D &clipped_line);


/// intersecting a plane with 6 faces of a 3D box using Intersect_Plane_3DRectnagle() to obtain the polygon/face
/// intersection of a plane and a 3DBox can be a polygon: a triangle, rectangle, pentagon or a hexagon (6).
/// NOTE: faces of the 3Dbox DONOT need to be a closed polygon (0-1-2-3)
bool Intersect_Plane_3DBoxFaces(LineTopologies box3d_faces, const ObjectPoints &box3d_vertices, //input
                           const Plane &plane, int polygon_number, //input
                           LineTopologies &polygon_edges, ObjectPoints &polygon_vertices);

void Intersect_Planes_3DBoxFaces(LaserPoints segments,int min_seg_size,
                                 LineTopologies box3d_faces,const ObjectPoints box3d_vertices,
                                 LineTopologies &polygons_edges, ObjectPoints &polygons_vertices,
                                 bool verbose);


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


