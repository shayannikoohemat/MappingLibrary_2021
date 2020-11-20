//
// Created by NikoohematS on 11-10-2017.
//

#ifndef VISUALIZATION_TOOLS_VISUALIZATION_TOOLS_H
#define VISUALIZATION_TOOLS_VISUALIZATION_TOOLS_H

#endif //VISUALIZATION_TOOLS_VISUALIZATION_TOOLS_H

#include <LaserPoints.h>
#include "Plane.h"

void VisualizePlanes(LaserPoints segmented_laserpoints, int minsizesegment, double max_edge_dist,
                    ObjectPoints &corners, LineTopologies &polygons, bool verbose);

void VisualizePlanes(LaserPoints segmented_laserpoints, int minsizesegment,
                     ObjectPoints &corners, LineTopologies &polygons, char* root, bool verbose);

void Visualize2DRectangles(vector< LaserPoints> &laserpoints_vec,ObjectPoints &rect_corners,
                           LineTopologies &rect_edges, LaserPointTag tag);

/// if min_z, max_z and height leave as dfault then the min and max would be extracted from the
/// segment's bounds, otherwise, height should be defined OR minz and maxz
void Minimum3DBox(ObjectPoints &corners, LineTopologies &polygon_lines, LaserPoints segmented_lp,
                  double min_z = 0, double max_z = 0, double height = 0);

void Min3DRectangle_to_3DBox (const ObjectPoints &rectangle_vertices, double offset_distance,
                              ObjectPoints &threeDbox_vertices, LineTopology &threeDbox_edges);

LaserPoints Project_points_to_Plane(LaserPoints lp, const Plane &plane);  /// it was const Plane &

void VisulizePlane3D (LaserPoints segment_lp, double max_dist, ObjectPoints &plane_corners,
                      LineTopology &plane_edges, bool verbose);

void EnclosingRectangle_with_Rotation (const LaserPoints &laserPoints, double max_dist,
                                       ObjectPoints &rectangle_corners, LineTopology &rectangle_edges,
                                       char*root, bool verbose=false);

void EnclosingRectangle_Rotation3D (const LaserPoints &laserPoints, double max_dist,
                                    ObjectPoints &rectangle_corners, LineTopology &rectangle_edges,
                                    char *root, bool verbose=false);

void EnclosingRectangle_Rotation3D (const LaserPoints &laserPoints, Plane plane, double max_dist,
                                    ObjectPoints &rectangle_corners, LineTopology &rectangle_edges,
                                    char *root, bool verbose=false);

void fit_3D_minimumRectangle (const LaserPoints &segmented_lpoints, double angle_threshold,
                              double max_edge_dist, int min_segment_size,
                              ObjectPoints &plane_corners, LineTopologies &plane_edges);

bool EnclosingRectangle3D (LaserPoints &laserpoints, double max_edge_dist, ObjectPoints &points,
                           LineTopology &polygon, char *root);

struct Line3DSegment {
    ObjectPoints begin_end_point;
    LineTopology line_top;
};

Line3DSegment Line3D_to_LineSegment (Line3D &line3d, Position3D &pos, double t);

/// this function always doesn't generate correct result
/// segmented_lp is preferred for better result
void EnclosingPolygon_test (LaserPoints lp, char *root);

void GenerateConvexPolygon(ObjectPoints &corners, LineTopologies &polygons,
                           LaserPoints segmented_lp, const LaserPointTag &tag);

void Min3DBox_OFF(ObjectPoints &corners, LineTopologies &polygon_lines, LaserPoints segmented_lp,
                  int min_segment_size, char *OFFfile, double min_z, double max_z, double height);

void Min3DRectangle_to_3DFaces (const ObjectPoints &rectangle_vertices, double offset_distance,
                                ObjectPoints &threeDbox_vertices, LineTopologies &threeDbox_faces);

void LineTopologies_to_OFF (const ObjectPoints &vertices, const LineTopologies &faces, char*root, bool verbose=false);

void LineTopologies_to_OFFBoxes (const ObjectPoints &vertices, const LineTopologies &faces, char *root, bool verbose=false);

void offset_polygon(const ObjectPoints &vertices, double offset_dist,
                    ObjectPoints &left_corners, ObjectPoints &right_corners,
                    LineTopology &left_polygon, LineTopology &right_polygon);

void offset_polygon(const ObjectPoints &vertices, const Plane &plane, double offset_dist,
                    ObjectPoints &left_corners, ObjectPoints &right_corners,
                    LineTopology &left_polygon, LineTopology &right_polygon);

Plane calculate_plane_between_two_parallel_planes (const Plane &plane1, const Plane &plane2);

Plane calculate_plane_between_two_parallel_segments (LaserPoints &segment1, LaserPoints &segment2, bool use_weight);

Plane offset_plane (const Plane &plane, double offset_distance);

/// if use_weight then the size of the segment (# points) is used as weight for calculating the normal
LaserPoints test_planes_calculation (LaserPoints &segment1, LaserPoints &segment2);

bool Extend_Segments_to_Intersection (LaserPoints &segment1, LaserPoints &segment2,
                                      const Plane &plane1, const Plane &plane2, double max_intersection_dist,
                                      LaserPoints &extended_segment1, LaserPoints &extended_segment2,
                                      bool &extension);
