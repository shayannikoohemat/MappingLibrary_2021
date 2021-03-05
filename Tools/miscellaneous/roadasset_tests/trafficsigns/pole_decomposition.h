//
// Created by shayan on 10/27/20.
//

#ifndef TRAFFICSIGNS_POLE_DECOMPOSITION_H
#define TRAFFICSIGNS_POLE_DECOMPOSITION_H

#endif //TRAFFICSIGNS_POLE_DECOMPOSITION_H

#include <LaserPoints.h>

///
/// \param lpoints
/// \param root
/// \param slice_height  smaller slice height takes more processing time
/// \param width_diff_tolerance
/// \param tag
/// \param RANSAC_linefitting
/// \param max_dist_toLine
/// \param verbose
void pole_decomposition_slicing(LaserPoints &lpoints, char *root, double slice_height=0.20,
        double width_diff_tolerance=0.05, LaserPointTag tag=SegmentNumberTag,
        bool RANSAC_linefitting=true, double max_dist_toLine=0.15, bool verbose=false);

void Renumber_objpoints(ObjectPoints &objpoints_in, LineTopologies &lines_topo_out,
                        ObjectPoints &objpoints_out, int seg_num, bool close_polygon=false);

// Store points and topology in one file
// this functions creads list of lines or polygons in linestopo_in, finds the corresponding points in objpoints_in
/// then gets the last number in objpoints_out and renumber vertices and lines and adds them to the output
/* This function needs to be tested it is not finalized */
void Renumber_objpointsANDlinestopo(ObjectPoints &objpoints_in, LineTopologies &linestopo_in,
                                    ObjectPoints &objpoints_out, LineTopologies &lines_topo_out,
                                    int seg_num, bool close_polygon);

void Visualize2DRectangles(vector< LaserPoints> &laserpoints_vec, ObjectPoints &rect_corners,
                           LineTopologies &rect_edges, LaserPointTag tag);


LaserPoints compare_slices (vector <LaserPoints> slices_vec, double tolerance, vector<LaserPoints> &accepted_slices);

Line3D RANSAC3DLine_customized(LaserPoints lp, double max_dist_to_line, int min_num_hits,
                               int max_num_tries, int &num_hits,
                               double max_dist_between_points,
                               LineSegments3D &segments);

int pole_decomposition_fittingLineRANSAC(LaserPoints &predicted_poles, char *root, double max_dist_toLine, bool verbose) ;