//
// Created by NikoohematS on 20-11-2018.
//

#ifndef VISUALIZATION_TOOLS_MODELINGINTERIORWALLS_H
#define VISUALIZATION_TOOLS_MODELINGINTERIORWALLS_H

#endif //VISUALIZATION_TOOLS_MODELINGINTERIORWALLS_H


#include "LaserPoints.h"
#include "LaserPoint.h"

///
//a function that reads the floor OR ceiling and extends the walls to the closest floor/ceiling.
// The function can be applied first for wall-floor and then the result shall be used
// for wall-ceiling to make sure none of the spaces are open.
/// This function use the ANN search to find the closest floor/ceiling object and do the extension

/// \param merged_walls_lp      #input walls or merged-walls.laser file
/// \param walls_planes         #calculated wall´s planes. If passed empty, then it will be calculated, ../walls.planes
/// \param ceiling_lp           #slab laser file, ceiling or floor
/// \param min_segment_size     # min # of points for a segment to be processed
/// \param root                 # output processing directory, ./process/
/// \param verbose              # verbose for DEBUG
void Extend_Walls_Ceiling (const LaserPoints & merged_walls_lp, const Planes &walls_planes, const LaserPoints &ceiling_lp,
                           int min_segment_size=100, char *root= nullptr, bool verbose=false);

/// This function extend the segments to their intersection anf creates new min-rectangles
/*
 *  Main goal of this function: extension of segments to each other
 * Modeling interior walls, floor and ceilings:
 * 1. input: all the walls, floors and ceilings which are segmented.
 *  Labels are optional and not necessary for this function(wall=4, floor=5 and ceiling=6).
 * 2. We assume the input walls are generalized. (using /utils/MergeSurfaces.cpp)
 * 3. This function first make sure all segments are intersecting in a specific threshold by extending the
 * segment to the intersection line
 * 4. A minimum_enclosing_rectangle_3D will be generated for all segments (in 3D e.g. slanted walls, ramps)
 * 5. The plane for each segment should be an input from the generalization step otherwise a FitPlane method will be used
 * in place.
 * The output would be written to the disk and later would be input to GenerateVolumetricWalls() to generate boxes of each object.
 * NOTE: planes can be empty, then the function calculate the planes per segment using FitPlane function
 * */
/// \param laserPoints            # input laser points (it can be just walls.laser or wall-floor-ceiling.laser)
/// \param planes                 # calculated wall´s planes. If passed empty, then it will be calculated, ../walls.planes
/// \param max_intersection_dist  # max dist for intersection. The intersection_dist is calculated from the intersection line to each segment
/// \param min_segment_size       # min # of points for a segment to be processed
/// \param root                   # output processing directory, ./process/
/// \param verbose                # verbose for DEBUG
void ModelingInteriorWalls (const LaserPoints &laserPoints, const Planes &planes, double max_intersection_dist,
                            int min_segment_size=100, char *root= nullptr, bool verbose=false);
///
/*
 * Generate Volumetric Walls, floor ceiling means generating boxes from minimum rectangles
 * Each minimum-reactangle represents a permanent structure.
 * NOTE: the input of this function can be the output of ModelingInteriorWalls()
 * 1. The minimum-rectangle is offset to both sides of its plane within a given offset distance
 * 2. The offset distance is a map of rectangle numbers (segment number) and the distance threshold, which for most
 * of the walls should be the same and for floor and ceiling is different.
 * 3. If the map is empty then fix_offset_dist will be used.
 * Also the map can be read from a file which is generated from segments. The output is written to the root
 * as new objectpoints and linetopologies. In a next step it should be converted to OFF format for 3D softwares.
 * */
/// \param wall_rectangles_vertices # vertices.objpts from ModelingInteriorWalls() function or other sources
/// \param wall_rectangle_edges     # edges.objpts from ModelingInteriorWalls() function or other sources
/// \param offset_distance_map      # a map of wall thickness (offset_distance) based on the wall-thickness stored in Residual attributes in each segment
/// \param fix_offset_dist          # If the map is empty then fix_offset_dist will be used. Or for walls without thickness info
/// \param root                     # output processing directory, ./process/
/// \param verbose                  # verbose for DEBUG
void GenerateVolumetricWalls (const ObjectPoints &wall_rectangles_vertices, LineTopologies &wall_rectangle_edges,
                              const std::map<int, double> &offset_distance_map,
                              double fix_offset_dist=0.20, // this multiple by two is the fixed wall thickness.
                              char *root= nullptr, bool verbose=false);
///
/* This function generates the input for GenerateVolumetricWalls function.
 * This function generates a map of wall thickness (offset_distance) based on the Residual attributes in each segment
 * Also two thresholds are suggested if the user wants to have lower and upper threshold for wall thickness
 * If the two thresholds are left as default=0.0 then the function uses the default residuals. */
/// \param mergedSegments
/// \param lower_wall_thickness
/// \param upper_wall_thickness
/// \return
std::map<int, double> Generate_offset_dist_map (const LaserPoints &mergedSegments, double lower_wall_thickness=0.0,
                                                double upper_wall_thickness=0.0);