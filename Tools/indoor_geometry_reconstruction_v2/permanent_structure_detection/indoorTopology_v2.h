//
// Created by root on 6/4/20.
//

#ifndef PERMANENT_STRUCTURE_DETECTION_INDOORTOPOLOGY_V2_H
#define PERMANENT_STRUCTURE_DETECTION_INDOORTOPOLOGY_V2_H

#endif //PERMANENT_STRUCTURE_DETECTION_INDOORTOPOLOGY_V2_H

///
/// \param laserFile                                #input laser file. Already with planar segments.
/// \param root_dir                                 # main directory for outputs
/// \param minSegmentSize                           #min segment size to be excluded.The unit is number-of-points
/// \param verbose                                  # verbose true if you need to Debug something
/// \param flat_angle                               # angle threshold to separate almost-horizontal segments
/// \param vertical_angle                           # angle threshold to separate almost-vertical segments
/// \param max_intersection_dist                    # max dist between two segments to be considered as adjacent in the graph
/// \param min_intersection_linelength              # min length of intersected line of two adjacent segment
/// \param sort_segments_by_number                  # if true it sorts the segments by segment number in the info-files
/// \param user_defined_ceil_floor_height           # if true user should define the ceiling & floor height using the next two parameters
/// \param ceiling_z_userdefined                    # if previous FLAG is true, set the highest ceiling Z-value here
/// \param floor_z_userdefined                      # if previous FLAG is true, set the lowest floor Z-value here
/// \param ceiling_buffer                           # within this buffer horizontal segments are considered ceiling
/// \param floor_buffer                             # within this buffer horizontal segments are considered floor
/// \param intersection_percentage      #percentage of overlap to discard one of the horizontal segments below the other one
/// \param apply_intersection_result    #if true then the lower segments would be excluded from candidate ceilings.
/// \param wallwall_vertical_dist_condition #if true two vertical walls (one above the other) with different normals are separated
/// \param wallwall_vertical_dist                   # the threshold related to previous FLAG. The distance between to walls on top of each other.
/// \param crop_walls_abovefloor_and_belowceiling   # if true wall points above the floor and below the ceiling average are cropped.
/// \param ceiling_z_average                        # the threshold related to previous FLAG
/// \param floor_z_average                          # the threshold related to previous FLAG
/// \param generate_wallpatches                     # if true it needs three next parameters to generalize segments -> result: "buffer_segmentation.laser"
/// \param Generate_WallPatches_dist_threshold      # distance along the normal direction between two segments to be considered for generalization
/// \param Generate_WallPatches_angle_threshold     # angle difference between two segments to be considered for generalization
/// \param Generate_WallPatches_dist_along_twosegments # distance along two segments to be considered for generalization
void
indoorTopology(char *laserFile, char *root_dir, int minSegmentSize=500,
                bool verbose=false, double flat_angle=10.0,
               double vertical_angle=10.0, double max_intersection_dist=0.10,
               double min_intersection_linelength=0.10,
               bool sort_segments_by_number=false, bool user_defined_ceil_floor_height=false,
               double ceiling_z_userdefined=0.0,
               double floor_z_userdefined=0.0, double ceiling_buffer=1.0,
               double floor_buffer=0.5,
               double intersection_percentage=0.50, bool apply_intersection_result=false,
               bool wallwall_vertical_dist_condition=false,
               double wallwall_vertical_dist=.30, bool crop_walls_abovefloor_and_belowceiling=false,
               double ceiling_z_average=0.0, double floor_z_average=0.0,
               bool generate_wallpatches=false, double Generate_WallPatches_dist_threshold=0.70,
               double Generate_WallPatches_angle_threshold=20.0,
               double Generate_WallPatches_dist_along_twosegments=0.40);

enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
    DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };
void SetColor(enum Color);