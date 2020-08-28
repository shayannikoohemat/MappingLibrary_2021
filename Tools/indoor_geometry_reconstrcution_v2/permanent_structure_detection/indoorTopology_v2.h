//
// Created by root on 6/4/20.
//

#ifndef PERMANENT_STRUCTURE_DETECTION_INDOORTOPOLOGY_V2_H
#define PERMANENT_STRUCTURE_DETECTION_INDOORTOPOLOGY_V2_H

#endif //PERMANENT_STRUCTURE_DETECTION_INDOORTOPOLOGY_V2_H


void
indoorTopology(char *laserFile, char *root_dir, int minSegmentSize=1000,
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
               double Generate_WallPatches_angle_threshold=0.20,
               double Generate_WallPatches_dist_along_twosegments=0.40);


// set the specific parameters for each function in the code, currently they are hard-coded
void data_preparation (LaserPoints &laserPoints, LaserPoints &out_laserPoints,
                       bool do_surface_growing, bool do_segment_refinement,
                       bool down_sampling, bool ascii_to_laser, char* path_to_ascii);


void segmentation_surfaceGrowing(LaserPoints &laserpoints);

enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
    DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };
void SetColor(enum Color);