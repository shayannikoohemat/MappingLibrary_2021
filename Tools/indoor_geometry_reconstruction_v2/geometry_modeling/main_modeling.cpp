//
// Created by shayan on 10/1/20.
//

#include <iostream>
#include "InlineArguments.h"
#include <cstdlib>
#include <map>
#include <LaserPoints.h>
#include <ctime>
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"
#include "ModelingInteriorWalls.h"

using namespace std;

void PrintUsage()
{
    printf("This tool is meant to be used to instruct how the 3Dmodeling functions should be applied.\n");
    printf("by 3D modeling, we mean the process of creating a vector model from planes"
           " and surfaces which is topologically consistent.\n");
    printf("Don't use this tool if you are not familiar with the modeling steps. Instead use individual tools in the subfolders. \n");
    printf("Usage: modeling \n");
    printf("      -root_dir         [root_dir] <required output processing direcotry, ./process/>\n");
    printf("      -i                [input laser file]    <if MERGE_SURF true, required input laser file , /data/foo.laser>\n");
    printf("      -minsegsize       [minSegmentSize] <min # of points for a segment to be processed.> \n");
    printf("      -plDist           [plane distance] <if MERGE_SURF true, define planes dist threshold to be merged, default is 0.5m>\n");
    printf("      -plAngle          [plane angle] <if MERGE_SURF true, define planes angles threshold to be merged, default is 5 degrees>\n");
    printf("      -maxDist_segments [max dist segments] <if MERGE_SURF true, define max seg. dist threshold to be merged, default is 1.0m>\n");
    printf("      -plDist2          [plane second dist] <if MERGE_SURF true, define max planes dist threshold to be merged, default is 0.80m>\n");
    printf("      -calc_midplane    [calc. mid plane] <if MERGE_SURF and calc_midplane true, then the middle plane of two faces are calculated.>\n");
    printf("      -calc_wplane      [calc. weighted plane] <if MERGE_SURF and calc_wplane true, then the weighted plane basedon the size of segments is calculated.>\n");
    printf("      -force_vertical   [force verticality] <if MERGE_SURF and force_vertical true, then for slightly slanted segmentes a vertical plane is calcualted, so define -ver_th.>\n");
    printf("      -ver_th           [verticality threshold] <if MERGE_SURF and force_vertical true, if a wall is slanted by -ver_th degrees a vertical plane is fitted, default 30 degrees.>\n");
    printf("      -i_slabs          [slabs laser file] <if EXT_W_SL true, then define floor OR ceiling OR both as a laser file, ./slabs.laser> \n");
    printf("      -ext_lpoints      [extended_laserpoints ] <if -MODEL_VolWall true, then define extended_walls as a laser file, ./extended_laserpoints.laser> \n");
    printf("      -wall_fl_cl       [input wall_fl_cl file] <if WALLS_Modeling true, then define the input laserfile containing walls, floors and ceilings, ./wfc.laser> \n");
    printf("      -max_extDist      [max extension dist] <if WALLS_Modeling true, then this threshold is used to extend the segments.> \n");
    printf("      -lwth             [lower_wall_thickness] <if MODEL_VolWall is true, default 0 if residuals as wall thick is given in laserpoints, otherwise -lwth is used, default 0.20> \n");
    printf("      -uwth             [upper_wall_thickness] <if MODEL_VolWall is true, default 0 if residuals as wall thick is given in laserpoints, otherwise -uwth is used, default 0.40> \n");
    printf("      -corners          [rectangles_vertices] <if MODEL_VolWall is true,then define corners.objpts as rectangles vertices> \n");
    printf("      -edges            [rectangles_edges] <if MODEL_VolWall is true,then define edges.top as rectangles edges> \n");
    printf("      -vertices         [3D boxes_vertices] <if MODEL2OFF is true,then define vertices.objpts as 3Dboxes vertices> \n");
    printf("      -faces            [3Dboxes_faces] <if MODEL2OFF is true,then define faces.top as 3Dboxes faces> \n");
    printf("      -MERGE_SURF       [merge surfaces] <If true, the program merges surfaces> \n");
    printf("      -EXT_W_SL         [EXTEND_WALLS_SLABS]    <If true, the program extends walls to slabs if there is a gap between them.>\n");
    printf("      -WALLS_Modeling   [walls modeling] <If true, walls extension and creating min rectangles will be applied.> \n");
    printf("      -MODEL_VolWall    [Model Vol. Walls] <If true, 3Dboxes are calculated from min rectangles.> \n");
    printf("      -MODEL2OFF        [Model to OFF] <If true, the model is converted to OFF format.> \n");
}

int main(int argc, char *argv[]){

    InlineArguments *args = new InlineArguments(argc, argv);

    std::clock_t start;
    start = std::clock();

    char str_root[500];

    if (args->Contains("-help")) {
        PrintUsage();
        exit(0);
    }

    // Check on required input files
    if (args->Contains("-help") ||
        //!args->Contains("-i") ||
        !args->Contains("-root_dir")
        //!args->Contains("-i_slabs") ||
        //!args->Contains("-wall_fl_cl") ||
        //!args->Contains("-EXT_W_SL") ||
        //!args->Contains("-MERGE_SURF") ||
        //!args->Contains("-WALLS_Modeling") ||
        //!args->Contains("-MODEL2OFF")
        )
    {
        if (!args->Contains("-help")) printf("Error: missing programme option."
                                             " -root_dir, a laser input and one of the booleans: "
                                             "-EXT_W_SL, -MERGE_SURF, -WALLS_Modeling, -MODEL_VolWall, -MODEL2OFF"
                                             "  are required parameters.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }

    char* root = args->String("-root_dir");
    /******************************************************************************************************************/
    /// STEP1. (optional) Merge surfaces created with indoor_topology tool. Recommended: just apply it on walls
    /* If necessary MergeSurfaces() from ../utils to create more consistent walls and planes,
    * then use the planes for ModelingInteriorWalls() */
    /*    OUTPUTS
        merged_surfaces.laser
        merged_planes.plane
        projected_merged_segments.laser >> intermediate results
        merge_info.txt >> metadata
     */
    //Mergesurfaces (laserpoints, planes_dist, planes_angle, segments_dist, min_segSize, root, planes_dist_second,true, false,true, verticality_angle_threshold, false);
    if(args->Contains("-MERGE_SURF")){
        printf("\n");
        printf("*** Applying Extension MERGE_Surfaces method. Merges coplanar and adjacent surfaces ...\n");
        LaserPoints lp;
       // lp.Read("../../test_data/geometry_modeling/wall_fl_cl_labeled_segmented.laser");
       // char* root = (char*) "../../test_data/geometry_modeling/out/";
       lp.Read(args->String("-i"));
        Mergesurfaces( lp,
                      args->Double("-plDist", 0.5),
                      args->Double("-plAngle", 5),
                      args->Double("-maxDist_segments", 1.0), // threshold between two segments to be considered for merging, e.g., a door way normally is around 0.80m which separates two segments.
                      args->Integer("-minsegsize", 100),
                      args->String("-root_dir"),
                      args->Double("-plDist2", 0.80), // maximum plane's distance to be merged.
                      args->Contains("-calc_midplane"),  // true is recommended
                      args->Contains("-calc_wplane"),    // false is recommended
                      args->Contains("-force_vertical"), // true is recommended for a consistent model if walls are vertical
                      args->Double("-ver_th", 30.0), // degrees
                      args->Contains("-v")
        );
    }

    /******************************************************************************************************************/
    /// STEP2: (optional) WALL to Ceiling/FLoor Extension
    /* first we extend merged_walls to the floor and ceiling separately, without any extension distance, it does it based on
     * the closest objects.
     * This step is not necessary if all walls are connected to the ceiling and floors.
     */
     /*    OUTPUTS
        extendedCeil_laserpoints.laser if applied on floors DO NOT forget to rename it to floor.
        extendedWall_laserpoints.laser >> main result
        rectanglesWalls_corners.objpts >> intermediate result
        rectanglesWalls_edges.top >> >> intermediate result
     */

    if (args->Contains("-EXT_W_SL")){ // if EXTEND_WALLS_SLABS is true
        printf("\n");
        printf("*** Applying Extension Wall-Slabs method. Extension of walls to Fl or Cl ...\n");
        LaserPoints merged_walls; // generate by Mergesurfaces()
        LaserPoints slabs; // it should be ceiling OR Floor. but we added them already into one file named it slab.laser
        Planes wall_planes; // generated with Mergesurfaces() otherwise pass it empty.
        /// read the merged_walls.laser from root-dir which should be created in the previous step
        strcpy(str_root, root); /// making a string of the path
        merged_walls.Read(strcat(str_root, "/merged_segments.laser")); // merged_segments.laser is the default name DON't rename it
        /// read the merged_planes.planes from the root_dir which should be created in the previous step
        strcpy(str_root, root);
        wall_planes.Read(strcat(str_root, "/merged_planes.planes"));
        printf("\n");
        if(wall_planes.empty())
            printf("!!!merged_planes is empty, a LeastSquare fitting plane is applied.\n");

        /// read slabs.laser
        slabs.Read(args->String("-i_slabs")); // floor+ceilings.laser
        /// calling the function from ModelingInteriorWalls.cpp
        Extend_Walls_Ceiling (merged_walls,
                wall_planes,
                slabs,
                args->Integer("-minsegsize", 100),
                args->String("-root_dir"),
                args->Contains("-v")
                );
    }
    /******************************************************************************************************************/
    /// STEP3:  /*  Modeling walls  */ extension of undershoot segments and generating the minimum_rectangle
    /*    OUTPUTS
        extended_laserpoints.laser >> main result
        rectangle_corners.objpts >> main result
        rectangle_edges.top >> main result
        thickness of walls is stored in residuals attribute in extended_laserpoints.laser
     */
    // NOTE: planes can be empty, then the function calculate the planes per segment using FitPlane function
    if(args->Contains("-WALLS_Modeling")){
        printf("\n");
        printf("*** Applying WALLS_Modeling method. Extension of segments and creating rectangles ...\n");
        LaserPoints lp;
        Planes wall_planes;
        lp.Read(args->String("-wall_fl_cl")); // this is wall+fl+cl.laser file
        if(lp.empty()) printf("ERROR: input merged_segments or laserfile is empty! \n");
        strcpy(str_root, root);
        wall_planes.Read(strcat(str_root, "/merged_planes.planes")); // this is merged_planes from Mergesurfaces()
        printf("\n");
        if(wall_planes.empty())
            printf("!!!merged_planes is empty, a LeastSquare fitting plane is applied.\n");

        ModelingInteriorWalls (lp,
                   wall_planes,
                   args->Double("-max_extDist", 0.5), // should be less than a narrow corridor
                   args->Integer("-minsegsize", 100),
                   args->String("-root_dir"),
                   args->Contains("-v")
        );
    }
    /******************************************************************************************************************/
    ///  STEP4: Modeling Volumetric Walls
    /* modeling volumetric walls (generate boxes from rectangles/polygons)
    the input of this function is directly the output of MergSurfaces() OR from
    ModelingInteriorWalls() from previous step. */
    /// NOTE: If you want to have separate OFF files for fl, cl and walls at the end, after step3 separate walls,
    /// floors and ceilings (using the labels) and proceed with this step.
    if(args->Contains("-MODEL_VolWall")){
        printf("\n");
        printf("*** Applying (MODEL_VolWall) GenerateVolumetricWalls method. Generating 3D boxes from rectangles ...\n");
        LaserPoints extended_wallflcl;
        if(!args->Contains("-ext_lpoints")){
            strcpy(str_root, root);
            extended_wallflcl.Read(strcat(str_root, "/extended_laserpoints.laser"));
            if(extended_wallflcl.empty()) printf("ERROR: extended_laserpoints.laser is empty or path is wrong! \n");
        }else
            extended_wallflcl.Read(args->String("-ext_lpoints"));
        //extended_wallflcl.Read("../../test_data/geometry_modeling/out/extended_laserpoints.laser");
        std::map <int, double> offset_dist_map;
        /// input values should be the expected wall thickness. Inputs divided by 2 will be offset_distance
        /// this function uses the wall thickness stored in the residual attribute of the segments.
        /// if upper and lower values are set to 0 then the function expects the residual attribute of the segments as thickness.
        offset_dist_map = Generate_offset_dist_map (extended_wallflcl,
                            args->Double("-lwth", 0.0), // 0.20 lower wall thickness
                            args->Double("-uwth", 0.0)  // 0.40 upper wall thickness
        );

        /// Generating 3D boxes from min rectangles
        /// the input of this function is the output of ModelingInteriorWalls()
        ObjectPoints walls_rectangle_vertices;
        LineTopologies walls_edges;
        if(!args->Contains("-corners")){
            walls_rectangle_vertices.Read(strcat(strcpy(str_root, root), "/rectangle_corners.objpts"));
            if(walls_rectangle_vertices.empty()) printf("ERROR: rectangle_corners.objpts is empty! \n");
        } else
            walls_rectangle_vertices.Read(args->String("-corners"));

        if(!args->Contains("-edges")){
            walls_edges.Read(strcat(strcpy(str_root, root), "/rectangle_edges.top"));
            if(walls_edges.empty()) printf("ERROR: rectangle_edges.top is empty! \n");
        } else
            walls_edges.Read(args->String("-edges"));
        /// calling the main function
        GenerateVolumetricWalls (walls_rectangle_vertices,
                                 walls_edges,
                                 offset_dist_map,
                                 args->Double("-fod", 0.10),
                                 args->String("-root_dir"),
                                 args->Contains("-v"));
    }
    /******************************************************************************************************************/
    /// STEP5: convert the model to the OFF format
    /* The output is off_output.off. If walls, fl and cl are separated, run this function for each one separately.*/
    if(args->Contains("-MODEL2OFF")){
        printf("\n");
        printf("*** Applying (MODEL2OFF) LineTopologies_to_OFFBoxes method. Generating OFF format from 3Dboxes ...\n");
        ObjectPoints vertices;
        LineTopologies faces;
        // if user doesnt define theinput it reads it from root_dir
        if(!args->Contains("-vertices")){
            vertices.Read(strcat(strcpy(str_root, root), "/walls_vertices.objpts"));
            if(vertices.empty()) printf("ERROR: walls_vertices.objpts is empty or wrong path! \n");
        } else
            vertices.Read(args->String("-vertices"));

        if(!args->Contains("-faces")){
            faces.Read(strcat(strcpy(str_root, root), "/walls_faces.top"));
            if(faces.empty()) printf("ERROR: walls_faces.top is empty or wrong path! \n");
        } else
            faces.Read(args->String("-faces"));
        /// calling the main function
        LineTopologies_to_OFFBoxes (vertices,
                                    faces,
                                    args->String("-root_dir")
        );
    }
    /******************************************************************************************************************/
    /// OPTIONAL: generate paper-thin walls instead of volumetric
    /* To do so, use the min_rectangles from STEP3 and apply the function VisualizePlanes() from
     * "../visualization_tools/visualization_tools.h" */


    double duration;
    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    std::cout<< "Total processing time: " << duration << "s" << "\n";

    return 0;
}

