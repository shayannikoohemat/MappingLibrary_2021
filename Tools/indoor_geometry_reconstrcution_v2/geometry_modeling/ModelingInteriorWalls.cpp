//
// Created by NikoohematS on 12-10-2018.
//

#include <ctime>
#include <iostream>
#include <iterator> // std::advance
#include <vector>
#include <map>
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "ModelingInteriorWalls.h"
#include "../visualization_tools/visualization_tools.h"
#include "../utils/utils.h"

//a function that reads the floor/ceiling and extends the walls to the closest floor/ceiling.
// The function can be applied first for wall-floor and then the result shall be used
// for wall-ceiling to make sure none of the spaces are open.
/// This function use the ANN search to find the closest floor/ceiling object and do the extension
void Extend_Walls_Ceiling (const LaserPoints & merged_walls_lp, const Planes &walls_planes, const LaserPoints &ceiling_lp,
        int min_segment_size, char *root, bool verbose){

    char str_root[500];
    strcpy (str_root, root);

    if (!merged_walls_lp.HasAttribute (SegmentNumberTag)){
        printf ("Error: Laserpoints should have SegmentNumberTag! \n");
        EXIT_FAILURE;
    }

    /// make a list of  wall segments
    vector<LaserPoints> walls_segments_vec;
    walls_segments_vec = PartitionLpByTag(merged_walls_lp, SegmentNumberTag, root);

    /// make a map of wall input planes
    /// the assumption is that plane_number and segmentNumber are the same for each wall segment
    std::map<int, Plane> planes_temp_map;
    if(!walls_planes.empty ()){
        for(auto &pl : walls_planes) {
            planes_temp_map.insert (std::pair<int, Plane> (pl.Number (), pl));
        }
    }

    /// make a map of walls segment_numbers and segments,
    /// fit a plane to each segment (if there is no plane) using PlaneFitting function and least square
    std::map<int, LaserPoints> walls_segments_map;
    std::map<int, Plane> walls_planes_map;
    for(auto &s : walls_segments_vec){
        if(s.HasAttribute (SegmentNumberTag) && s.size () > min_segment_size){
            walls_segments_map.insert (std::pair<int, LaserPoints> (s[0].SegmentNumber (), s));
            /// we assume segmentNumber and planeNumber are the same for each segment
            auto pl_m_it = planes_temp_map.find(s[0].SegmentNumber ());
            if(pl_m_it != planes_temp_map.end ()){
                walls_planes_map.insert (std::pair<int, Plane> (s[0].SegmentNumber (), pl_m_it->second));
                //planes_vec.push_back (walls_planes_map.find(s[0].SegmentNumber ())->second);
            }else {
                /// else fit a plane to the segment points
                Plane plane;
                plane = s.FitPlane (s[0].SegmentNumber ());
                plane.Number () = s[0].SegmentNumber ();
                walls_planes_map.insert (std::pair <int, Plane> (s[0].SegmentNumber (), plane));
                //planes_vec.push_back (s.FitPlane (s[0].SegmentNumber ()));
            }
        }
    }

    /* NOTE: note that the segment number of ceiling and walls could be in some cases the same, so we keep them separate*/
    /* make a map for CEILING */
    /// make a list of  ceiling segments
    vector<LaserPoints> ceiling_segments_vec;
    ceiling_segments_vec = PartitionLpByTag(ceiling_lp, SegmentNumberTag, root);

    /// makes a map of ceiling planes and segments
    std::map<int, LaserPoints> ceiling_segments_map;
    std::map<int, Plane> ceiling_planes_map;
    for (auto &clSeg : ceiling_segments_vec){
        if (clSeg.HasAttribute (SegmentNumberTag) && clSeg.size () > min_segment_size){
            ceiling_segments_map.insert (std::pair<int, LaserPoints> (clSeg[0].SegmentNumber (), clSeg));
            /// fit a plane to the segment points
            Plane plane;
            plane = clSeg.FitPlane (clSeg[0].SegmentNumber ());
            plane.Number () = clSeg[0].SegmentNumber ();
            ceiling_planes_map.insert (std::pair <int, Plane> (clSeg[0].SegmentNumber (), plane));
        }
    }

    /// make a KNN from Ceiling_lp for the search
    KNNFinder <LaserPoint> finder(ceiling_lp);

    /// loop through walls and find the closest ceiling and extend them to the intersection line based on their planes
    for (auto &wallSeg_m : walls_segments_map) {
        int wallSeg_no = wallSeg_m.first;
        LaserPoints wall_segment = wallSeg_m.second;
        Plane wall_plane;
        auto plane1_it = walls_planes_map.find (wallSeg_no);
        if (plane1_it != walls_planes_map.end ()) {
            wall_plane = plane1_it->second;
        }

        /// find the closest ceiling segment to this wall
        vector<double> distance_v;
        vector<int> indices_v;
        //TODO: this usage of KNNFinder should be checked if is the closest segment
        finder.FindKnn (wall_segment, 1, distance_v, indices_v);
        //printf ("indices size: %d \n", indices_v.size ());

        LaserPoint closest_ceil_point;
        closest_ceil_point = ceiling_lp[indices_v[0]];
        int closest_ceil_segNo;
        closest_ceil_segNo = closest_ceil_point.SegmentNumber ();

        /// obtaining the closest ceil segment
        LaserPoints closest_ceil_segment;
        auto ceilSeg_it = ceiling_segments_map.find(closest_ceil_segNo);
        if(ceilSeg_it != ceiling_segments_map.end ()){
            closest_ceil_segment = ceilSeg_it->second;
        }
        /// obtaining the plane of the ceil segment
        Plane ceil_plane;
        auto plane2_it = ceiling_planes_map.find (closest_ceil_segNo);
        if (plane2_it != ceiling_planes_map.end ()) {
            ceil_plane = plane2_it->second;
        }
        /* DEBUG */
        // write the closest ceil segment to check
        //strcpy (str_root, root);
        //closest_ceil_segment.Write ((strcat (str_root, "closest_ceil_segment.laser")), false);
        /* END DEBUG */


        /// extend two segments
        LaserPoints wall_extended_seg, ceil_extended_seg;
        bool extension=false;
        double max_intersection_dist = 10000.0; /// infinity
        /// extend two segments to the intersection line if their planes intersect and update segments_map
        if(Extend_Segments_to_Intersection (wall_segment, closest_ceil_segment, wall_plane, ceil_plane,
                                            max_intersection_dist, wall_extended_seg, ceil_extended_seg, extension)){

            //printf ("extending segments... \n");
            if(extension){
               // fprintf (statfile, "%4d, %4d, %s, %.2f \n", seg_no1, seg_no2, "Yes", extension_dist);
            }
            /// we update the segments_map with the new extended segments
            auto seg1_it = walls_segments_map.find(wallSeg_no);
            if (seg1_it != walls_segments_map.end ()){
                seg1_it->second = wall_extended_seg;
            }
            /// updating Wallsegment: Wallsegment belongs to the outer for-loop but should be updated every time inside the inner loop
            wall_segment = wall_extended_seg; // very important.

            /// second segment
            auto seg2_it = ceiling_segments_map.find(closest_ceil_segNo);
            if(seg2_it != ceiling_segments_map.end ()){
                seg2_it-> second = ceil_extended_seg;
            }

        } // end of segments extension
    }

    /// collect the extended walls
/*    LaserPoints extended_walls;
    for (auto &wall : walls_segments_map){
        extended_walls.AddPoints (wall.second);
    }

    /// collect the extended ceilings
    LaserPoints extended_ceil;
    for (auto &ceil : ceiling_segments_map){
        extended_ceil.AddPoints (ceil.second);
    }*/

    printf ("\n");
    printf ("Generating minimum rectangles for walls ... wait \n");
    ObjectPoints rectanglesWalls_corners;
    LineTopologies rectanglesWalls_edges;
    int next_number; // initialized later
    int line_number=0;
    LaserPoints extendedWall_laserpoints;
    for (auto &seg_map : walls_segments_map){
        int seg_no = seg_map.first;
        LaserPoints segment = seg_map.second;
        Plane plane = walls_planes_map.find (seg_no)->second; //planes remain the same and dont change
        extendedWall_laserpoints.AddPoints (segment);
        /// generating the minimum rectangle in 3D for extended segments
        ObjectPoints corners; // output
        LineTopology edges;   // output // this is not used
        EnclosingRectangle_Rotation3D (segment, plane, 0.10, corners, edges, root);

        /*DEBUG*/
/*        strcpy (str_root,root);
        corners.Write(strcat(str_root, "corners.objpts"));
        LineTopologies edgess;
        edgess.push_back (edges);
        strcpy (str_root,root);
        edgess.Write(strcat(str_root, "edges.top"), false);*/
        /*END of DEBUG*/

        // Store points and topology in one file
        if (rectanglesWalls_corners.empty()){
            next_number = 0;
        } else {
            next_number = (rectanglesWalls_corners.back ()).Number () + 1;
        }

        /// update the numbering for objectpoints of corners
        PointNumber pnumber;
        LineTopology rectangle_edges;
        for (int i=0; i < corners.size() ; i++){
            pnumber = PointNumber(next_number + i);
            //corner = ObjectPoint(corners[i], pnumber, cov3d);
            corners[i].NumberRef () = pnumber;
            rectanglesWalls_corners.push_back(corners[i]);
            rectangle_edges.push_back(pnumber);  // making the linetopology
        }
        rectangle_edges.push_back(PointNumber(next_number)); // Close the polygon

        if (rectangle_edges.IsClockWise(rectanglesWalls_corners)){
            rectangle_edges.MakeCounterClockWise(rectanglesWalls_corners);
        }else{
            rectangle_edges.MakeClockWise(rectanglesWalls_corners);
        }

        rectangle_edges.Number () = seg_no; //++line_number;
        rectanglesWalls_edges.push_back (rectangle_edges);
    }

    strcpy (str_root, root);
    rectanglesWalls_corners.Write(strcat (str_root, "rectanglesWalls_corners.objpts"));
    strcpy (str_root, root);
    rectanglesWalls_edges.Write(strcat (str_root, "rectanglesWalls_edges.top"), false);

    strcpy (str_root, root);
    extendedWall_laserpoints.Write(strcat(str_root, "extendedWall_laserpoints.laser"), false);


    printf ("\n");
    printf ("Generating minimum rectangles for ceilings ... wait \n");
    ObjectPoints rectanglesCeil_corners;
    LineTopologies rectanglesCeil_edges;
    int next_number_ceil; // initialized later
    //int line_number=0;
    LaserPoints extendedCeil_laserpoints;
    for (auto &seg_map : ceiling_segments_map){
        int seg_no = seg_map.first;
        LaserPoints segment = seg_map.second;
        Plane plane = ceiling_planes_map.find (seg_no)->second; //planes remain the same and dont change
        extendedCeil_laserpoints.AddPoints (segment);
        /// generating the minimum rectangle in 3D for extended segments
        ObjectPoints corners; // output
        LineTopology edges;   // output // this is not used
        EnclosingRectangle_Rotation3D (segment, plane, 0.10, corners, edges, root);

        /*DEBUG*/
/*        strcpy (str_root,root);
        corners.Write(strcat(str_root, "corners.objpts"));
        LineTopologies edgess;
        edgess.push_back (edges);
        strcpy (str_root,root);
        edgess.Write(strcat(str_root, "edges.top"), false);*/
        /*END of DEBUG*/

        // Store points and topology in one file
        if (rectanglesCeil_corners.empty()){
            next_number_ceil = 0;
        } else {
            next_number_ceil = (rectanglesCeil_corners.back ()).Number () + 1;
        }

        /// update the numbering for objectpoints of corners
        PointNumber pnumber;
        LineTopology rectangle_edges;
        for (int i=0; i < corners.size() ; i++){
            pnumber = PointNumber(next_number_ceil + i);
            //corner = ObjectPoint(corners[i], pnumber, cov3d);
            corners[i].NumberRef () = pnumber;
            rectanglesCeil_corners.push_back(corners[i]);
            rectangle_edges.push_back(pnumber);  // making the linetopology
        }
        rectangle_edges.push_back(PointNumber(next_number_ceil)); // Close the polygon

        if (rectangle_edges.IsClockWise(rectanglesCeil_corners)){
            rectangle_edges.MakeCounterClockWise(rectanglesCeil_corners);
        }else{
            rectangle_edges.MakeClockWise(rectanglesCeil_corners);
        }

        rectangle_edges.Number () = seg_no; //++line_number;
        rectanglesCeil_edges.push_back (rectangle_edges);
    }

    strcpy (str_root, root);
    rectanglesCeil_corners.Write(strcat (str_root, "rectanglesCeil_corners.objpts"));
    strcpy (str_root, root);
    rectanglesCeil_edges.Write(strcat (str_root, "rectanglesCeil_edges.top"), false);
    strcpy (str_root, root);
    extendedCeil_laserpoints.Write(strcat(str_root, "extendedCeil_laserpoints.laser"), false);

}

/*
 *  Main goal of this function: extension of segments to each other
 * Modeling interior walls, floor and ceilings:
 * 1. input: all the walls, floors and ceilings which are segmented.
 *  Labels are optional and not necessary for this function(wall=4, floor=5 and ceiling=6).
 * 2. We assume the input walls are generalized.
 * 3. This function first make sure all segments are intersecting in a specific threshold by extending the
 * segment to the intersection line
 * 4. A minimum_enclosing_rectangle_3D will be generated for all segments (in 3D e.g. slanted walls, ramps)
 * 5. The plane for each segment should be an input from the generalization step otherwise a FitPlane method will be used
 * in place.
 * The output would be written to the disk and later would be input to GenerateVolumetricWalls() to generate boxes of each object.
 * NOTE: planes can be empty, then the function calculate the planes per segment using FitPlane function
 * */

void ModelingInteriorWalls (const LaserPoints &laserPoints, const Planes &planes,
                            double max_intersection_dist, int min_segment_size, char *root, bool verbose){

    char str_root[500];
    strcpy (str_root, root);

    if (!laserPoints.HasAttribute (SegmentNumberTag)){
        printf ("Error: Laserpoints should have SegmentNumberTag! \n");
        EXIT_FAILURE;
    }

    /// make a list of segments
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(laserPoints, SegmentNumberTag, root);

    /// make a map of planes
    /// the assumption is that plane_number and segmentNumber are the same for each segment
    std::map<int, Plane> planes_temp_map;
    if(!planes.empty ()){
        for(auto &pl : planes) {
            planes_temp_map.insert (std::pair<int, Plane> (pl.Number (), pl));
        }
    }

    /// sort segments by size of the segment
    sort(segments_vec.begin (), segments_vec.end (), compare_lp_size );

    /// make a map of segment_numbers and segments,
    /// fit a plane to each segment (if there is no plane) using PlaneFitting function and least square
    std::map<int, LaserPoints> segments_map;
    std::map<int, Plane> planes_map;
    for(auto &s : segments_vec){
        if(s.HasAttribute (SegmentNumberTag) && s.size () > min_segment_size){
            segments_map.insert (std::pair<int, LaserPoints> (s[0].SegmentNumber (), s));
            /// we assume segmentNumber and planeNumber are the same for each segment
            auto pl_m_it = planes_temp_map.find(s[0].SegmentNumber ());
            if(pl_m_it != planes_temp_map.end ()){
                planes_map.insert (std::pair<int, Plane> (s[0].SegmentNumber (), pl_m_it->second));
                //planes_vec.push_back (planes_map.find(s[0].SegmentNumber ())->second);
            }else {
                /// else fit a plane to the segment points
                Plane plane;
                plane = s.FitPlane (s[0].SegmentNumber ());
                plane.Number () = s[0].SegmentNumber ();
                planes_map.insert (std::pair <int, Plane> (s[0].SegmentNumber (), plane));
                //planes_vec.push_back (s.FitPlane (s[0].SegmentNumber ()));
            }
        }
    }

    /// compare segments two by two for intersection and extension to the intersection
    FILE *statfile;
    strcpy (str_root,root);
    statfile = fopen(strcat(str_root, "intersection_info.txt"),"w");
    fprintf(statfile, "seg1, seg2, extension, extension_dist \n");
    //std::map<int, LaserPoints>::iterator seg_m_it1;
    for (auto seg_m_it1 = segments_map.begin (); seg_m_it1 != segments_map.end (); seg_m_it1++){
        int seg_no1 = seg_m_it1->first;
        LaserPoints segment1 = seg_m_it1->second;
        Plane plane1;
        auto plane1_it = planes_map.find (seg_no1);
        if ( plane1_it != planes_map.end ()){
            plane1 = plane1_it->second;
        }

        /// get the second segment
        auto seg_m_it2 = std::next (seg_m_it1);
        for(; seg_m_it2 != segments_map.end (); seg_m_it2++){
            if (verbose){
                printf ("Iteration #: %d and %d \n", distance(segments_map.begin (), seg_m_it1),
                        distance(segments_map.begin (), seg_m_it2)); //debug
            }

            int seg_no2 = seg_m_it2->first;
            LaserPoints segment2 = seg_m_it2->second;
            Plane plane2;
            auto plane2_it = planes_map.find (seg_no2);
            if (plane2_it != planes_map.end ()){
                plane2 = plane2_it->second;
            }

            printf ("intersecting segments: %d (%d) and %d (%d)  \r",
                    seg_no1, segment1.size (), seg_no2, segment2.size ());
            // print for debug
            //if(verbose){ /// plane numbers should be the same as segment numbers
            //    printf ("intersecting planes: %d and %d  \n", plane1.Number (), plane2.Number ());
            //}
/*            if (seg_no1 == 200 && seg_no2 == 940){
                printf ("debug \n");
            }*/

            /// the distance of two segments
            vector <double> extension_dist_v;
            extension_dist_v = segment1.Distance (segment2, 1.0);
            double extension_dist=0.0;
            if (!extension_dist_v.empty ()){
                extension_dist = *min_element (extension_dist_v.begin (), extension_dist_v.end ());
            }

            LaserPoints extended_seg1, extended_seg2;
            bool extension=false;

            /// NOTE: intersection_dist is calculated from the intersection line to each segment, but extension_dist
            /// is calculated from segments laser points to each other. Maybe this comparison cause problem in some cases.
            if (extension_dist > max_intersection_dist){
                fprintf (statfile, "%4d, %4d, %s, %.2f \n", seg_no1, seg_no2, "No", extension_dist);
                continue;
            }

            /// extend two segments to the intersection line if their planes intersect and update segments_map
            if(Extend_Segments_to_Intersection (segment1, segment2, plane1, plane2,
                                                max_intersection_dist, extended_seg1, extended_seg2, extension)){

                //printf ("extending segments... \n");
                if(extension){
                    fprintf (statfile, "%4d, %4d, %s, %.2f \n", seg_no1, seg_no2, "Yes", extension_dist);
                }
                /// we update the segments_map with the new extended segments
                auto seg1_it = segments_map.find(seg_no1);
                if (seg1_it != segments_map.end ()){
                    seg1_it->second = extended_seg1;
                }
                /// updating segment1: Segment1 belongs to the outer for-loop but should be updated every time inside the inner loop
                segment1 = extended_seg1; // very important.

                /// second segment
                auto seg2_it = segments_map.find(seg_no2);
                if(seg2_it != segments_map.end ()){
                    seg2_it-> second = extended_seg2;
                }

            } // end of segments extension
            if (!extension){
                fprintf (statfile, "%4d, %4d, %s, %.2f \n", seg_no1, seg_no2, "No", extension_dist);
            }
        } // end of second for
    } // end of first for

    printf ("\n");
    printf ("Generating minimum rectangles ... wait \n");
    ObjectPoints rectangles_corners;
    LineTopologies rectangles_edges;
    int next_number; // initialized later
    int line_number=0;
    LaserPoints extended_laserpoints;
    for (auto &seg_map : segments_map){
        int seg_no = seg_map.first;
        LaserPoints segment = seg_map.second;
        Plane plane = planes_map.find (seg_no)->second; //planes remain the same and dont change
        extended_laserpoints.AddPoints (segment);
        /// generating the minimum rectangle in 3D for extended segments
        ObjectPoints corners; // output
        LineTopology edges;   // output // this is not used
        EnclosingRectangle_Rotation3D (segment, plane, 0.10, corners, edges, root);

        /*DEBUG*/
/*        strcpy (str_root,root);
        corners.Write(strcat(str_root, "corners.objpts"));
        LineTopologies edgess;
        edgess.push_back (edges);
        strcpy (str_root,root);
        edgess.Write(strcat(str_root, "edges.top"), false);*/
        /*END of DEBUG*/

        // Store points and topology in one file
        if (rectangles_corners.empty()){
            next_number = 0;
        } else {
            next_number = (rectangles_corners.back ()).Number () + 1;
        }

        /// update the numbering for objectpoints of corners
        PointNumber pnumber;
        LineTopology rectangle_edges;
        for (int i=0; i < corners.size() ; i++){
            pnumber = PointNumber(next_number + i);
            //corner = ObjectPoint(corners[i], pnumber, cov3d);
            corners[i].NumberRef () = pnumber;
            rectangles_corners.push_back(corners[i]);
            rectangle_edges.push_back(pnumber);  // making the linetopology
        }
        rectangle_edges.push_back(PointNumber(next_number)); // Close the polygon

        if (rectangle_edges.IsClockWise(rectangles_corners)){
            rectangle_edges.MakeCounterClockWise(rectangles_corners);
        }else{
            rectangle_edges.MakeClockWise(rectangles_corners);
        }

        rectangle_edges.Number () = seg_no; //++line_number;
        rectangles_edges.push_back (rectangle_edges);
    }

    strcpy (str_root, root);
    rectangles_corners.Write(strcat (str_root, "rectangle_corners.objpts"));
    strcpy (str_root, root);
    rectangles_edges.Write(strcat (str_root, "rectangle_edges.top"), false);
    strcpy (str_root, root);
    extended_laserpoints.Write(strcat(str_root, "extended_laserpoints.laser"), false);

    fclose(statfile);

}


/* This function generates the input for GenerateVolumetricWalls function.
 * This function generates a map of wall thickness (offset_distance) based on the Residual attributes in each segment
 * Also two thresholds are suggested if the user wants to have lower and upper threshold for wall thickness
 * If the two thresholds are left as default=0.0 then the function uses the default residuals. */
std::map<int, double> Generate_offset_dist_map (const LaserPoints &mergedSegments, double lower_wall_thickness,
                                                double upper_wall_thickness){

    if (!mergedSegments.HasAttribute (ResidualTag)){
        printf ("Error: Laserpoints should have ResidulaTag as offset_distance or wall_thickness! \n");
        EXIT_FAILURE;
    }

    if (!mergedSegments.HasAttribute (SegmentNumberTag)){
        printf ("Error: Laserpoints should have SegmentNumberTag! \n");
        EXIT_FAILURE;
    }
    /// make a list of segments
    char* root; // we dont use this, just an arguments
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(mergedSegments, SegmentNumberTag, root);

    /// make a map of distances extracted from the residual values
    /// the assumption is that plane_number and segmentNumber are the same for each segment
    std::map<int, double> offset_distance_map;
    for(auto &s : segments_vec) {
        if(lower_wall_thickness == 0.0 && upper_wall_thickness == 0.0 ){
            offset_distance_map.insert (std::pair<int, double> (s[0].SegmentNumber (), s[0].Residual ()));
        } else {
            if (s[0].Residual () <= upper_wall_thickness){
                offset_distance_map.insert (std::pair<int, double> (s[0].SegmentNumber (), lower_wall_thickness/2)); /// divided by 2 is because we are calculating offset
            }
            if (s[0].Residual () > upper_wall_thickness){
                offset_distance_map.insert (std::pair<int, double> (s[0].SegmentNumber (), upper_wall_thickness/2));
            }
            if(!s.HasAttribute (ResidualTag)){
                offset_distance_map.insert (std::pair<int, double> (s[0].SegmentNumber (), lower_wall_thickness/2));
            }
        }
    }
    return offset_distance_map;
}


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

void GenerateVolumetricWalls (const ObjectPoints &wall_rectangles_vertices, LineTopologies &wall_rectangle_edges,
                              const std::map<int, double> &offset_distance_map, double fix_offset_dist, char *root, bool verbose){

    char str_root[500];
    strcpy (str_root, root);

    ObjectPoints walls_vertices;
    LineTopologies walls_faces;
    LineTopology walls_edges;
    int rectnagle_count=0;
    if(wall_rectangles_vertices.empty () || wall_rectangle_edges.empty ()) {
        printf ("Error: rectnagle vertices or edges are empty!!! \n");
        exit (0);
    }

    /// READING THE TOPOLOGIES PER RECTANGLE
    for (auto &rectangle : wall_rectangle_edges){
        ++rectnagle_count;
        /// collect the vertices of the rectangle
        ObjectPoints rectangle_vertices;
        /// initialize the next_number for numbering of vertices
        int next_number;
        if (walls_vertices.empty()){
            next_number = 0;
        } else {
            next_number = (walls_vertices.back ()).Number () + 1;
        }
        /// remove the last pointnumber of the polygon,
        /// we don't need a closed polygon as a face
        rectangle.pop_back ();
        //rectangle.Sort ();   /// not needed becasue we renumber the the pointnumbers
        for (int i=0; i < rectangle.size (); i++){
            ObjectPoint vertex;
            vertex = *wall_rectangles_vertices.GetPoint (rectangle[i]);
            /// renumebr the vertices pointnumber for generating the boxes
            vertex.Number () = next_number +i; // very important for numbering
            rectangle_vertices.push_back (vertex);
        }
        /// obtain the offset_distance
        double offset_dist;
        if(!offset_distance_map.empty ()){
            auto offset_dist_it = offset_distance_map.find (rectangle.Number ());
            if (offset_dist_it != offset_distance_map.end ()){
                offset_dist = offset_dist_it->second;
            } else offset_dist = fix_offset_dist;
        } else {
            offset_dist = fix_offset_dist;
        }
        /// convert the rectangle to a 3Dbox with 6 faces
        if (rectangle_vertices.empty ()){
            printf("Rectangle # %d is empty!!! \n", rectnagle_count);
            continue;
        }
        LineTopologies threeDbox_faces;
        ObjectPoints threeDbox_vertices;
        Min3DRectangle_to_3DFaces (rectangle_vertices, offset_dist, threeDbox_vertices, threeDbox_faces);
        for (auto &face : threeDbox_faces) face.Number () = rectangle.Number ();
        if(verbose){ //debug
            strcpy (str_root, root);
            threeDbox_vertices.Write(strcat (str_root, "threeDbox_vertices.objpts"));
            strcpy (str_root, root);
            threeDbox_faces.Write(strcat (str_root, "threeDbox_faces.top"), false);
        }

        walls_vertices.insert(walls_vertices.end (), threeDbox_vertices.begin (), threeDbox_vertices.end ());
        walls_faces.insert(walls_faces.end (), threeDbox_faces.begin (), threeDbox_faces.end ());
        // debug
/*        strcpy (str_root, root);
        walls_vertices.Write(strcat (str_root, "walls_vertices.objpts"));
        strcpy (str_root, root);
        walls_faces.Write(strcat (str_root, "walls_faces.top"), false);*/
    }


    printf("In total %d rectanlge/boxes are generated. \n", rectnagle_count);
    strcpy (str_root, root);
    walls_vertices.Write(strcat (str_root, "walls_vertices.objpts"));
    strcpy (str_root, root);
    walls_faces.Write(strcat (str_root, "walls_faces.top"), false);

}


///
/*
 * This function gets the permanent structure (walls, floor and ceiling) point clouds as input and fit a box to each segment.
 * Each permanent-structure layer has a label (wall=4, floor=5 and ceiling=6) and it is processed based on segments.
 * The function by default, fit a box to the segment and extract the width and height from the segment, if there is no given parameter.
 * If the wall width is not fixed, the function calculates the residuals of each wall segment and consider it as
 * the thickness of the wall. If the thickness is bigger than a threshold then is set to Width_max and if is smaller
 * than a threshold is set to Width_min.
 * The same for the wall height, if a fixed height is not given, the height of the wall is extended to the CLOSEST floor
 * and ceiling segment. If the closest floor or ceiling is sloped or ramp then the max and min of the segment is picked respectively.
 * The output, is the corner and edges of the boxes fitted to each segment.
 * For better result, in a pre-processing step, all the parallel segments in the wall, floor or ceiling
 * should be merged to one patch.
 * The function doesn't decide on the class of the segment based on the normal vector angle (vertical or horizontal).
 * Therefore, everything that is as wall, floor or ceiling can have arbitrary normal angle.
 * */


/// not a complete function
void ModelingInteriorWalls(const LaserPoints &segmented_walls, const LaserPoints &segmented_floor,
                           const LaserPoints &segmented_ceiling,
                           char *root, int min_segment_size,
                           double min_width = NULL, double max_width = NULL , double fixed_height = NULL)
                                   //char *corners, char*edges,char *OFFfile,
{
    std::clock_t start;
    double duration;
    start = std::clock ();

    char str_root[500];
    strcpy (str_root, root); // initialize the str_root with root string


    printf (" input laser points size:  %d \n ", segmented_walls.size ());
    printf (" input surface point size:  %d \n ", segmented_floor.size ());  /// walls or wall_patches
    printf (" input trajectory points size:  %d \n ", segmented_ceiling.size ());

    if (segmented_walls.size () < 2) {
        printf("walls file is empty or not readable! \n");
        exit(0);
    }

    if (segmented_floor.size () < 2) {
        printf("floor file is empty or not readable! \n");
        exit(0);
    }

    if (segmented_ceiling.size () < 2) {
        printf("ceiling file is empty or not readable! \n");
        exit(0);
    }

/*    /// remove duplicate points because it affects the enclosingrectangle function
    segmented_lp.RemoveDoublePoints();
    cout << "nr of points after RemoveDoublePoints: " << segmented_lp.size() << endl;
    /// remove duplicate points because it affects the enclosingrectangle function
    segmented_lp.RemoveDoublePoints();
    cout << "nr of points after RemoveDoublePoints: " << segmented_lp.size() << endl;
    /// remove duplicate points because it affects the enclosingrectangle function
    segmented_lp.RemoveDoublePoints();
    cout << "nr of points after RemoveDoublePoints: " << segmented_lp.size() << endl;*/
}
