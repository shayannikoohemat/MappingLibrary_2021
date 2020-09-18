//
// Created by NikoohematS on 21-2-2019.
//

#include "CreateAdjGraph.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <map>
#include "indoor_reconstruction.h"
#include "post_processing.h"
#include "visualization_tools.h"
#include "AdjGraph.h"

/// printing in DOT Graph format
void PrintGraph (LineTopologies edges, char *root){

    char str_root[500];
    FILE *adjGraph;
    strcpy (str_root,root);
    adjGraph = fopen(strcat(str_root, "AdjGraph.dot"),"w");
    printf("\n PRINT the GRAPH in DOT format \n");

    //cout << "graph adjGraph { " << endl;
    fprintf(adjGraph, "graph adjGraph { \n");
    for (auto &e : edges){
        PointNumberList pnList;
        pnList = e.PointNumberListReference ();
        int begin_point, end_point;
        begin_point = pnList[0].Number ();
        end_point = pnList[1].Number ();
        fprintf(adjGraph, "%d" , begin_point);
        fprintf(adjGraph, " -- ");
        fprintf (adjGraph, "%d" , end_point);
        //cout << " [distance = " << e.distance_prop << ", " ;
        //cout <<  " angle = " << e.angle_prop << "];" ;
        fprintf(adjGraph, "\n");
    }
    fprintf(adjGraph, " } \n");
    //cout << "}" ;
    printf("\n");
}


/// this function makes an adjacency graph of segmented point clouds
/// there are extra calculation for minimum_rectangle for special cases
void CreateAdjGraph (LaserPoints &laserPoints, int min_segment_size, double dist_threshold,
                      char *root, bool verbose) {

    bool sort_segments_vector = true;

    char str_root[500];
    strcpy (str_root, root);

    if (!laserPoints.HasAttribute (SegmentNumberTag)) {
        printf ("Error: Laserpoints should have SegmentNumberTag! \n");
        EXIT_FAILURE;
    }

    /// make a list of segments
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag (laserPoints, SegmentNumberTag);

    vector<int> segment_nr_vec;
    segment_nr_vec = laserPoints.AttributeValues (SegmentNumberTag);

    /// sort segments by segment number
    if(sort_segments_vector) {
        printf("Sorting segments by SegmentNumberTag...wait \n");
        sort(segments_vec.begin (), segments_vec.end (), compare_lp_segmenttag);  // too expensive
    }

    /// make a map of planes
    /// make a map of segment_numbers and segments,
    /// make a map of min_rectangles and their segmentNumbers
    /// fit a plane to each segment (if there is no plane) using PlaneFitting function and least square
    std::map<int, LaserPoints> segments_map;
    std::map<int, Plane> planes_map;
    std::map<int, MinimumRectangle> minRectangles_map;
    for (auto &s : segments_vec) {
        int seg_nr = s[0].SegmentNumber ();
        if (s.HasAttribute (SegmentNumberTag) && s.size () > min_segment_size) {
            /// make a map of segment number and its corresponding laserpoints
            segments_map.insert (std::pair<int, LaserPoints> (seg_nr, s));
            /// fit a plane to the segment points and make a map
            Plane plane;
            plane = s.FitPlane (seg_nr);
            plane.Number () = seg_nr;
            planes_map.insert (std::pair<int, Plane> (seg_nr, plane));
            //planes_vec.push_back (s.FitPlane (s[0].SegmentNumber ()));

            ObjectPoints corners;
            LineTopology edges;
            //LineTopologies edges_s;
            //segment.Read("E:/publication_data/modeling/data/test/seg58.laser");  //slanted_wall2
            //segment.Read("E:/publication_data/modeling/data/tudelft/modeling_walls/floor_final_modifiedSegmentation.laser");
            EnclosingRectangle_Rotation3D (s, 0.10, corners, edges, root);
            double area;
            area = edges.CalculateArea (corners);

            MinimumRectangle minimumRectangle;
            minimumRectangle.rect_area = area;
            minimumRectangle.rect_corners = corners;
            minimumRectangle.rect_edges = edges;
            minimumRectangle.rect_number = static_cast<int>(s.size ());

            minRectangles_map.insert (std::pair<int, MinimumRectangle> (seg_nr, minimumRectangle));

        }
    }

    /// sort segments by size of the segment
    sort (segments_vec.begin (), segments_vec.end (), compare_lp_size);

    /// compare segments two by two for checking the adjacency and making the graph
    FILE *statfile;
    strcpy (str_root, root);
    statfile = fopen (strcat (str_root, "adjacency_info.txt"), "w");
    fprintf (statfile, "seg1, seg2, angle, distance \n");
    //std::map<int, LaserPoints>::iterator seg_m_it1;
    vector<int> node_numbers;
    ObjectPoints node_points; /// segment centers
    LineTopologies edge_lines;
    Covariance3D cov3d;
    cov3d = Covariance3D(0, 0, 0, 0, 0, 0); /// for making objpoint
    int     line_number      = 0;
    for (auto seg_m_it1 = segments_map.begin (); seg_m_it1 != segments_map.end (); seg_m_it1++) {
        int seg_no1 = seg_m_it1->first;
        LaserPoints segment1 = seg_m_it1->second;
        Plane plane1;
        auto plane1_it = planes_map.find (seg_no1);
        if (plane1_it != planes_map.end ()) {
            plane1 = plane1_it->second;
        }

        /// there are two methods for getting the center of the segment
        Position3D seg_center_pos;
        //segment1.Centroid(point_list, segment_nr1) /// method2, needs the pointlist
        seg_center_pos = plane1.CentreOfGravity (); /// method 1, doesnt need pointlist
        ObjectPoint seg_center_point;
        seg_center_point = ObjectPoint(seg_center_pos, seg_no1, cov3d);
        node_points.push_back (seg_center_point);

        /// get the second segment
        auto seg_m_it2 = std::next (seg_m_it1);
        for (; seg_m_it2 != segments_map.end (); seg_m_it2++) {
            if (verbose) {
                printf ("Iteration #: %d and %d \n", distance (segments_map.begin (), seg_m_it1),
                        distance (segments_map.begin (), seg_m_it2)); //debug
            }

            int seg_no2 = seg_m_it2->first;
            LaserPoints segment2 = seg_m_it2->second;
            Plane plane2;
            auto plane2_it = planes_map.find (seg_no2);
            if (plane2_it != planes_map.end ()) {
                plane2 = plane2_it->second;
            }

            printf ("Checking segments: %d (%d) and %d (%d)  \r",
                    seg_no1, segment1.size (), seg_no2, segment2.size ());

            /// the distance of two segments
            vector<double> dist_v;
            dist_v = segment1.Distance (segment2, 1.0);
            double distance = 0.0;
            if (!dist_v.empty ()) {
                distance = *min_element (dist_v.begin (), dist_v.end ());
            }

            double angle;
            /// calculate planes' normal
            Vector3D normal1, normal2;
            normal1 = (plane1).Normal ();
            normal2 = (plane2).Normal ();
            /// plane angle is calculated by angle between their normals
            angle = (acos (normal1.DotProduct (normal2) / (normal1.Length () * normal2.Length ()))) * 180.0 / PI;

            /// making the graph based on the proximity
            if (distance <= dist_threshold) {
                line_number++;
                LineTopology edge_line;
                edge_line = LineTopology(line_number, 1, seg_no1, seg_no2);
                edge_lines.push_back (edge_line);
            }
        } // end of second segment
    } // end of first segment loop

    strcpy (str_root,root);
    node_points.Write(strcat(str_root, "node_points.objpts")), strcpy (str_root,root);
    edge_lines.Write(strcat(str_root, "edge_lines.top"), false);
    strcpy (str_root,root);
    laserPoints.Write(strcat(str_root, "laserPoints.laser"), false);

    PrintGraph (edge_lines, root);
}