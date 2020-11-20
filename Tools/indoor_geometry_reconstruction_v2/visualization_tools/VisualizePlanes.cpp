//
// Created by NikoohematS on 4-4-2017.
//

//
// Created by NikoohematS on 23-3-2017.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include "visualization_tools.h"


//VisulizePlane3D (LaserPoints segment_lp, double max_dist, ObjectPoints &plane_corners, LineTopology &plane_edges, bool);

/* for horizontal planes */
/// visualize planes on all segments of the points clouds using VisulizePlane3D function
void VisualizePlanes(LaserPoints segmented_laserpoints, int minsizesegment, double max_edge_dist,
                      ObjectPoints &corners, LineTopologies &polygons, bool verbose){

    vector<int>             segment_numbers;
    vector<int>::iterator   segment_it;

    segment_numbers = segmented_laserpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    printf ("Number of laser segments: %d\n", segment_numbers.size());

    LineTopology polygon;

    for(segment_it = segment_numbers.begin(); segment_it != segment_numbers.end(); segment_it++) {

        /// selecting points by segment and saving in segment_lpoints
        LaserPoints segment_lpoints;
        segment_lpoints = segmented_laserpoints.SelectTagValue(SegmentNumberTag, *segment_it);
        if (!polygon.empty()) polygon.erase(polygon.begin(), polygon.end());
        int polygon_cnt =0;
        if (segment_lpoints.size() > minsizesegment) {
            polygon_cnt++;
            /// fit a plane to the segment and generate plane' corners and edges
            VisulizePlane3D(segment_lpoints, max_edge_dist, corners, polygon, false);

/*            int next_number; /// the next_number is the number after the last number in the corners file
            if (corners.empty()) next_number = 0; /// later next_number-4 =0
            else next_number = (corners.end() - 1)->Number() + 1;
            for (int i=0; i < 4; i++){
                polygon.push_back(PointNumber(next_number + i));
            }
            polygon.push_back(PointNumber(next_number)); // close the polygon // clockwise
            polygon.MakeCounterClockWise(corners);*/

            polygons.push_back(polygon);
        }
        printf ("Number of polygons: %d\n", polygon_cnt);
    }

    //corners.Write("D://test//visualisation//walls_planes.objpts");
    //polygons.Write("D://test//visualisation//walls_planes.top", false);

    //corners.Write("D://test//indoor_reconstruction//3rdfloor_601040_final//2nd_iteration_walls//walls_planes.objpts");
    //polygons.Write("D://test//indoor_reconstruction//3rdfloor_601040_final//2nd_iteration_walls//walls_planes.top", false);
}

/* for slanted, horizontal and vertical planes */
/// visualize planes on all segments of the points clouds using EnclosingRectangle_Rotation3D function
void VisualizePlanes(LaserPoints segmented_laserpoints, int minsizesegment,
                     ObjectPoints &corners, LineTopologies &polygons, char* root, bool verbose){

    vector<int>             segment_numbers;
    vector<int>::iterator   segment_it;

    segment_numbers = segmented_laserpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    printf ("Number of laser segments: %d\n", segment_numbers.size());

    LineTopology polygon;
    int next_number; // initialized later
    for(segment_it = segment_numbers.begin(); segment_it != segment_numbers.end(); segment_it++) {
        /// selecting points by segment and saving in segment_lpoints
        LaserPoints segment_lpoints;
        segment_lpoints = segmented_laserpoints.SelectTagValue(SegmentNumberTag, *segment_it);
        segment_lpoints.RemoveDoublePoints (); /// for vertical segments could be harmful but necessary for TIN reconstrution
        if (!polygon.empty()) polygon.erase(polygon.begin(), polygon.end());
        int polygon_cnt =0;
        int seg_no;
        if (segment_lpoints.size() > minsizesegment) {
            polygon_cnt++;
            seg_no = segment_lpoints[0].SegmentNumber ();
            /// fit a plane to the segment and generate plane' corners and edges
            ObjectPoints corners_per_rectangle;
            LineTopology edges_per_rectnagle;
            EnclosingRectangle_Rotation3D (segment_lpoints, 0.10, corners_per_rectangle, edges_per_rectnagle, root);
            /// add the corners and lines to one file
            // Store points and topology in one file
            if (corners.empty()){
                next_number = 0;
            } else {
                next_number = (corners.back ()).Number () + 1;
            }

            /// update the numbering for objectpoints of corners
            PointNumber pnumber;
            LineTopology rectangle_edges;
            for (int i=0; i < corners_per_rectangle.size() ; i++){
                pnumber = PointNumber(next_number + i);
                //corner = ObjectPoint(corners[i], pnumber, cov3d);
                corners_per_rectangle[i].NumberRef () = pnumber;
                corners.push_back(corners_per_rectangle[i]);
                rectangle_edges.push_back(pnumber);  // making the linetopology
            }
            rectangle_edges.push_back(PointNumber(next_number)); // Close the polygon

            if (rectangle_edges.IsClockWise(corners)){
                rectangle_edges.MakeCounterClockWise(corners);
            }else{
                rectangle_edges.MakeClockWise(corners);
            }

            rectangle_edges.Number () = seg_no; //++line_number;
            polygons.push_back(rectangle_edges);
        }
        //printf ("Number of polygons: %d\n", polygon_cnt);
    }
}

void Visualize2DRectangles(vector< LaserPoints> &laserpoints_vec,ObjectPoints &rect_corners,
                           LineTopologies &rect_edges, LaserPointTag tag){

    int next_number=0;
    int line_number=0;
    for (auto &s : laserpoints_vec){ // s is a slice or a segment of laserpoints
        if (s.size() > 4){
            int seg_num = s[0].Attribute(tag);
            DataBoundsLaser db;
            db = s.DeriveDataBounds(0);
            /// Find the minimum enclosing rectangle for the segment in 2D
            s.DeriveTIN();
            ObjectPoints corners_per_rectangle;
            LineTopology edges_per_rectnagle;
            double max_edge_dist=0.10;
            s.EnclosingRectangle(max_edge_dist, corners_per_rectangle, edges_per_rectnagle);
            // Store points and topology in one file
            if (rect_corners.empty()){
                next_number = 0;
            } else {
                next_number = (rect_corners.back ()).Number () + 1;
            }
            /// update the numbering for objectpoints of corners
            PointNumber pnumber;
            LineTopology rectangle_edges_renumebr;
            for (int i=0; i < corners_per_rectangle.size() ; i++){
                pnumber = PointNumber(next_number + i);
                //corner = ObjectPoint(corners[i], pnumber, cov3d);
                corners_per_rectangle[i].NumberRef () = pnumber;
                /// set slice center z-value to the rectangle corners
                corners_per_rectangle[i].Z() = s.CentreOfGravity(s.TaggedPointNumberList(tag, seg_num)).GetZ();
                rect_corners.push_back(corners_per_rectangle[i]);
                rectangle_edges_renumebr.push_back(pnumber);  // making the linetopology
            }
            rectangle_edges_renumebr.push_back(PointNumber(next_number)); // Close the polygon
            if (rectangle_edges_renumebr.IsClockWise(rect_corners)){
                rectangle_edges_renumebr.MakeCounterClockWise(rect_corners);
            }else{
                rectangle_edges_renumebr.MakeClockWise(rect_corners);
            }

            rectangle_edges_renumebr.Number () = line_number++; //seg_num
            rect_edges.push_back(rectangle_edges_renumebr);
        }
    } // end of slices
}

