//
// Created by NikoohematS on 24-3-2017.
//

#include <iostream>
#include <vector>
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "visualization_tools.h"

/// This function calculates a 3Dbox around each segment of the laserpoints,
/// by extruding the 2D minimum rectangle in the z-direction.
/// The height of the box is user defined or extracted from the height of the segment.
/// Corners and polygon_lines are the output.
/// The box is not oriented and is a vertical box.
//// This function is more meant for vertical segments not horizontal or slanted.
void Minimum3DBox(ObjectPoints &corners, LineTopologies &polygon_lines, LaserPoints segmented_lp,
                  double min_z, double max_z, double height){

    segmented_lp.RemoveDoublePoints();
    cout << "nr of points after RemoveDoublePoints: " << segmented_lp.size() << endl;

    LineTopology polygon_line;

    vector <int>                 segment_numbers;
    vector <int>::iterator       segment_number;
    segment_numbers = segmented_lp.AttributeValues(SegmentNumberTag);
    for (segment_number = segment_numbers.begin(); segment_number != segment_numbers.end(); segment_number++) {
        LaserPoints seg_laser_points;
        seg_laser_points = segmented_lp.SelectTagValue(SegmentNumberTag, *segment_number);

        LaserPoints lp;
        DataBoundsLaser db;

        lp = seg_laser_points;
        db = lp.DeriveDataBounds(0);

        ObjectPoint obj_pnt;
        //LineTopologies polygon_lines;

        /// Find the minimum enclosing rectangle
        lp.DeriveTIN();
        if (!polygon_line.empty()) polygon_line.erase(polygon_line.begin(), polygon_line.end());
        lp.EnclosingRectangle(0.1, corners, polygon_line); // corners and polygon_line are output of the function

        int next_number; /// the next_number is the number after the last number in the corners file
        if (corners.empty()) next_number = 4; /// later next_number-4 =0
        else next_number = (corners.end() - 1)->Number() + 1;

        /// add z values to 4 corners to make upper rectangle
        for (int i=next_number -4; i < next_number ; i++){
            if(max_z ==0.0){
                corners[i].Z() = db.Maximum().GetZ(); // upper rectangle
            }else{
                corners[i].Z () = max_z;
            }
        }

        /// constructing lower polygon
        for (int i=0; i < 4; i++){

            obj_pnt.X() = corners[next_number -4 + i].X();
            obj_pnt.Y() = corners[next_number -4 + i].Y();
            /// if user defines the height or min_z then use it, otherwise use segment height
            if(min_z ==0.0){
                if (height == 0.0) {
                    obj_pnt.Z() = db.Minimum().GetZ();
                } else {
                    obj_pnt.Z() = db.Maximum().GetZ() - height;
                }
            }else{
                obj_pnt.Z() = min_z;
            }

            obj_pnt.Number() = next_number + i;
            corners.push_back(obj_pnt);
            polygon_line.push_back(PointNumber(next_number + i));
        }
        polygon_line.push_back(PointNumber(next_number)); // close the polygon // clockwise
        polygon_line.MakeCounterClockWise(corners);

        /// connecting upper rectangle to the below one
        //next_number = next_number +4;
        /// counter clockwise polygon
        polygon_line.push_back(PointNumber(next_number+1)); // 4-5 duplicate line
        polygon_line.push_back(PointNumber(next_number-3));
        polygon_line.push_back(PointNumber(next_number-2)); // 1-2 duplicate line
        polygon_line.push_back(PointNumber(next_number+2));
        polygon_line.push_back(PointNumber(next_number+3)); // 6-7 duplicate line
        polygon_line.push_back(PointNumber(next_number-1));

        /*     1<---------2
              /|		 /|
            /  |		/ |
           0---------->3  |
           |   |	   |  |
           |   |	   |  |
           |   |	   |  |
           |   5-------|->6
           |  /		   | /
           | /		   |/
           4<---------7            */

        polygon_lines.push_back(polygon_line);
    }

    //corners.Write("D://test//visualisation//doors.objpts");
    //polygon_lines.Write("D://test//visualisation//doors.top", false);

}


/*  This function makes a 3Dbox/or a polyhedron around a minimum3Drectangle. By 3D, we mean the rectangle has arbitrary orientation.
 * The input is the rectangle vertices and the output is the box edges and vertices.
 * The box is generated for a given offset from the rectangle plane.
 * The numbering of vertices starts from the number of input vertices plus the number of vertices of the offset rectangle/polygon.
 * The number of faces is: 2+n. (n is number of vertices of the polygon or rectangle)
 * The function is not tested for polyhedron.
 * */

void Min3DRectangle_to_3DBox (const ObjectPoints &rectangle_vertices, double offset_distance,
        ObjectPoints &threeDbox_vertices, LineTopology &threeDbox_edges){

    /// first we offset the plane of the rectangle to both sides
    ObjectPoints left_corners, right_corners;
    LineTopology left_edges, right_edges; /// are not used
    offset_polygon (rectangle_vertices, offset_distance, left_corners, right_corners, left_edges, right_edges);

    /// add left_polygon to the threeDbox
    for (int i=0; i < left_corners.size (); i++){
        threeDbox_vertices.push_back (left_corners[i]);
        threeDbox_edges.push_back (left_corners[i].Number ());
    }
    threeDbox_edges.push_back((threeDbox_vertices.front ()).Number ()); // close the polygon // clockwise

    /// initialize the next_number for numbering of vertices
    int next_number;
    if (threeDbox_vertices.empty()){
        next_number = 0;
    } else {
        next_number = (threeDbox_vertices.back ()).Number () + 1;
    }

    /// add right_polygon to the threeDbox
    PointNumber pnumber;
    for (int i=0; i < right_corners.size (); i++){
        pnumber = PointNumber(next_number + i);
        right_corners[i].NumberRef () = pnumber; // updating the PointNumber
        threeDbox_vertices.push_back (right_corners[i]); /// add the updated vertex to the threeDbox
        threeDbox_edges.push_back (pnumber);
    }
    threeDbox_edges.push_back(PointNumber(next_number)); // close the polygon // clockwise
    threeDbox_edges.MakeCounterClockWise(threeDbox_vertices);

    /// connecting upper rectangle to the below one
    /// counter clockwise polygon
    threeDbox_edges.push_back(PointNumber(next_number+1)); // 4-5 duplicate line
    threeDbox_edges.push_back(PointNumber(next_number-3));
    threeDbox_edges.push_back(PointNumber(next_number-2)); // 1-2 duplicate line
    threeDbox_edges.push_back(PointNumber(next_number+2));
    threeDbox_edges.push_back(PointNumber(next_number+3)); // 6-7 duplicate line
    threeDbox_edges.push_back(PointNumber(next_number-1));

}


// TODO: write a function to make a cube or a voxel to visualize voxels