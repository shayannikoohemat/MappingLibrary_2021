//
// Created by NikoohematS on 24-3-2017.
//

#include <iostream>
#include <vector>
#include "LaserPoints.h"
#include "LaserPoint.h"

void Minimum3DRectangle(ObjectPoints &corners, LineTopology &polygon_line, LaserPoints segmented_lp, double height){

    /// read laser points
    char* laserFile;
    //laserFile   = (char *) "D://test//visualisation//seg09.laser";  //morph_3rdFloor_small  //morph_small_83k_noFloorCeil
    //laserFile = (char*) "D://test//visualisation//walls.laser";
    laserFile = (char*) "D://test//indoor_reconstruction//3rdfloor_601040_final//"
            "space_partitioning_doordetection_3rdfloor//door_results_segmented.laser";
    segmented_lp.Read(laserFile);
    segmented_lp.RemoveDoublePoints();
    cout << "nr of points after RemoveDoublePoints: " << segmented_lp.size() << endl;

    LineTopologies polygon_lines;

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
            corners[i].Z() = db.Maximum().GetZ(); // upper rectangle
        }

        /// constructing below polygon
        for (int i=0; i < 4; i++){

            obj_pnt.X() = corners[next_number -4 + i].X();
            obj_pnt.Y() = corners[next_number -4 + i].Y();
            /// if user defines the height then use it, otherwise use segment height
            if (height == 0) {
                obj_pnt.Z() = db.Minimum().GetZ();
            } else {
                obj_pnt.Z() = db.Maximum().GetZ() - height;
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

    corners.Write("D://test//visualisation//doors.objpts");
    polygon_lines.Write("D://test//visualisation//doors.top", false);

}