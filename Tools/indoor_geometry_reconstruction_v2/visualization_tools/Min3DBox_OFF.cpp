//
// Created by NikoohematS on 18-10-2017.
//

//
// Created by NikoohematS on 24-3-2017.
//

#include <iostream>
#include <vector>
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "visualization_tools.h"


/* NOTE: OFF (Object File Format) is a simple geometry format that is readable by meshlab, cloudcompare and ...  */

/// This function calculates a 3Dbox around each segment of the laserpoints, **NOT an oriented bbox**
/// by extruding the 2D minimum rectangle in the z-direction.
/// The height of the box is user defined or extracted from the height of the segment.
/// conrners and polygon_lines are the output and the result is written to a OFF format file.
void Min3DBox_OFF(ObjectPoints &corners, LineTopologies &polygon_lines, LaserPoints segmented_lpoints,
                  int min_segment_size, char *OFFfile, double min_z, double max_z, double height){

    double angle_threshold;//horizontal or vertical angle threshold ;
    angle_threshold = 10.0;

    /// remove duplicate points because it affects the enclosingrectangle function
    segmented_lpoints.RemoveDoublePoints();
    cout << "nr of points after RemoveDoublePoints: " << segmented_lpoints.size() << endl;

    LineTopology polygon_line;

    /// partitions laserpoints based on segments
    vector <int>                 segment_numbers;
    vector <int>::iterator       segment_number;
    segment_numbers = segmented_lpoints.AttributeValues(SegmentNumberTag);
    if (segment_numbers.size() == 0) cout << "There is no segmented points. Please add a file with segments." << endl;
    printf("Nmber of segments: %d \n", segment_numbers.size ());

    int box_cnt =0; // number of 3d boxes
    for (segment_number = segment_numbers.begin(); segment_number != segment_numbers.end(); segment_number++) {
        LaserPoints seg_laser_points;
        seg_laser_points = segmented_lpoints.SelectTagValue(SegmentNumberTag, *segment_number);
        if(seg_laser_points.size () < min_segment_size) continue;

        box_cnt++;
        LaserPoints lp;
        DataBoundsLaser db;
        LineTopologies polygon_lines_temp;

        /// get the databound of the segment
        lp = seg_laser_points;
        db = lp.DeriveDataBounds(0);

        ObjectPoint obj_pnt;
        /// Find the minimum enclosing rectangle for the segment
        lp.DeriveTIN();
        if (!polygon_line.empty()) polygon_line.erase(polygon_line.begin(), polygon_line.end());
        /// generated corners have Z value at 0
        lp.EnclosingRectangle(0.1, corners, polygon_line); // corners and polygon_line are output of the function

        int next_number; /// the next_number is the number after the last number in the corners file
        if (corners.empty()) next_number = 4; /// later next_number-4 =0
        else next_number = (corners.end() - 1)->Number() + 1;

        /// add z values to 4 corners to make upper polygon
        for (int i=next_number -4; i < next_number ; i++){
            if(max_z ==0.0){
                corners[i].Z() = db.Maximum().GetZ(); // upper rectangle
            }else{
                corners[i].Z () = max_z;
            }

        }
        /// remove the last pointnumber of the polygon,
        /// we don't need to close the polygon as a face
        polygon_line.pop_back();

        /// add the upper polygon to polygon_lines and erase polygon_line
        polygon_lines.push_back(polygon_line);  polygon_line.Erase();

        /// constructing below polygon
        for (int i=0; i < 4; i++){

            obj_pnt.X() = corners[next_number -4 + i].X();
            obj_pnt.Y() = corners[next_number -4 + i].Y();
            /// if user defines the height then use it, otherwise use segment height
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
        /// close the polygon // clockwise
        //polygon_line.push_back(PointNumber(next_number)); // we dont need to close the polygon as a face

        /// print polygon for debug
/*        printf ("lower polygon: \n");
        LineTopology::iterator line_it2;
        for (line_it2 = polygon_line.begin(); line_it2 != polygon_line.end(); line_it2++){
            printf(" %d ", line_it2->Number());
        }
        printf("\n");*/
        /// end of debug print

        polygon_line.MakeCounterClockWise(corners);
        /// add the below polygon (face) to polygon lines
        polygon_lines.push_back(polygon_line); polygon_line.Erase();

        /// Constructing front face: 0 4 7 3
        polygon_line.push_back(PointNumber(next_number-4));
        polygon_line.push_back(PointNumber(next_number-0));
        polygon_line.push_back(PointNumber(next_number+3));
        polygon_line.push_back(PointNumber(next_number-1));
        polygon_lines.push_back(polygon_line); polygon_line.Erase();

        /// Constructing backward face: 1 2 6 5
        polygon_line.push_back(PointNumber(next_number-3));
        polygon_line.push_back(PointNumber(next_number-2));
        polygon_line.push_back(PointNumber(next_number+2));
        polygon_line.push_back(PointNumber(next_number+1));
        polygon_lines.push_back(polygon_line); polygon_line.Erase();

        /// Constructing left face: 0 1 5 4
        polygon_line.push_back(PointNumber(next_number-4));
        polygon_line.push_back(PointNumber(next_number-3));
        polygon_line.push_back(PointNumber(next_number+1));
        polygon_line.push_back(PointNumber(next_number));
        polygon_lines.push_back(polygon_line); polygon_line.Erase();

        /// Constructing right face: 2 3 7 6
        polygon_line.push_back(PointNumber(next_number-2));
        polygon_line.push_back(PointNumber(next_number-1));
        polygon_line.push_back(PointNumber(next_number+3));
        polygon_line.push_back(PointNumber(next_number+2));
        polygon_lines.push_back(polygon_line); polygon_line.Erase();

/*        LineTopologies::iterator polygon_it;
        for(polygon_it = polygon_lines.begin(); polygon_it != polygon_lines.end(); polygon_it++){
            PointNumberList pnum_list = polygon_it->PointNumberListReference();
            for (int i=0; i<pnum_list.size(); i++){
                printf (" %d \n ", pnum_list[i]);
            }
        }*/

    /// sketch of the 6 faces, all the normals should point outside the cube,
    /// we use right-hand rule to order the vertices for each face, while the thumb always pointing out
        /*     1<---------2
              /|		 /|
            /  |  top	/ |
           0---------->3  |
           |   |	   |  |
           |   |	   |  |
           |   |	   |  |
           |   5-------|->6
           |  /		   | /
           | /	below  |/
           4<---------7            */

        /// debug
        //corners.Write("D:/test//visualization/off_conversion//vertices_intermediate.objpts");
        //polygon_lines.Write("D:/test//visualization/off_conversion//lines_topology_intermediate.top", false);
    }

    printf("Nmber of boxes: %d \n", box_cnt);

    //corners.Write("D:/test/visualization/firebrigade/wall_floor.objpts");//"D:/test//visualization/off_conversion//vertices.objpts");
    //polygon_lines.Write("D:/test/visualization/firebrigade/wall_floor.top", false);//"D:/test//visualization/off_conversion//lines_topology.top", false);

    /// convert polygon_lines and corners to OFF format:
    FILE *OFF;
    OFF = fopen(OFFfile, "w");
    if (!OFF){
        fprintf(stderr, "Error opening output file %s\n", OFFfile);
        exit(0);
    }

    fprintf(OFF, "OFF \n");

    /// #vertices, #faces, #edges
    fprintf(OFF, "%d %d %d \n", corners.size(), polygon_lines.size(), 0);

    /// write points or vertices
    ObjectPoints::const_iterator points_it;
    for(points_it = corners.begin(); points_it != corners.end(); points_it++){
        fprintf(OFF, "%.3f ", points_it -> X());
        fprintf(OFF, "%.3f ", points_it -> Y());
        fprintf(OFF, "%.3f",  points_it -> Z());
        fprintf(OFF, "\n");
    }

    /// write faces from polygon_lines
    LineTopologies::const_iterator face_it;
    for(face_it = polygon_lines.begin(); face_it !=polygon_lines.end(); face_it++){
        int num_of_points;
        num_of_points = (int) face_it ->size(); /// for box faces should be always 4
        fprintf(OFF, "%d ", num_of_points); /// print number of vertices
        /// print vertices of the face
        for(int m=0; m < num_of_points; m++){
            fprintf(OFF,"%d " ,(*face_it)[m].Number());
        }
        fprintf(OFF, "\n");
    }
    fclose(OFF);
}


/*  NOTE: This output is suitable to be converted to OFF format */
/*  This function makes a 3Dbox/or a polyhedron around a minimum3Drectangle. By 3D, we mean the rectangle has arbitrary orientation.
 * The input is the rectangle vertices and the output is the box faces (instead of edges) and vertices.
 * The box is generated for a given offset from the rectangle plane.
 * The numbering of vertices starts from the number of input vertices plus the number of vertices of the offset rectangle/polygon.
 * The number of faces is: 2+n. (n is number of vertices of the polygon or rectangle)
 * The function should be modified for for polyhedron.
 * The output is written in a way to be used for OFF format and 3D software. Which means instead of edges ,
 * faces are generated from vertices.
 * */
///
/// \param rectangle_vertices
/// \param offset_distance
/// \param threeDbox_vertices
/// \param threeDbox_faces
void Min3DRectangle_to_3DFaces (const ObjectPoints &rectangle_vertices, double offset_distance,
                              ObjectPoints &threeDbox_vertices, LineTopologies &threeDbox_faces){


    if(rectangle_vertices.empty ()) {
        printf ("Error: rectnagle vertices are empty!!! \n");
        exit (0);
    }

    /// first we offset the plane of the rectangle to both sides
    ObjectPoints left_corners, right_corners;
    LineTopology left_edges, right_edges; // are not used
    offset_polygon (rectangle_vertices, offset_distance, left_corners, right_corners, left_edges, right_edges);

    LineTopology face_edges;

    /// add left_polygon to the threeDbox
    for (int i=0; i < left_corners.size (); i++){
        threeDbox_vertices.push_back (left_corners[i]);
        face_edges.push_back (left_corners[i].Number ());
    }
    face_edges.push_back((threeDbox_vertices.front ()).Number ()); //close the polygon // clockwise

    /// add left face to the threeDbox_faces and erase it for the next face
    threeDbox_faces.push_back (face_edges);
    face_edges.Erase ();

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
        face_edges.push_back (pnumber);
    }

    /// add right face to the threeDbox_faces and erase it for next face
    threeDbox_faces.push_back (face_edges);
    face_edges.Erase ();

    /// Constructing front face: 0 4 7 3
    face_edges.push_back(PointNumber(next_number-4));
    face_edges.push_back(PointNumber(next_number-0));
    face_edges.push_back(PointNumber(next_number+3));
    face_edges.push_back(PointNumber(next_number-1));
    threeDbox_faces.push_back(face_edges); face_edges.Erase();

    /// Constructing backward face: 1 2 6 5
    face_edges.push_back(PointNumber(next_number-3));
    face_edges.push_back(PointNumber(next_number-2));
    face_edges.push_back(PointNumber(next_number+2));
    face_edges.push_back(PointNumber(next_number+1));
    threeDbox_faces.push_back(face_edges); face_edges.Erase();

    /// Constructing lower face: 0 1 5 4
    face_edges.push_back(PointNumber(next_number-4));
    face_edges.push_back(PointNumber(next_number-3));
    face_edges.push_back(PointNumber(next_number+1));
    face_edges.push_back(PointNumber(next_number));
    threeDbox_faces.push_back(face_edges); face_edges.Erase();

    /// Constructing upper face: 2 3 7 6
    face_edges.push_back(PointNumber(next_number-2));
    face_edges.push_back(PointNumber(next_number-1));
    face_edges.push_back(PointNumber(next_number+3));
    face_edges.push_back(PointNumber(next_number+2));
    threeDbox_faces.push_back(face_edges); face_edges.Erase();
}

/* This function converts the linetopologies and vertices of objects to vertices and faces that are readable by
 * meshlab software and cloudcompare. In meshlab it is possible to save the off to collada format (*.dae) to import it into sketchup.
 * The input of this function should be FACES or closed polygons or closed rectangles
 * each face or a rectangle is stored separately in the linetopologies file
 * More attributes can be stored in OFF format but should be added to this function
 * */
void LineTopologies_to_OFF (const ObjectPoints &vertices, const LineTopologies &faces, char *root, bool verbose){

    char str_root[500];
    strcpy (str_root, root);

    /// open an empty off_file
    char * OFFfile = strcat (str_root, "off_output.off");
    FILE *OFF;
    OFF = fopen(OFFfile, "w");
    if (!OFF){
        fprintf(stderr, "Error opening output file %s\n", OFFfile);
        exit(0);
    }
    fprintf(OFF, "OFF \n");

    /// #vertices, #faces, #edges
    fprintf(OFF, "%d %d %d \n", vertices.size(), faces.size(), 0);

    /// write points or vertices
    ObjectPoints::const_iterator points_it;
    for(points_it = vertices.begin(); points_it != vertices.end(); points_it++){
        fprintf(OFF, "%.3f ", points_it -> X());
        fprintf(OFF, "%.3f ", points_it -> Y());
        fprintf(OFF, "%.3f",  points_it -> Z());
        fprintf(OFF, "\n");
    }

    /// write faces from polygon_lines
    LineTopologies::const_iterator face_it;
    for(face_it = faces.begin(); face_it !=faces.end(); face_it++){
        int num_of_points;
        num_of_points = (int) face_it ->size(); /// for box faces should be always 4
        fprintf(OFF, "%d ", num_of_points); /// print number of vertices
        /// print vertices of the face
        for(int m=0; m < num_of_points; m++){
            fprintf(OFF,"%d " ,(*face_it)[m].Number());
        }
        fprintf(OFF, "\n");
    }
    fclose(OFF);
}


void LineTopologies_to_OFFBoxes (const ObjectPoints &vertices, const LineTopologies &faces, char *root, bool verbose){

    char str_root[500];
    strcpy (str_root, root);

    /// open an empty off_file
    char * OFFfile = strcat (str_root, "off_output.off");
    FILE *OFF;
    OFF = fopen(OFFfile, "w");
    if (!OFF){
        fprintf(stderr, "Error opening output file %s\n", OFFfile);
        exit(0);
    }
    fprintf(OFF, "OFF \n");

    /// #vertices, #faces, #edges
    fprintf(OFF, "%d %d %d \n", vertices.size(), faces.size(), 0);

    /// write points or vertices
    ObjectPoints::const_iterator points_it;
    for(points_it = vertices.begin(); points_it != vertices.end(); points_it++){
        fprintf(OFF, "%.3f ", points_it -> X());
        fprintf(OFF, "%.3f ", points_it -> Y());
        fprintf(OFF, "%.3f",  points_it -> Z());
        fprintf(OFF, "\n");
    }

    /// write faces from polygon_lines
    LineTopologies::const_iterator face_it;
    for(face_it = faces.begin(); face_it !=faces.end(); face_it++){
        int num_of_points;
        //num_of_points = (int) face_it ->size();
        num_of_points = 4;  /// for a box, faces should be always 4
        fprintf(OFF, "%d ", num_of_points); /// print number of vertices
        /// print vertices of the face
        for(int m=0; m < num_of_points; m++){
            fprintf(OFF,"%d " ,(*face_it)[m].Number());
        }
        fprintf(OFF, "\n");
    }
    fclose(OFF);
}