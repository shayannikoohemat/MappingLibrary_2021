//
// Created by NikoohematS on 24-3-2017.
//

#include <iostream>
#include <vector>
#include <Vector3D.h>
#include <Plane.h>
#include "LaserPoints.h"
#include "Face.h"
#include "visualization_tools.h"

//LaserPoints Project_points_to_Plane(LaserPoints lp, const Plane &plane);

/*
 * This code fit a plane to one segment and export the corners and edges of the plane
 * the segment can have an arbitrary orientation
 * */

/**** NOTE:   This code doesn't fit a minimum rectangle, it just fit a plane and finds corners for visualization  ****/
    /// @param max_dist Maximum distance between a point and the line for a
    ///point to be considered a nearby point
    /// this function is for one segment
void VisulizePlane3D (LaserPoints segment_lp, double max_dist, ObjectPoints &plane_corners,
                      LineTopology &plane_edges, bool verbose){

    //char* laserFile;
    //laserFile = (char*) "D://test//visualization//tiltedplane2.laser";
    //segment_lp.Read(laserFile);
    //max_dist = 1.5;


    /// project segment points to the fitted plane
    Plane plane;
        bool plane_is_horizontal;
        plane_is_horizontal = false;
    plane = segment_lp.FitPlane(segment_lp[0].SegmentNumber(), segment_lp[0].SegmentNumber(), SegmentNumberTag);
    LaserPoints projected_toplane;
    projected_toplane = Project_points_to_Plane(segment_lp, plane);
    projected_toplane.SetAttribute(SegmentNumberTag, segment_lp[0].SegmentNumber());
   if (verbose){
       //projected_toplane.Write("D://test//visualization//projected_points.laser", false);
       //projected_toplane.Write("E:/Laser_data/ETH_dataset/penthouse/out/projected_points.laser", false);
       //projected_toplane.Write("E:/publication_data/FB_dataset/process/out/occlusion_test/projected_points.laser", false);
   }

    /// make a pointnumberlist of projected points
    PointNumberList seg_projected_pointlist;
    seg_projected_pointlist = projected_toplane.SelectTagValueList(SegmentNumberTag, segment_lp[0].SegmentNumber());

    double zmin, zmax; /// zmin and zmax of the segment
    //projected_toplane.AttributeRange(ZCoordinateTag, zmin, zmax);

    /// get the data bounds of the projected points, to extract the height of the segment via zmin and zmax
    DataBoundsLaser db, db_projectedpoints;
    db = segment_lp.DeriveDataBounds(0); // for double check with zmin and zmax
    db_projectedpoints = projected_toplane.DeriveDataBounds(0);
    /// if the plane is horizontal the corners should be calculated differently than a slanted planted
    /// if the plane is horizontal we replace minZ and maxZ with MinX and MaxX, to get the left and right bounds
    if(plane.IsHorizontal (0.10 * 3.14159 / 180)) /// 10 degree is the tolerance of being flat
        plane_is_horizontal = true;

    if (plane_is_horizontal){
        zmin = db_projectedpoints.Minimum().GetX ();
        zmax = db_projectedpoints.Maximum ().GetX ();
    } else{
        zmin = db_projectedpoints.Minimum().GetZ();
        zmax = db_projectedpoints.Maximum().GetZ();
    }

    /// reconstruct two planes in zmin and zmax of the segment's plane
    Vector3D vec;
    if(plane_is_horizontal){ /// if segment's plane is horizontal we add two planes in left and right sides
        vec={1,0,0};
    }else {
        vec={0,0,1}; /// normal of the planes is parallel to z axis
    }

    /// initializing two planes in above and below the 3D plane
    /// if the segment's plane is horizontal this two planes would be in left and right
    Plane horizontal_plane_min, horizontal_plane_max;
    horizontal_plane_min.Initialise(), horizontal_plane_max.Initialise();
    horizontal_plane_min.SetDistance(zmin), horizontal_plane_min.SetNormal(vec), horizontal_plane_min.Number()=1;
    horizontal_plane_max.SetDistance(zmax), horizontal_plane_max.SetNormal(vec), horizontal_plane_max.Number()=2;

    /// intersect min and max plane with segment's plane and get two lines in zmin and zmax
    Line3D line_min;
    double pbegin_min, pend_min;
    pbegin_min = pend_min = 0.0;
    if (Intersect2Planes(plane, horizontal_plane_min, line_min)){
        projected_toplane.FaceNearLine(seg_projected_pointlist, line_min, max_dist, pbegin_min, pend_min); // crashes
    };
    Line3D line_max;
    double pbegin_max, pend_max;
    pbegin_max = pend_max = 0.0;
    if(Intersect2Planes(plane, horizontal_plane_max, line_max)){
        projected_toplane.FaceNearLine(seg_projected_pointlist, line_max, max_dist, pbegin_max, pend_max);
    };

    // Derive the begin and end points of the min and max lines
    Position3D point1_min, point2_min,
            point1_max, point2_max;
    point1_min = line_min.Position(pbegin_min);
    point2_min = line_min.Position(pend_min);

    point1_max = line_max.Position(pbegin_max);
    point2_max = line_max.Position(pend_max);

    /// vector of corners in position3D format
    vector <Position3D> corners_pos;
    /// The order is important for connecting points later in a polygon
    corners_pos.push_back(point1_min);
    corners_pos.push_back(point1_max);
    corners_pos.push_back(point2_max);
    corners_pos.push_back(point2_min);


    if (!plane_edges.empty()) plane_edges.erase(plane_edges.begin(), plane_edges.end());

    // make objectpoints of corners
    ObjectPoint corner;
    Covariance3D cov3d;
    cov3d = Covariance3D(0, 0, 0, 0, 0, 0);
    PointNumber pnumber;
    // Store points and topology
    int next_number;
    if (plane_corners.empty()){
        next_number = 0;
    } else {
        next_number = (plane_corners.end() - 1)->Number() + 1;
    }

    for (int i=0; i < corners_pos.size() ; i++){
        pnumber = PointNumber(next_number + i);
        corner = ObjectPoint(corners_pos[i], pnumber, cov3d);
        plane_corners.push_back(corner);
        plane_edges.push_back(pnumber);
    }
    plane_edges.push_back(PointNumber(next_number)); // Close polygon
    if (plane_edges.IsClockWise(plane_corners)){
        plane_edges.MakeCounterClockWise(plane_corners);
    }else plane_edges.MakeClockWise(plane_corners);

    plane_edges.SetAttribute(LabelTag, 3);

    LineTopologies planes_edges;
    planes_edges.push_back(plane_edges);

/*    if (verbose){
        plane_corners.Write("D://test//visualisation//plane_corners.objpts");
        planes_edges.Write("D://test//visualisation//plane_edges.top", false);
    }*/

}

/// not finished
void EnclosingRectangle_with_Rotation (const LaserPoints &laserPoints, double max_dist,
                                        ObjectPoints &rectangle_corners, LineTopology &rectangle_edges,
                                        char *root, bool verbose){
    char str_root[500];
    double pi = 4.0 * atan (1);
    /// fit a plane to the point clouds
    Plane plane;
    if (laserPoints.HasAttribute (SegmentNumberTag)){
        plane = laserPoints.FitPlane (laserPoints[0].SegmentNumber ());
    } else {
        plane = laserPoints.FitPlane (1);
    }

    Vector3D normal;
    normal = plane.Normal ();

    // debug: set normal to (1,1,1) at 45 degrees
    //normal = Vector3D(1,1,1);

    double alpha; /// the angle of the 3D vector with the y-axis (rotation about the z-axis)
    if (normal.X () != 0.0){
        alpha = atan2(normal.Y () , normal.X ());
        if (alpha > pi/2.0) alpha -= pi;
    } else {
        alpha = 0.0;  /// ??? plane is vertical and the normal is parallel to the y-axis
    }

    double teta; // the angle of the 3D vector with the xy-plane
    if (normal.X () !=0 && normal.Y () != 0.0 ){
        teta = atan (normal.Z () / sqrt(pow(normal.X (), 2.0) + pow(normal.Y (), 2.0)));
        if (teta > pi/2.0) teta -= pi;
    }else {
        teta = 0.0; // ??? plane is horizontal no need for rotation
    }

    double betha; //
    if (normal.Y () != 0.0){
        betha = atan(normal.X () / normal.Y ());
        if (betha > pi/2.0) betha -= pi;
    } else {
        betha = 0.0;  /// ??? plane is vertical and the normal is parallel to the y-axis
    }

    if (verbose){
        printf ("alpha angle: %.1f\n", alpha * 180 / pi);
        printf ("teta angle: %.1f\n", teta * 180 / pi);
    }
/*    Vector3D vx, vz;
    vx = Vector3D(1, 0, 0);
    vz = Vector3D(0, 0, 1);*/
    Rotation3D rotZ_3d;
    /// rotation about z-axis
    rotZ_3d =  Rotation3D (normal, alpha);
    /// rotation about x-axis
    Rotation3D rotX_3d; // rotation about x-axis
    rotX_3d =  Rotation3D (normal, teta);

    Rotation3D rotY_3d; // rotation about y-axis
    rotY_3d =  Rotation3D (normal, betha);

    // debug
    Vector3D vec;
    vec = rotX_3d * rotZ_3d * normal;
    if (verbose){
        vec.PrintVector ();
    }

    double dist;
    dist = normal.Length (); /// ???
    LaserPoints rotated_lp;
    for (auto &p : laserPoints){
        // translate the point back to the origin
        LaserPoint p_to_origin;
        p_to_origin = p - normal * dist;
        LaserPoint rotated_p;
        rotated_p = (rotX_3d) * (rotZ_3d) * Vector3D (p_to_origin.Position3DRef ());
        //rotated_p = (rotZ_3d) * Vector3D (p.Position3DRef ());
        //Vector3D pv = Vector3D(p.Position3DRef ());

        /// translate point back to the position
        LaserPoint new_p;
        new_p = rotated_p + normal * dist;

        rotated_lp.push_back (new_p);
    }

    strcpy (str_root, root);
    rotated_lp.Write(strcat (str_root, "rotated_points.laser"), false);

    // TODO: calculate the enclosing rectangle and rotate the vertices back to the original plane

}

/* This function is recommended for fitting a minimum rectangle in 3D such as a slanted segment */
/* This function rotates the point clouds to a horizontal plane and
 * fit a 2D-minimumrectangle. Then rotates everything to the 3D plane */
/// fit a minimum rectangle to a segment with an arbitrary normal vector
void EnclosingRectangle_Rotation3D (const LaserPoints &laserPoints, double max_dist,
                                       ObjectPoints &rectangle_corners, LineTopology &rectangle_edges,
                                       char *root, bool verbose){
    char str_root[500];
    double pi = 4.0 * atan (1);
    /// fit a plane to the point clouds
    Plane plane;
    if (laserPoints.HasAttribute (SegmentNumberTag)){
        plane = laserPoints.FitPlane (laserPoints[0].SegmentNumber ());
    } else {
        plane = laserPoints.FitPlane (1);
    }

    Vector3D normal;
    normal = plane.Normal ();

    /// make a horizontal plane
    Vector3D vecZ = Vector3D(0,0,1);
    LaserPoint pt = laserPoints[0];
    Plane horizontal_plane (pt, vecZ);

    Line3D horizontal_line3d;
    Intersect2Planes(horizontal_plane, plane, horizontal_line3d);
    double angle;
    angle = Angle(normal, vecZ); /// this is the rotation angle
    if (verbose){
        printf ("angle: %.1f\n", angle * 180 / pi);
    }

    Rotation3D rotation3D;
    rotation3D = Rotation3D(horizontal_line3d.Direction (), angle); /// rotation matrix

    /// rotate laserpoints to the xy-plane
    Plane rotated_plane;
    LaserPoints rotated_lp;
    for (auto &p: laserPoints){
        LaserPoint rotated_p;
        rotated_p = rotation3D * Vector3D(p.Position3DRef ());
        rotated_lp.push_back (rotated_p);
        rotated_plane.AddPoint (rotated_p);
    }
    //rotated_plane = rotated_lp.FitPlane (1); // this fails
    rotated_plane.Recalculate ();
    rotated_plane.Number () = 1;

    Position3D rotated_plane_center;
    rotated_plane_center = rotated_plane.CentreOfGravity ();

    /// Calculate the minimum enclosing rectangle for the rotated points
    /// get the databound of the segment
    DataBoundsLaser db;
    db = rotated_lp.DeriveDataBounds(0);

    /// Find the minimum enclosing rectangle for the segment in 2D
    rotated_lp.DeriveTIN();
    if (!rectangle_edges.empty()) rectangle_edges.erase(rectangle_edges.begin(), rectangle_edges.end());
    /// generated corners have Z value at 0
    ObjectPoints temp_corners;
    LineTopology plane_edge;
    rotated_lp.EnclosingRectangle(max_dist, temp_corners, plane_edge); // corners and edges are the output of the function

    /// rotate corners to the original plane with the transpose of rotation matrix
    for (auto & corner : temp_corners){ /// temp_corners are the corners of the rectangle in 2D
        /* important step */
        /// the Z-values of 2D-rectangle should be adjusted to the plane of the rotated_lp
        //corner.Z () = (db.Minimum ().GetZ () + db.Maximum ().GetZ ()) / 2.0; // not a good practice
        corner.Z () = rotated_plane_center.GetZ (); // this is more accurate becasue we get the z of the rotated plane

        Vector3D new_corner_v;
        new_corner_v = rotation3D.Transpose () * Vector3D(corner.Position3DRef ());
        //new_corner_v = (rotation3D.Transpose ()).Rotate (corner.Position3DRef ());
        //updating the x, y, z of the corner;
        corner.SetX (new_corner_v.X ());
        corner.SetY (new_corner_v.Y ());
        corner.SetZ (new_corner_v.Z ());

        rectangle_corners.push_back (corner);
        rectangle_edges.push_back (corner.Number ());
    }

    /// close the rectangle
    rectangle_edges.push_back (temp_corners[0].Number ());

    if(verbose){
        strcpy (str_root, root);
        rotated_lp.Write(strcat (str_root, "rotated_points.laser"), false);
    }
}

/* This function is recommended for fitting a minimum rectangle in 3D such as a slanted segment */
/* This function rotates the point clouds to a horizontal plane and
 * fit a 2D-minimumrectangle. Then rotates everything to a Given 3D-plane
 * Note: the difference with previous function EnclosingRectangle_Rotation3D is that the plane is given and
 * no fitting plane is required here */
/// fit a minimum rectangle to a segment with an arbitrary normal vector
void EnclosingRectangle_Rotation3D (const LaserPoints &laserPoints, Plane plane, double max_dist,
                                    ObjectPoints &rectangle_corners, LineTopology &rectangle_edges,
                                    char *root, bool verbose){
    char str_root[500];
    double pi = 4.0 * atan (1);

    Vector3D normal;
    normal = plane.Normal ();

    Position3D point_on_plane;
    point_on_plane = plane.Project(laserPoints[0].Position3DRef());

    /// make a horizontal plane
    Vector3D vecZ = Vector3D(0,0,1);
    LaserPoint pt = laserPoints[0];
    Plane horizontal_plane (pt, vecZ);

    Line3D horizontal_line3d;
    Intersect2Planes(horizontal_plane, plane, horizontal_line3d);
    double angle;
    angle = Angle(normal, vecZ); /// this is the rotation angle
    if (verbose){
        printf ("angle: %.1f\n", angle * 180 / pi);
    }

    Rotation3D rotation3D;
    rotation3D = Rotation3D(horizontal_line3d.Direction (), angle); /// rotation matrix

    /// rotate laserpoints to the xy-plane
    Plane rotated_plane;
    LaserPoints rotated_lp;
    for (auto &p: laserPoints){
        LaserPoint rotated_p;
        rotated_p = rotation3D * Vector3D(p.Position3DRef ());
        rotated_lp.push_back (rotated_p);
        rotated_plane.AddPoint (rotated_p);
    }
    //rotated_plane = rotated_lp.FitPlane (1); // this fails

    /*  Recalculating the rotated plane has a problem, because if we want to use the input plane as the reference plane
     * any recalculation of the plane, may be change the parameters of the original input plane */
    /* Alternatively, after recalculation we use a rotated point on the original plane
     *  , which was projected on the plane, instead of the rotated_plane.CentreOfGravity ()*/
    rotated_plane.Recalculate ();
    rotated_plane.Number () = 1;
    //Position3D rotated_plane_center;
    //rotated_plane_center = rotated_plane.CentreOfGravity ();
    /// this is the alternative solution for the Recalculated CentreOfGravity () to avoid a plane shift after recalculation
    Position3D rotated_point_on_plane;
    rotated_point_on_plane = rotation3D * Vector3D(point_on_plane);

    /// Calculate the minimum enclosing rectangle for the rotated points
    /// get the databound of the segment
    DataBoundsLaser db;
    db = rotated_lp.DeriveDataBounds(0);

    /// Find the minimum enclosing rectangle for the segment in 2D
    rotated_lp.DeriveTIN();

    // this line is to make sure when we use this function in a loop, then rectangle_edges becomes empty everytime
    if (!rectangle_edges.empty()) rectangle_edges.erase(rectangle_edges.begin(), rectangle_edges.end());
    /// generated corners have Z value at 0
    ObjectPoints temp_corners;
    LineTopology plane_edge;
    rotated_lp.EnclosingRectangle(max_dist, temp_corners, plane_edge); // corners and edges are the output of the function

    /// rotate corners to the original plane with the transpose of rotation matrix
    for (auto & corner : temp_corners){ /// temp_corners are the corners of the rectangle in 2D
        /* important step */
        /// the Z-values of 2D-rectangle should be adjusted to the plane of the rotated_lp
        //corner.Z () = (db.Minimum ().GetZ () + db.Maximum ().GetZ ()) / 2.0; // not a good practice
        //corner.Z () = rotated_plane_center.GetZ (); // this is more accurate becasue we get the z of the rotated plane
        corner.Z () = rotated_point_on_plane.GetZ (); // this is better than rotated_plane_center whic is derived from recalculated CenterOfGravity

        Vector3D new_corner_v;
        new_corner_v = rotation3D.Transpose () * Vector3D(corner.Position3DRef ());
        //new_corner_v = (rotation3D.Transpose ()).Rotate (corner.Position3DRef ());
        //updating the x, y, z of the corner;
        corner.SetX (new_corner_v.X ());
        corner.SetY (new_corner_v.Y ());
        corner.SetZ (new_corner_v.Z ());

        rectangle_corners.push_back (corner);
        rectangle_edges.push_back (corner.Number ());
    }

    /// close the rectangle
    rectangle_edges.push_back (temp_corners[0].Number ());

    if(verbose){
        strcpy (str_root, root);
        rotated_lp.Write(strcat (str_root, "rotated_points.laser"), false);
    }
}



/* This function is discourage to use because it projects the 2Dminimumrecatngle instead of rotation*/
    /// fitting a minimum rectangle to a segment with an arbitrary normal vector
    /// it accepts point clouds with several segments as input
void fit_3D_minimumRectangle (const LaserPoints &segmented_lpoints, double angle_threshold, double max_edge_dist,
                              int min_segment_size,
                              ObjectPoints &plane_corners,
                                LineTopologies &plane_edges){

        vector<int>             segment_numbers;
        double                  PI;
        PI                      = 3.14159;
        Covariance3D cov3d;
        cov3d = Covariance3D(0, 0, 0, 0, 0, 0);

        segment_numbers = segmented_lpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        printf ("Number of laser segments: %d\n", segment_numbers.size());

        LineTopology plane_edge;
        ObjectPoints temp_corners;
        for ( auto & segment_no : segment_numbers){
            /// selecting points by segment and saving in segment_lpoints
            LaserPoints segment;
            segment = segmented_lpoints.SelectTagValue(SegmentNumberTag, segment_no);
            if (segment.size () < min_segment_size) continue; /// skip small segments

/*                for(auto &p : segment){ //debug
                    p.Z () = p.Z () - 7.0;
                }*/
            /// fit a plane to the segment
            Plane plane;
            plane = segment.FitPlane (segment_no);

            Vector3D normal;
            normal = plane.Normal ();

            /// get the databound of the segment
            DataBoundsLaser db;
            db = segment.DeriveDataBounds(0);

            /// Find the minimum enclosing rectangle for the segment in 2D
            segment.DeriveTIN();
            if (!plane_edge.empty()) plane_edge.erase(plane_edge.begin(), plane_edge.end());
            /// generated corners have Z value at 0
            segment.EnclosingRectangle(max_edge_dist, temp_corners, plane_edge); // corners and edges are the output of the function

            /// find the intersection of the points in z direction with the plane
            for (auto &vertex : temp_corners){
                Position3D vertex_tmp;
                vertex_tmp = Position3D(vertex.X (), vertex.Y (), 1.0);
                Line3D vTovtemp_line;
                vTovtemp_line = Line3D(vertex.Position3DRef (), vertex_tmp);
                Position3D intersection_pos;
                ObjectPoint new_vertex;
                if (IntersectLine3DPlane (vTovtemp_line, plane, intersection_pos)){
                    new_vertex = ObjectPoint(intersection_pos, vertex.Number (), cov3d);
                    plane_corners.push_back (new_vertex);
                }
                //double dist_vertex_to_plane;
                //dist_vertex_to_plane = plane.Distance (vertex.Position3DRef ());
                //Vector3D v3d;
                //Rotation3D rot;
                //double rotation_angle;
                //rotation_angle = Angle (normal, Vector3D(0,0,1)) * 180.0 / PI;

                //new_vertex = vertex;
                //new_vertex.Z () = vertex.Z () - dist_vertex_to_plane * normal.Z ();
                //plane_corners.push_back (new_vertex);
            }
            segment.Write("E:/publication_data/modeling/data/test/out/segment.laser", false);
        }
        temp_corners.Write("E:/publication_data/modeling/data/test/out/corners_2D.objpts");
        plane_edges.push_back (plane_edge);

}

/// This function is not complete
/// adopted code from bool LaserPoints::EnclosingRectangle in LaserScan
/// we assume the laserpoints has just one segment
bool EnclosingRectangle3D (LaserPoints &laserpoints, double max_edge_dist, ObjectPoints &points,
                           LineTopology &polygon, char *root) {
    /// prepare the output file
    char str_root [500];

    Plane plane;
    if (laserpoints.HasAttribute (SegmentNumberTag)){
        plane = laserpoints.FitPlane (laserpoints[0].SegmentNumber ());
    } else {
        plane = laserpoints.FitPlane (1);
    }

    /// project points to the plane
    LaserPoints projected_points;
    projected_points = Project_points_to_Plane (laserpoints, plane);

/*    /// calcualte TIN and databounds
    laserpoints.RemoveDoublePoints (false); /// because in 3D maybe computationally expensive
    laserpoints.DeriveDataBounds (0);
    TIN *tin;
    tin = laserpoints.DeriveTIN (); /// for derive TIN the laserpoints should NOT have duplicate points

    // Check for presence of TIN and data bounds
    if (tin == nullptr) return false;
    if (!laserpoints.DataBounds().XYBoundsSet()) return false;

    /// create the TIN Edges
    TINEdges tinEdges;
    tinEdges.Derive (laserpoints.TINReference ());
    //laserpoints.RemoveLongEdges (tinEdges, max_edge_dist); /// not sure if is necessary

    /// Derive the convex hull
    PointNumberList pnrList;
    //for ( auto &p : laserpoints) pnrList.push_back (p.GetPointNumber ()); /// doesnt generate pnrlist correctly
    for (int i=0; i < laserpoints.size (); i++) pnrList.push_back (PointNumber(i));

    ObjectPoints lpointsObjpoints;
    lpointsObjpoints=laserpoints.ConstructObjectPoints(); /// make objectpoints out of laserpoints
    int contour_nr;
    contour_nr = laserpoints.HasAttribute (SegmentNumberTag) ? laserpoints[0].SegmentNumber () : 1;

    /// derive contour
    LineTopology lpointsContour;
    lpointsContour = laserpoints.DeriveContour (contour_nr, pnrList, tinEdges, true, SegmentNumberTag); /// this can be modified for different TAGs
    if (!lpointsContour.IsClosed())
        lpointsContour.push_back(*(lpointsContour.begin()));

    /// generate contour vertices
    ObjectPoints lpointsContourVertices;
    for(int j=0;j<lpointsContour.size();j++)
    {
        ObjectPoint objpt;
        objpt=lpointsObjpoints[lpointsContour[j].Number()];
        lpointsContourVertices.push_back(objpt);
    }

    /// write contours for debug
    LineTopologies contour_edges;
    contour_edges.push_back (lpointsContour);
    /// write to the file
    strcpy(str_root, root);
    lpointsContourVertices.Write(strcat(str_root, "contour_points.objpts"));
    strcpy(str_root, root);
    contour_edges.Write(strcat(str_root, "contour_edges.top"), false);*/

// Derive the convex hull
    /// contour 3D is the projection of contour vertices to the segment plane
    LineTopology contour3D_edges;
    ObjectPoints contour3D_points;
    laserpoints = projected_points; /// not a good practice, change it later
    laserpoints.DeriveContour3D (contour3D_points, contour3D_edges, max_edge_dist);
    if (!contour3D_edges.IsClosed())
        contour3D_edges.push_back(*(contour3D_edges.begin()));


    LineTopologies contour3Ds_edges;
    contour3Ds_edges.push_back (contour3D_edges);
    /// write to the file
    strcpy(str_root, root);
    contour3D_points.Write(strcat(str_root, "contour3D_points.objpts"));
    strcpy(str_root, root);
    contour3Ds_edges.Write(strcat(str_root, "contour3D_edges.top"), false);
// Determine minimum enclosing rectangle

    // Determine the convex hull edge belonging to the minimum enclosing rectangle
    double best_area = 1.0e30;
    Line3D bb_lines[4];
    Position3D corners[4];
    LineTopology::iterator node, node1, node2, prev_node, next_node, convex_node;
    node1 = contour3D_edges.begin ();
    LaserPoints::const_iterator laser_point_it, laser_point1_it, laser_point2_it;
    /// from SegmentOutlines.cc /LaserPoints::EnclosingRectangle
    laser_point1_it = laserpoints.begin() + node1->Number(); /// getting the laserpoints number of the nodes in the edges
    //double dist_min1, dist_max1, dist_min2, dist_max2;
    for (node1++; node1!=contour3D_edges.end(); node1++){
        // Construct a line for this convex hull edge
        laser_point2_it = laser_point1_it;
        laser_point1_it = laserpoints.begin() + node1->Number();
        Line3D line3d;
        line3d = Line3D (laser_point1_it -> Position3DRef (), laser_point2_it -> Position3DRef ());

        /// debug
        LineTopologies line3d_tops;
        Line3DSegment line3DSegment;
        Position3D beginPos;
        beginPos = laser_point1_it -> Position3DRef ();
        line3DSegment = Line3D_to_LineSegment (line3d, beginPos, 5.0);
        line3d_tops.push_back (line3DSegment.line_top);
        strcpy(str_root, root);
        line3DSegment.begin_end_point.Write (strcat(str_root, "3dline_points.objpts"));
        strcpy(str_root, root);
        line3d_tops.Write(strcat(str_root, "3dline_edge.top"), false);
        /// end of debug

// determine the sizes of the bounding rectangle
        double dist, dist_max1, dist_min1 ;
        dist = dist_max1 = dist_min1 = 0.0;
        for (node=contour3D_edges.begin(); node!=contour3D_edges.end()-1; node++){
           laser_point_it = laserpoints.begin () + node -> Number ();
           dist = fabs(line3d.DistanceToPoint (laser_point_it -> Position3DRef ()));
           if (node == contour3D_edges.begin ()){
               dist_min1 = dist_max1 = dist;
           } else {
               if (dist < dist_min1) dist_min1 = dist;
               if (dist > dist_max1) dist_max1 = dist;
           }
        }

        // /* Construct a plane through the point and perpendicular to the line */
        Plane perp_plane (laser_point_it ->Position3DRef (), line3d.Direction ().Normalize ()); /// ???
        /// calculate the intersection of two planes as the perpendicular 3D line
        Line3D perp_line3d;

        if (!Intersect2Planes (plane, perp_plane, perp_line3d)){
            EXIT_FAILURE;
        } ; /// ???

        /// debug: draw the perp_line

        LineTopologies line3d_tops2;
        Line3DSegment line3DSegment2;
        Position3D beginPos2;
        beginPos2 = laser_point_it -> Position3DRef ();
        line3DSegment2 = Line3D_to_LineSegment (perp_line3d, beginPos2, 5.0);
        line3d_tops2.push_back (line3DSegment2.line_top);
        strcpy(str_root, root);
        line3DSegment2.begin_end_point.Write (strcat(str_root, "3dline_points2.objpts"));
        strcpy(str_root, root);
        line3d_tops2.Write(strcat(str_root, "3dline_edge2.top"), false);
        ///

        double dist_max2, dist_min2 ;
        dist_max2 = dist_min2 = 0.0;
        for (node=contour3D_edges.begin(); node!=contour3D_edges.end()-1; node++){
            laser_point_it = laserpoints.begin () + node -> Number ();
            dist = fabs(perp_line3d.DistanceToPoint (laser_point_it -> Position3DRef ()));
            if (dist < dist_min2) dist_min2 = dist;
            if (dist > dist_max2) dist_max2 = dist;
        }

        // Derive the bounding lines if this is the best area so far
        double area;
        area = (dist_max1 - dist_min1) * (dist_max2 - dist_min2);
    // calculate the new position of the rectanlge edges
        Position3D pos1_min, pos1_max, pos2_min, pos2_max;
        pos1_min = Position3D (line3d.CentreOfGravity ().X () + dist_min1,
                               line3d.CentreOfGravity ().Y () + dist_min1,
                               line3d.CentreOfGravity ().Z () + dist_min1);
        pos1_max = Position3D (line3d.CentreOfGravity ().X () + dist_max1,
                               line3d.CentreOfGravity ().Y () + dist_max1,
                               line3d.CentreOfGravity ().Z () + dist_max1);
        pos2_min = Position3D (perp_line3d.CentreOfGravity ().X () + dist_min2,
                               perp_line3d.CentreOfGravity ().Y () + dist_min2,
                               perp_line3d.CentreOfGravity ().Z () + dist_min2);
        pos2_max = Position3D (perp_line3d.CentreOfGravity ().X () + dist_max2,
                               perp_line3d.CentreOfGravity ().Y () + dist_max2,
                               perp_line3d.CentreOfGravity ().Z () + dist_max2);

        Line2D l;
        if (area < best_area) {
            best_area = area;
            bb_lines[0] = Line3D(pos1_min, line3d.Direction ());
            bb_lines[1] = Line3D(pos1_max, line3d.Direction ());
            bb_lines[2] = Line3D(pos2_min, perp_line3d.Direction ());
            bb_lines[3] = Line3D(pos2_max, perp_line3d.Direction ());
        }
    }
    // Determine the corners of the bounding box
    Intersection2Lines (bb_lines[0], bb_lines[2], corners[0]);
    Intersection2Lines (bb_lines[0], bb_lines[3], corners[1]);
    Intersection2Lines (bb_lines[1], bb_lines[3], corners[2]);
    Intersection2Lines (bb_lines[1], bb_lines[2], corners[3]);

    // Store points and topology
    int next_number;
    if (points.empty()) next_number = 0;
    else next_number = (points.end() - 1)->Number() + 1;
    if (!polygon.empty()) polygon.erase(polygon.begin(), polygon.end());
    for (int i=0; i<4; i++) {
        ObjectPoint point;
        point.X() = corners[i].X();
        point.Y() = corners[i].Y();
        point.Number() = next_number + i;
        points.push_back(point);
        polygon.push_back(PointNumber(next_number + i));
    }
    polygon.push_back(PointNumber(next_number)); // Close polygon
    polygon.MakeCounterClockWise(points);

    return true;
}



Line3DSegment Line3D_to_LineSegment (Line3D &line3d, Position3D &pos, double t){

    Line3DSegment line3DSegment;
    PointNumber pn1, pn2;
    Covariance3D cov3d;
    cov3d = Covariance3D(0, 0, 0, 0, 0, 0);

    /// convert begin and end-point of line to objpoint
    pn1 = PointNumber(1);
    line3DSegment.begin_end_point.push_back (ObjectPoint(pos, pn1, cov3d)) ;

    pn2 = PointNumber(2);
    Position3D pos2;
    pos2 = pos + line3d.Direction ().Normalize () * t; /// ???
    line3DSegment.begin_end_point.push_back(ObjectPoint(pos2, pn2, cov3d));
    /// create the line_topology
    line3DSegment.line_top = LineTopology(1, 1, pn1, pn2);
    return line3DSegment;
}


/// this function always doesn't generate correct result for 3D oriented planes, slanted segments
/// borrowed from LaserPoints::EnclosingPolygon ()
/// segmented_lp is preferred for better result
void EnclosingPolygon_test (LaserPoints lp, char *root){

    char str_root [500];

    lp.RemoveDoublePoints (); /// very important to generate TIN
    lp.DeriveDataBounds (0);
    lp.DeriveTIN ();

    ObjectPoints vertices;
    LineTopologies polygon;
    LineTopology edges;

    /// if the lp has segment_number this segmentation parameters are not used
    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;  /// initialization
    // seg_parameter -> MaxDistanceInComponent()  = 0.3;

    OutliningParameters outliningParameters;
    // outliningParameters.MaximumDistancePointIntersectionLine () = 0.20;
    // outliningParameters.MinimumAnglePreferredDirections ()      = 20.0;
    lp.EnclosingPolygon (vertices, edges, *seg_parameter, outliningParameters, false);

    polygon.push_back (edges);

    strcpy(str_root, root);
    vertices.Write(strcat(str_root, "polygon_vertices.objpts"));
    strcpy(str_root, root);
    polygon.Write(strcat(str_root, "polygon_edges.top"), false);
}


/*
void LaserPoints::DeriveContour3D(ObjectPoints &contour_obj, LineTopology &contour_top, double max_edge_dist) const
{

    ObjectPoint objpt;
    ObjectPoints objpts;
    PointNumberList        component;
    TINEdges edges;
    LineTopology           laser_contour;
    Vector3D point, direction;
    Vector3D vec1=Vector3D(0,0,1);
    Line3D horizontal_line;
    Position3D pt;
    Plane myplane;
    double angle;
    Rotation3D *rot1;
    LaserPoints rotated, input_laser;

    input_laser=(*this);

    pt=input_laser[0];

    input_laser.Label(1);
    myplane=input_laser.FitPlane(1,1,LabelTag);

    for(int j=0;j<input_laser.size();j++)
    {
        point=myplane.Project(input_laser[j]);
        input_laser[j].SetX(point.X());
        input_laser[j].SetY(point.Y());
        input_laser[j].SetZ(point.Z());
    }

    objpts=input_laser.ConstructObjectPoints();

    Plane horizontal_plane(pt,vec1);

    Intersect2Planes(horizontal_plane, myplane, horizontal_line);
    angle=Angle(myplane.Normal(), vec1);

    rot1=new Rotation3D(horizontal_line.Direction(), angle);

    rotated=input_laser;

    for(int j=0;j<rotated.size();j++)
    {
        point=(rot1->Transpose ()).Rotate(rotated[j]);
        rotated[j].SetX(point.X());
        rotated[j].SetY(point.Y());
        rotated[j].SetZ(point.Z());
    }

    rotated.DeriveTIN();
    rotated.DeriveDataBounds(0);
    edges.Derive(rotated.TINReference());
    rotated.RemoveLongEdges(edges, max_edge_dist);

    component.clear();

    for (int j=0; j<rotated.size(); j++)
        component.push_back(PointNumber(j));

    contour_top = rotated.DeriveContour(1, component, edges, false);

    contour_obj.clear();
    for(int j=0;j<contour_top.size();j++)
    {
        objpt=objpts[contour_top[j].Number()];
        contour_obj.push_back(objpt);
    }

    /// added by shayan to close the contour
      if (!laser_contour.IsClosed())
    laser_contour.push_back(*(laser_contour.begin()));


}*/
