//
// Created by NikoohematS on 22-10-2018.
//

/*
 *  offset vertices of a polygon about the a plane and return new vertices
 * */

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <Plane.h>
#include "../include/visualization_tools.h"
/// offset the vertices of a rectangle or a polygon, along the normal vector, on both sides with a given offset dist
/// the result is Obj points and line topology on the both sides
void offset_polygon(const ObjectPoints &vertices, double offset_dist,
                     ObjectPoints &left_corners, ObjectPoints &right_corners,
                        LineTopology &left_polygon, LineTopology &right_polygon){

    /// reconstruct a plane with 4 vertices
    Plane plane;
    plane = Plane(vertices[0].Position3DRef (), vertices[1].Position3DRef (),vertices[2].Position3DRef ());
    plane.AddPoint (vertices[3].Position3DRef (), true); // true means recalculate the plane

    Vector3D normal;
    normal = plane.Normal ();

    int first_vertice_number;
    first_vertice_number = vertices[0].Number (); /// for closing the polygon
    for (auto &v : vertices){
        //int next_number;
        //next_number;
        /// this is the offset in the direction of the normal
        ObjectPoint vLeft;
        vLeft.X () = v.X () - offset_dist * normal.X ();
        vLeft.Y () = v.Y () - offset_dist * normal.Y ();
        vLeft.Z () = v.Z () - offset_dist * normal.Z ();
        vLeft.Number () = v.Number ();
        left_corners.push_back (vLeft);
        left_polygon.push_back (v.Number ());

        /// this is the offset in the opposite direction of the normal
        ObjectPoint vRight;
        vRight.X () = v.X () + offset_dist * normal.X ();
        vRight.Y () = v.Y () + offset_dist * normal.Y ();
        vRight.Z () = v.Z () + offset_dist * normal.Z ();
        vRight.Number () = v.Number ();
        right_corners.push_back (vRight);
        right_polygon.push_back (v.Number ());
    }

    /// close the polygon
    left_polygon.push_back (first_vertice_number);
    right_polygon.push_back (first_vertice_number);

    //left_polygons.push_back (left_polygon);
    //right_polygons.push_back (right_polygon);
}

/// offset the vertices of a rectangle or a polygon, along the normal vector on both sides,
/// using a given offset dist and a given plane
/// the result is Obj points and line topologies on the both sides
void offset_polygon(const ObjectPoints &vertices, const Plane &plane, double offset_dist,
                    ObjectPoints &left_corners, ObjectPoints &right_corners,
                    LineTopology &left_polygon, LineTopology &right_polygon){

    Vector3D normal;
    normal = plane.Normal ();

    int first_vertice_number;
    first_vertice_number = vertices[0].Number (); /// for closing the polygon
    for (auto &v : vertices){
        //int next_number;
        //next_number;
        /// this is the offset in the direction of the normal
        ObjectPoint vLeft;
        vLeft.X () = v.X () - offset_dist * normal.X ();
        vLeft.Y () = v.Y () - offset_dist * normal.Y ();
        vLeft.Z () = v.Z () - offset_dist * normal.Z ();
        vLeft.Number () = v.Number ();
        left_corners.push_back (vLeft);
        left_polygon.push_back (v.Number ());

        /// this is the offset in the opposite direction of the normal
        ObjectPoint vRight;
        vRight.X () = v.X () + offset_dist * normal.X ();
        vRight.Y () = v.Y () + offset_dist * normal.Y ();
        vRight.Z () = v.Z () + offset_dist * normal.Z ();
        vRight.Number () = v.Number ();
        right_corners.push_back (vRight);
        right_polygon.push_back (v.Number ());
    }

    /// close the polygon
    left_polygon.push_back (first_vertice_number);
    right_polygon.push_back (first_vertice_number);

    //left_polygons.push_back (left_polygon);
    //right_polygons.push_back (right_polygon);
}

Plane calculate_plane_between_two_parallel_planes (const Plane &plane1, const Plane &plane2){

    //double pi = 4.0 * atan(1.0);
/*    /// calculate the planes distances
    double planes_dist;
    planes_dist = fabs(plane1.Distance (plane2.CentreOfGravity ()));*/

    /*    /// calculate the average angle
    double angle;
    angle = Angle(plane1.Normal(), plane2.Normal());  /// in radian
    if (angle > pi/2.0) angle -= pi;*/

    /// calcualte a point between two planes
    Position3D point;
    point = (plane1.CentreOfGravity () + plane2.CentreOfGravity ()) / 2;

    /// calculate a normal vector between two normals
    Vector3D normalVec;
    if((plane1.Normal ().DotProduct (plane2.Normal ())) < 0.0){
        normalVec = (plane1.Normal ().Normalize () - plane2.Normal ().Normalize ()) ;
    } else {
        normalVec = (plane1.Normal ().Normalize () + plane2.Normal ().Normalize ()) ;
    }

    //normal=normalVec/sqrt(len);
    //distance = normal.DotProduct( point );

    /// initialize the plane with the new normal and the point as a vector3D
    Plane plane (point, normalVec);
    return plane;
}

/// if use_wieght is true then size of segments are used as weight for normal calculation
Plane calculate_plane_between_two_parallel_segments (LaserPoints &segment1, LaserPoints &segment2, bool use_weight){
    Plane plane1, plane2, plane_middle;
    plane1 = segment1.FitPlane (segment1[0].SegmentNumber ());
    plane2 = segment2.FitPlane (segment2[0].SegmentNumber ());

    Vector3D new_normal;
    if (use_weight){
        new_normal = ((segment1.size () * plane1.Normal () + segment2.size () * plane2.Normal ()) /
                      (segment1.size () + segment2.size ())).Normalize ();
    } else {
        new_normal = (plane1.Normal () + plane2.Normal ()).Normalize ();
    }

    /// calculate a point between two planes
    Position3D point;
    point = (plane1.CentreOfGravity () + plane2.CentreOfGravity ()) / 2;

    Plane updated_plane (point, new_normal);

    return updated_plane;
}

LaserPoints test_planes_calculation (LaserPoints &segment1, LaserPoints &segment2){

    Plane plane1, plane2, plane_middle;
    plane1 = segment1.FitPlane (segment1[0].SegmentNumber ());
    plane2 = segment2.FitPlane (segment2[0].SegmentNumber ());

    /// calculate the middle plane
    plane_middle = calculate_plane_between_two_parallel_planes (plane1, plane2);
    plane_middle.Number () = segment1[0].SegmentNumber ();

    /// extract the min rectangle to visualize it
    LaserPoints segment1_projected, segment2_projected, projected_points;
    segment1_projected = Project_points_to_Plane (segment1, plane_middle);
    segment2_projected = Project_points_to_Plane (segment2, plane_middle);

    (Project_points_to_Plane (segment1, plane1)).Write("E:/publication_data/modeling/data/test/out/segment1_projected.laser", false);
    (Project_points_to_Plane (segment2, plane2)).Write("E:/publication_data/modeling/data/test/out/segment2_projected.laser", false);
    //segment1_projected = segment1.ProjectToPlane (plane_middle.Normal ()); // it crashes
    //segment2_projected = segment2.ProjectToPlane (plane_middle.Normal ()); // it crashes

    projected_points.AddPoints (segment1_projected);
    projected_points.AddPoints (segment2_projected);

    return projected_points;
}


Plane offset_plane (const Plane &plane, double offset_distance){

    Position3D offset_pos;
    offset_pos = plane.CentreOfGravity () + offset_distance * plane.Normal ();
    Plane offset_plane (offset_pos, plane.Normal ());

    return offset_plane;
}

/// This function intersect two segments based on their given planes and add the points near each segment and on the
/// intersection line to each segment. Later it is possible to calculate the min rectangle of the segment.
/// This function doesn't calculate new planes and doesn't modify given planes.
/// if "bool extension=false" means two planes intersect but they are not close enough to be extended.
/// *** NOTE: after applying this function segments shouldn't be modified in another step
bool Extend_Segments_to_Intersection (LaserPoints &segment1, LaserPoints &segment2,
                                      const Plane &plane1, const Plane &plane2, double max_intersection_dist,
                                      LaserPoints &extended_segment1, LaserPoints &extended_segment2,
                                      bool &extension) {
    LaserPoints lp; // this is for the sake of FaceNearLine function
    lp.AddPoints (segment1);
    lp.AddPoints (segment2);
    /// collect the point_list of each segment
    PointNumberList pointNumberList1, pointNumberList2;
    pointNumberList1 = lp.TaggedPointNumberList (SegmentNumberTag, segment1[0].SegmentNumber ());
    pointNumberList2 = lp.TaggedPointNumberList (SegmentNumberTag, segment2[0].SegmentNumber ());

    /// if there is no intersection, extended_segments remain the same as input
    extended_segment1 = segment1;
    extended_segment2 = segment2;

    /// check if two planes of segments intersect
    Line3D intersection_line;
    extension = false;
    if (Intersect2Planes(plane1, plane2, intersection_line)){
        /// get the segmentline near both segments
        double begin1, end1; /// begin and end of points on line segment near each segment
        bool facenearline1 = false;
        if(lp.FaceNearLine (pointNumberList1, intersection_line, max_intersection_dist, begin1, end1)){
            Position3D  intersection_begin1, intersection_end1;
            intersection_begin1 = intersection_line.Position(begin1);
            intersection_end1 = intersection_line.Position (end1);
            /// add it to the segment1
            extended_segment1.push_back (intersection_begin1);
            extended_segment1.push_back (intersection_end1);
            extended_segment1.SetAttribute (SegmentNumberTag, segment1[0].SegmentNumber ());
            facenearline1 = true;
        }
        /// repeat the same for the second segment
        double begin2, end2; /// begin and end of points on line segment near each segment
        bool facenearline2 = false;
        if(lp.FaceNearLine (pointNumberList2, intersection_line, max_intersection_dist, begin2, end2)){
            Position3D  intersection_begin2, intersection_end2;
            intersection_begin2 = intersection_line.Position(begin2);
            intersection_end2 = intersection_line.Position (end2);
            /// add it to the segment1
            extended_segment2.push_back (intersection_begin2);
            extended_segment2.push_back (intersection_end2);
            extended_segment2.SetAttribute (SegmentNumberTag, segment2[0].SegmentNumber ());
            facenearline2 = true;
        }
        if (facenearline1 && facenearline2) extension = true;
        return true;
    } else {
        return false;
    }
}




