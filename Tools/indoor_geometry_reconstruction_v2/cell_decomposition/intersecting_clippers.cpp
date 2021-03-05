#include "intersecting_clippers.h"
#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
//#include <syslimits.h>
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"


/*
 * 1. import room point clouds with(out) furniture inside
 * 2. create planes and their min-rectnagle
 * 3. intersect planes to the bounding box of the point cloud
 *  3.1 intersect pair of planes to eachother
 * 4. store faces/polygons in the 3D space as a result of intersections
 * 5. label which cells belong to the same room and which belong to the outside (not done)
 *
 * */
/// NOT FINISHED
void intersect_planes(char *laserfile, char *root_dir, int min_segment_size)
{
    char str_root[500];
    strcpy (str_root, root_dir);

    LaserPoints lp;
    lp.Read(laserfile);

    vector<LaserPoints> object_seg_vec; // object can be walls, fl, cl or furniture or noise
    object_seg_vec = PartitionLpByTag(lp, SegmentNumberTag, root_dir);

    std::map<int, Plane> objects_plane_map;
    std::map<int, LaserPoints> objects_segments_map;

    // iterate through segments and create: 1. plane 2. min rectangle of the plane
    for (auto &s : object_seg_vec){
        if(s.HasAttribute(SegmentNumberTag) && s.size() > min_segment_size){
            int seg_num = s[0].SegmentNumber();
            objects_segments_map.insert (std::pair<int, LaserPoints> (seg_num, s)); // make a map of segments
            Plane plane;
            plane = s.FitPlane(seg_num);
            plane.Number() = seg_num;
            objects_plane_map.insert(std::pair<int, Plane>(seg_num, plane)); // make a map of planes
        }
    }
    // create a cuboid with 6 faces around the input data as the bound planes
    ObjectPoints lp_global_vertices; // output
    LineTopologies lp_global_faces; // output
    Planes cube_planes; // output
    std::map<int, Positions3D> planes_pos_map; // output
    cube_planes = bounding_cube(root_dir, lp, lp_global_vertices, lp_global_faces, planes_pos_map);

    // clip segments' planes with cube planes
    // we can loop in both segments or planes depends but the clipper code fits a plane itself to the given segment
/*    for (auto plane_it=objects_plane_map.begin(); plane_it != objects_plane_map.end(); plane_it++){
        int plane_num, seg_num = plane_it->first;
        Plane seg_plane = plane_it->second;
        // fetch the segment points for the plane
        LaserPoints segment_lp;
        auto segment_it = objects_segments_map.find(seg_num);
        if(segment_it !=objects_segments_map.end()){
            segment_lp = segment_it->second;
        }
    }*/


}



// collect list of pair segments which intersect in the bound of the data
/// a sweeping plane is more optimised to check the pair intersections but we do O(n2)
std::multimap<int, Plane> collect_intersectingPlanes(Planes planes,
                         ObjectPoints polygons_v, LineTopologies polygons_e)
 {
      vector< pair <int,int>> pair_polygons;
      std::multimap<int, Plane> polygon_splitters_mmap;
     /// make a map of planes
     /// the assumption is that plane_number and segmentNumber and polygon_number are the same
//     std::map<int, Plane> planes_map;
//     if(!planes.empty ()){
//         for(auto &plane : planes) {
//             planes_map.insert (std::pair<int, Plane> (plane.Number(), plane));
//         }
//     }

     polygons_e.Sort(); // sort by line number
     for (auto &polygon : polygons_e){
         ObjectPoints vertices = GetCorresponding_vertices(polygons_v, polygon);
         for (auto &plane : planes){
             LineSegment3D linesegment;
             if(Intersect_Plane_3DRectnagle(polygon, vertices, plane, linesegment)){
                 //pair_polygons.push_back(std::make_pair(polygon_num, plane_num));
                 polygon_splitters_mmap.insert(std::pair<int, Plane> (polygon.Number(), plane));
            }
         }
     }
//     for (auto it1 = polygons_e.begin (); it1 != polygons_e.end (); it1++){
//         // get the polygon here
//         LineTopology polygon = *it1;
//         int polygon_num = polygon.Number();
//         ObjectPoints vertices = GetCorresponding_vertices(polygons_v, polygon);
//         // get the second polygon and its plane
//         auto it2 = std::next (it1);
//         for(; it2 != polygons_e.end (); it2++){
//             int plane_num = (*it2).Number();
//             Plane plane;
//             auto plane_it = planes_map.find (plane_num);
//             if (plane_it != planes_map.end ()){
//                 plane = plane_it->second;
//             }
//             /// now we have both the polygon and a plane to intersect
//             //cout << "intersecting polygons: " << polygon_num << " and " << plane_num << endl; //DEBUG
//             LineSegment3D linesegment;
//             if(Intersect_Plane_3DRectnagle(polygon, vertices, plane, linesegment)){
//                 pair_polygons.push_back(std::make_pair(polygon_num, plane_num));
//             }
//         } // end of 2nd for
//     } // end of 1st for

     // print list of pairs for double-check
//     for (auto &p : pair_polygons){                                     //DEBUG
//         cout << "pairs:" << p.first << "<>" << p.second << endl;
//     }
     for (auto &m :polygon_splitters_mmap){

     }
     return polygon_splitters_mmap;
 }

void pairwise_split(Planes planes, ObjectPoints polygons_v, LineTopologies polygons_e,
                    double snap_dist, ObjectPoints new_polygons_v, LineTopologies new_polygons_e)
{

    polygons_e.Sort(); // sort by line number
    ObjectPoints updated_vertices;
    for (auto &polygon : polygons_e){
        cout << "for Polygon: " << polygon.Number() << endl; //DEBUG
        Planes splitPlanes;
        //LineSegments3D splitLinesegments;
        ObjectPoints vertices = GetCorresponding_vertices(polygons_v, polygon);
        for (auto &plane : planes){
            LineSegment3D linesegment;
            if(Intersect_Plane_3DRectnagle(polygon, vertices, plane, linesegment)){
                splitPlanes.push_back(plane);
                //splitLinesegments.push_back(linesegment);
            }
        }
        if(splitPlanes.empty())
            continue;
        LineTopologies polygons_to_be_cut;
        ObjectPoints polygons_to_be_cut_vertices;
        polygons_to_be_cut.push_back(polygon);
        polygons_to_be_cut_vertices = vertices;
        Planes::iterator plane_it = splitPlanes.begin();
        int cnt=0;
        while (plane_it != splitPlanes.end()){
            LineTopologies new_polygons; //children of polygon
            LineTopologies remained_polygons; // not intersected children
            Plane splitter = *plane_it;
            cout << "   Plane: " << splitter.Number() << endl; //DEBUG
            // split polygons inside polygons_to_be_cut vector
            for(auto &current_polygon : polygons_to_be_cut){
                cout << "       is cutting:" << polygon.Number() << "-" << cnt << endl; //DEBUG
                LineSegment3D linesegment3d;
               ObjectPoints current_vertices = GetCorresponding_vertices(polygons_to_be_cut_vertices, current_polygon);
                if(!Intersect_Plane_3DRectnagle(current_polygon, current_vertices, splitter, linesegment3d))
                    continue;
                LineTopologies tmp;
                if(SplitPolygon3DByLineSegment3D(current_vertices, current_polygon, linesegment3d, snap_dist, tmp)){
                    new_polygons.insert(new_polygons.end(), tmp.begin(), tmp.end());
                    polygons_to_be_cut_vertices = current_vertices;
                }else{
                    remained_polygons.push_back(current_polygon);
                    polygons_to_be_cut_vertices = current_vertices;
                }
                cnt++; // DEBUG
            }
            polygons_to_be_cut = new_polygons; // update polygons_to_be_cut
            polygons_to_be_cut.insert(polygons_to_be_cut.end(), remained_polygons.begin(), remained_polygons.end());
            plane_it++;
        } // end of while
    } // end of polygons
}

void test_pairwise_split(LaserPoints segments, LineTopologies box3d_faces,
                         ObjectPoints box3d_vertices, double snap_dist){
    ObjectPoints vertices; LineTopologies polygons;
    // intersect the segments with the bbox and get the polygons
    Intersect_Planes_3DBoxFaces(segments, 5, box3d_faces, box3d_vertices, polygons, vertices, true, true);
    vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices.objpts");
    polygons.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges.top", false);

    Planes planes;
    vector<LaserPoints> segments_vec;
    char *root_dir;
    segments_vec = PartitionLpByTag(segments, SegmentNumberTag, root_dir);
    for (auto &s : segments_vec){
        Plane plane;
        plane = s.FitPlane(s[0].SegmentNumber());
        plane.Number() = s[0].SegmentNumber();
        planes.push_back(plane);
    }
    ObjectPoints new_polygons_v; LineTopologies new_polygons_e;
    pairwise_split(planes, vertices, polygons, snap_dist, new_polygons_v, new_polygons_e);

}


/// NOT FINISHED
 /// polygon=face, plane=spliter
void pairwise_split_old(Planes planes, ObjectPoints polygons_v, LineTopologies polygons_e,
                   map<int, int> pair_polygons, double snap_dist,
                   ObjectPoints new_polygons_v, LineTopologies new_polygons_e)
 {
     /// make a map of planes
     /// the assumption is that plane_number and segmentNumber and polygon_number are the same
     std::map<int, Plane> planes_map;
     if(!planes.empty ()){
         for(auto &plane : planes) {
             planes_map.insert (std::pair<int, Plane> (plane.Number(), plane));
         }
     }

     /// make a map of polygons&vertices and numbers are keys.
     /// These keys are the same in segment numbers and plane numbers
     // later we update these multimaps with each split
     std::multimap<int, LineTopology>  polygons_map;
     std::multimap<int, ObjectPoints>  vertices_map;
     polygons_e.Sort(); // sort by line number
     for (auto & polygon : polygons_e){
         int polygon_num = polygon.Number();
         ObjectPoints vertices = GetCorresponding_vertices(polygons_v, polygon);
         polygons_map.insert(std::pair<int, LineTopology> (polygon_num, polygon));
         vertices_map.insert(std::pair<int, ObjectPoints> (polygon_num, vertices));
     }

     // this pair is just the numebrs (key) to the polygons which intersect
     // for this operation it is important that pair polygon is sort/ordered by first values (this is taken care of when it was created)
     for(auto &pair : pair_polygons){
         // split the polygon (pair.first) with the plane (pair.second)
         // keep a list of splits and update pair.first in the multimap
         LineTopology polygon;
         ObjectPoints vertices;
         int polygon_num  = pair.first;
         auto polygon_it  = polygons_map.find(polygon_num);
         auto vertices_it = vertices_map.find(polygon_num);
         if(polygon_it  != polygons_map.end()) polygon  = polygon_it->second;
         if(vertices_it != vertices_map.end()) vertices = vertices_it->second;

         typedef std::multimap<int, LineTopology>::iterator MmapIt_e;
         typedef std::multimap<int, ObjectPoints>::iterator MmapIt_v;
         std::pair <MmapIt_e, MmapIt_e> poly_range; // the range iterator to split polygons with the same key or parent polygon
         poly_range = polygons_map.equal_range(polygon_num);
         vector<LineTopology> child_polys;
         vector<ObjectPoints> child_vertices;

         // we take the plane from the second value of the pair
         Plane plane;
         int plane_num = pair.second;
         auto plane_it = planes_map.find (plane_num);
         if (plane_it != planes_map.end ()){
             plane = plane_it->second;
         }
         // now we need the intersection segment of polygon/face and the pane
         LineSegment3D linesegment;
         LineTopologies new_polygons; // this is one or two (one if the polygon is cut in its edge it returns the same polygon)
         ObjectPoints new_vertices;
         LineTopologies remained_polygons; // not intersected
         if(!Intersect_Plane_3DRectnagle(polygon, vertices, plane, linesegment))
             continue; // this can't happen because pairs should already have intersection
         // split the polygon
         if(SplitPolygon3DByLineSegment3D(vertices, polygon, linesegment, snap_dist, new_polygons)){
         // update pair.first polygon in the map
             for (auto &child_poly : new_polygons){
                 child_polys.push_back(child_poly);
                 ObjectPoints child_v=GetCorresponding_vertices(vertices, child_poly);
                 child_vertices.push_back(child_v);
             }
         } // if split happens
     } // for of pairs

 }


/// NOT FINISHED
/// 1.intersect two polygons/planes in 3D space
/// 2. get the intersection linesegment: Intersect_Plane_3DRectnagle()
/// 3. from the vertices of this linessegment with vertices of polygons create new faces (1 to 4 new faces)
void ExtractFaces_from_3DPolygonsIntersection(LaserPoints laserPoints, Planes planes,
                                              int min_segment_size, char* root,
                                              ObjectPoints polygons_vertices, //input polygons of segments
                                              LineTopologies polygons_edges, //input polygons of segments
                                              ObjectPoints &vertices,  //output faces
                                              LineTopologies &faces) // //output faces
{   char str_root[500];
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
             planes_temp_map.insert (std::pair<int, Plane> (pl.Number(), pl));
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
     /// make a multimap of polygons and corresponding vertices
     /// the assumption is that polygon number and segmentNumber are the same for each segment
     std::multimap<int, LineTopology> polygons_map;
     std::multimap<int, ObjectPoints> vertices_map;
     // accumulate polygons map and vertices map
    for (auto& polygon : polygons_edges){
        int polygon_num = polygon.Number();
        ObjectPoints polygon_vertices = GetCorresponding_vertices(polygons_vertices, polygon);
        // if polygon is closed remove the duplicate point
        if(polygon.IsClosed()) {
            PointNumber lastPoint((polygon.end()-1)->Number());
            //cout << "polygon is closed!!!" << lastPoint.Number() << endl;
            //polygon.Remove(lastPoint);
            LineTopologies polys_tmp;
            polys_tmp.push_back(polygon);
            polygon_vertices.RemoveDoublePoints(polys_tmp, 0.0001);
        }
        polygons_map.insert (std::pair<int, LineTopology> (polygon.Number(), polygon));
        vertices_map.insert (std::pair<int, ObjectPoints> (polygon.Number(), polygon_vertices));
    }
    //TODO: loop through each segment and other segment/polygon/plane, intersect them, and collect list of created polygons.
}


/// NOT FINISHED
void ExtractFaces_from_TwoPolygons_Intersection(LineTopology polygon1, ObjectPoints vertices1, Plane plane1,
                                                LineTopology polygon2, ObjectPoints vertices2, Plane plane2,
                                                LineTopologies &faces, ObjectPoints &vertices)
{
    LineSegments3D clipped_polygon;
    LineSegment3D lineSegment3d;
    int count=0;
    /// this intersects plane2 with the edges of polygon1
    if(Intersect_Plane_3DRectnagle(polygon1, vertices1, plane2, lineSegment3d)){
        /// create new vertices and new topology,
        /// check if vertices are the same(within an epsilon dist) use one of them (later RemoveDoublePoints())
        clipped_polygon.push_back(lineSegment3d);
        PointNumber pn1, pn2;
        Covariance3D cov3d;
        cov3d = Covariance3D(0, 0, 0, 0, 0, 0);
        /// convert begin and end-point of line intersect to objpoint
        count++;
        pn1 = PointNumber(count);
        ObjectPoint beginp = ObjectPoint(lineSegment3d.BeginPoint(), pn1, cov3d);
        count++;
        pn2 = PointNumber(count);
        ObjectPoint endp = ObjectPoint(lineSegment3d.EndPoint(), pn2, cov3d);
        //TODO: how to make new faces in plane1/polygon1???
        //polygon_vertices.push_back(beginp); // this is the polygon of the plane2 we want the polygons/faces created in plane1
        //polygon_vertices.push_back(endp);
        /// create the line_topology
        LineTopology edge_topo;
        //edge_topo = LineTopology(polygon_number, 1, pn1, pn2);
        //polygon_edges.push_back(edge_topo);
        //intersection = true;
    }
}


/// TODO: better to create a class for bounding_cube which should have:
/// 6faces, 6 planes, 12edges/3Dlinesegments, 8 vertices, centroid
/// methods: obtain edges of each face
// because planes are infinite, we want to bound them with the bound of the input laser
// create a cuboid with 6 faces around the input data as the bound planes
Planes bounding_cube(char *root_dir, LaserPoints &lp, ObjectPoints &lp_vertices,
        LineTopologies &lp_faces, std::map<int, Positions3D> &planes_pos_map) {
    char str_root[500];
    strcpy (str_root, root_dir);

    DataBoundsLaser dataBoundsLaser;
    dataBoundsLaser = lp.DeriveDataBounds(0);
    //lp.DeriveTIN(); is taken care of in Min3DBox_OFF()
    //lp.EnclosingRectangle(0.10, lp_vertices, lp_faces);
    LaserPoints lp_noSeg;
    lp_noSeg = lp;
    lp_noSeg.RemoveAttributes();
    lp_noSeg.SetAttribute(SegmentNumberTag, 0); // for Min3DBox_OFF() all points should have one segment number
    char *OFFout;
    OFFout = strcat(str_root, "/cube_global.off");
    /// create a cuboid from lp_noSeg: (lp_vertices, lp_faces)
    Min3DBox_OFF(lp_vertices, lp_faces, lp_noSeg, 20, OFFout,
                 dataBoundsLaser.Minimum().Z(), dataBoundsLaser.Maximum().Z(), 0);

    strcpy(str_root, root_dir);
    lp_vertices.Write(strcat(str_root, "/cube_global_vertices.objpts"));
    strcpy(str_root, root_dir);
    lp_faces.Write(strcat(str_root, "/cube_global_edges.top"), false);
    // create 6 planes from the global cuboid
    Planes faces_planes;
    for (int i =0; i < lp_faces.size(); i++){  // loop in 6 faces of the cuboid
        PointNumberList pnumlist = lp_faces[i].PointNumberListReference();
        ObjectPoints face_points;
        Positions3D face_vertices;
        int plane_num = i+1; // to start from 1 not 0
        for (int p=0; p < pnumlist.size() ; p++) { // there should be 4 points in this list
            ObjectPoint vertex = lp_vertices.GetPoint(pnumlist[p].NumberRef())->ObjectPointRef();
            face_points.push_back(vertex);
            face_vertices.push_back(vertex);
            // when face vertices are three reconstruct the plane
            //WARNING: we ignore the fourth point of the face for creating the plane later we should project it to the plane
            cout <<  "face size:" << face_points.size() << endl;
            if(face_points.size() == 3){
                Plane face_pl(face_points[0], face_points[1], face_points[2]);
                face_pl.Number() = plane_num;
                faces_planes.push_back(face_pl);
            }
/*            if(face_points.size() ==4){
                //TODO: it is better instead of the fourth vertex, its projection on the plane to be used.
                // so when we export the four points they are coplanar
                Plane lastplane = faces_planes.back();
                Position3D fourth_vertex_projection = lastplane.Project(face_points[4]);
                face_points[4] = fourth_vertex_projection; ???
            }*/
            // now we replace the fourth projection with its projection

        }
        planes_pos_map.insert(std::pair<int, Positions3D> (plane_num, face_vertices));
    }
    strcpy(str_root, root_dir);
    faces_planes.Write(strcat(str_root, "/cube_global_planes.planes"));

    return faces_planes;
}

/// we use counterclockwise to order the vertices for each face, the right thumb always pointing outside
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


/// TODO look at Use: polygon.IntersectPolygonByLine() for a better implementation
/// intersecting a plane with the edges of a clipping rectangle (or polygon)
///  to obtain the line segment that cuts the rectangle
bool Intersect_Plane_3DRectnagle(LineTopology clipperRectangle_edges,
                                 ObjectPoints clipperRectangle_vertices,
                                 const Plane& plane, LineSegment3D &clipped_line){
    // we intersect the plane with the edges of the clipping rectangle
    // get the edges as linesegments
    if(!clipperRectangle_edges.IsClosed()){
        PointNumber pointNumber(clipperRectangle_edges.front());
        clipperRectangle_edges.push_back(pointNumber); // close the polygon to make a polygon topology
    }
    LineSegments3D rect_edges(clipperRectangle_vertices, clipperRectangle_edges);

    DataBounds3D bounds3D = clipperRectangle_vertices.Bounds();
    cout << "bound Min:"  << bounds3D.Minimum();
    cout << "bound Max:"  << bounds3D.Maximum() << endl;

    // intersect the plane with the edges
    Positions3D intersected_poses;
    LaserPoints intersected_points;
    // for each edge of the clipping rectangle find the intersection point with the plane
    int cnt=0;
    for (auto &e : rect_edges){
        cout << "EDGE: " << ++cnt << endl; //DEBUG
        // make a line3d from each edge
        Line3D edge_line(e.BeginPoint().Position3DRef(), e.EndPoint().Position3DRef());
        // intersect this edge_line to the plane
        Position3D intersected_pos;
        if(IntersectLine3DPlane(edge_line, plane, intersected_pos)){
            /// alternative solution to slect the line segment:
            /// add all points to the ObjPoints or LaserPoints and sort them on coords
            /// Then from the most left one or lowest one pair them as line segments,
            /// take the middle point and check if it is inside the polygon
            /// check if the intersected_pos is between the vertices of
            /// the linesegment and not outside of the clipping polygon
            // TODO look at Use: polygon.IntersectPolygonByLine() for a better implementation

            /// this doesn't work
           cout << "intersection position:" << intersected_pos; // << endl later
//           if(Point_Inside_3DLine(intersected_pos, e, 0.01)){
//               cout << "point on the line segment" << endl;
//              intersected_poses.push_back(intersected_pos);
//              intersected_points.push_back(intersected_pos);
//           }
             /// this checks with all 4 vertices, maybe a better way is just to check with the linesegment points
            if(bounds3D.Inside(intersected_pos, 0.01)){
                 cout << " Inside!" << endl; //DEBUG
                intersected_poses.push_back(intersected_pos);
                intersected_points.push_back(intersected_pos);
            } else cout << " Outside!" << endl; //DEBUG
        } else
            return false; // edge-line and plane are parallel
    }
    if(intersected_points.size() == 0){
       cout << "WARNINGS! NO INTERSECTION!!!" << endl;
       return false;
    }
    if(intersected_points.size() == 1){
       cout << "WARNINGS! intersected pose is just ONE!!!" << endl;
       return false;
    }
    clipped_line.Initialise();
    if(intersected_poses.size() == 2){
        cout << "Successfull! intersected poses are TWO!!!" << endl;
        LineSegment3D clipped_line_temp(intersected_poses[0], intersected_poses[1]); // this is very error prone if 0 and 1 have wrong orders
        clipped_line = clipped_line_temp;
    }

    if(intersected_poses.size() > 2){
        cout << "WARNINGS! intersected poses are more than TWO!!!" << endl;
        LineSegment3D clipped_line_temp(intersected_poses[0], intersected_poses[2]);// why 0 and 2 // this is very error prone if 0 and 2 have wrong orders
        clipped_line = clipped_line_temp;
    }
    //intersected_points.Write("intersected_points.laserpoints", false);

    return true;
}


/// the input polygon is split to two new_polygons with a given line_segment in 3D space)
/// look at void test_SplitPolygon3DByLineSegment3D() for test
/// this function updates the list of polygon_points()
bool SplitPolygon3DByLineSegment3D(ObjectPoints &polygon_points,
                                   LineTopology &polygon_edges,
                               const LineSegment3D &line_segment,
                               double snap_dist,
                               LineTopologies &new_polygons)
{
  // Check if the begin or end point is already near a node of the polygon
  int begin_index, end_index;
  begin_index = end_index = -1;
  LineTopology::const_iterator   node_it;
  ObjectPoints::const_iterator   corner_it;
  for (node_it=polygon_edges.begin(); node_it!=polygon_edges.end()-1; node_it++) {
    corner_it = polygon_points.ConstPointIterator(*node_it);
    if((line_segment.BeginPoint().Distance(corner_it->Position3DRef())) < snap_dist)
        begin_index = polygon_points.FindPoint(*node_it);
    if((line_segment.EndPoint().Distance(corner_it->Position3DRef())) < snap_dist)
        end_index = polygon_points.FindPoint(*node_it);
    //if (Distance(edge_segment.BeginPoint(), Position2D(corner_it->vect2D())) < snap_dist)
    //  begin_index = polygon_points.FindPoint(*node_it);
    //if (Distance(edge_segment.EndPoint(), Position2D(corner_it->vect2D())) <snap_dist)
     // end_index = polygon_points.FindPoint(*node_it);
  }
  // Determine the edges in which new nodes need to be inserted
  LineTopology polygon = polygon_edges;
  //ObjectPoint new_point;
  LineSegments3D segments = LineSegments3D(polygon_points, polygon_edges);
  LineSegments3D::const_iterator segment_it;
  int begin_segment_index = -1;
  int segment_index, found;
  if (begin_index == -1) {    // New node needed for begin of line segment
    // Add the new point
    ObjectPoint new_point;
    new_point.X() = line_segment.BeginPoint().X();
    new_point.Y() = line_segment.BeginPoint().Y();
    new_point.Z() = line_segment.BeginPoint().Z();
    new_point.Number() = (polygon_points.end()-1)->Number() + 1;
    polygon_points.push_back(new_point);
    begin_index = polygon_points.size() - 1;
    // Find the segment
    for (segment_it=segments.begin(), found=0, segment_index=0;
         segment_it!=segments.end() && !found; segment_it++, segment_index++) {
      if (segment_it->Distance(line_segment.BeginPoint()) < 0.01) {
        found = 1;
        begin_segment_index = segment_index;
      }
    }
    // Insert the new node into the polygon
    polygon.insert(polygon.begin() + begin_segment_index + 1, new_point.NumberRef());
  }

  int end_segment_index;
  if (end_index == -1) {      // New node needed for end of line segment
    // Add the new point
    ObjectPoint new_point;
    new_point.X() = line_segment.EndPoint().X();
    new_point.Y() = line_segment.EndPoint().Y();
    new_point.Z() = line_segment.EndPoint().Z();
    new_point.Number() = (polygon_points.end()-1)->Number() + 1;
    polygon_points.push_back(new_point);
    end_index = polygon_points.size() - 1;
    // Find the segment
    for (segment_it=segments.begin(), found=0, segment_index=0;
         segment_it!=segments.end() && !found; segment_it++, segment_index++) {
      if (segment_it->Distance(line_segment.EndPoint()) < 0.01) {
        found = 1;
        end_segment_index = segment_index;
      }
    }
    // Insert the new node into the polygon
    if (begin_segment_index != -1 && end_segment_index > begin_segment_index)
        end_segment_index++;
    polygon.insert(polygon.begin() + end_segment_index + 1, new_point.NumberRef());
  }
  return polygon.Split(polygon_points[begin_index].NumberRef(),
                       polygon_points[end_index].NumberRef(), new_polygons);
}

/// we use two segments (which we know they intersect) to create the intersection line segment
/// and use it to split segment1 with the plane of segment2
void test_SplitPolygon3DByLineSegment3D(ObjectPoints &polygon_v, LineTopologies &polygon_e,
                                        LaserPoints segment,
                                        ObjectPoints &new_polygon_v, LineTopologies &new_poly_e)
{
    Plane plane = segment.FitPlane(segment[0].SegmentNumber());
    plane.Number() = segment[0].SegmentNumber(); // this is necessary
    LineSegment3D linesegment;
    LineTopology polygon_top;
    polygon_top = polygon_e[0];
    if(Intersect_Plane_3DRectnagle(polygon_top, polygon_v, plane, linesegment))
    {
        SplitPolygon3DByLineSegment3D(polygon_v, polygon_top, linesegment, 0.01, new_poly_e);
    } else cout << "polygons don't intersect!" << endl;
}

/// intersecting a plane with the 12edges of a 3DBox to obtain the polygon/face
/// intersection of a plane and a 3DBox can be a polygon: a triangle, rectangle, pentagon or a hexagon (6).
/// \param clipper_edges_topo  clipping 3DBox
/// \param clipper_vertices  clipping 3DBox
/// \param plane  target plane which intersects the edges of the 3DBox
/// \return  the line segment which is inside the plane of the clipping 3DBox and is bounded by the edges
/*bool Intersect_Plane_3DBoxEdges (LineTopologies clipperBox_edges,  ObjectPoints clipperBox_vertices, Plane plane){
    // the problem of this method is that we get the intersection points but we dont have their order of connection
    // to create the clipping polygon/hexagon --> solution: use create the convexhull
    // or calcualte the scalars: look at this function in 2D : int LineTopology::IntersectPolygonByLine()

}*/


/// intersecting a plane with 6 faces of a 3D box using Intersect_Plane_3DRectnagle() to obtain the polygon/face
/// intersection of a plane and a 3DBox can be a polygon: a triangle, rectangle, pentagon or a hexagon (6).
/// NOTE1: we drop very close vertices of the polygon <0.01m
/// NOTE2: faces of the 3Dbox DONOT need to be a closed polygon (0-1-2-3)
// WHY int polygon_number is required?
bool Intersect_Plane_3DBoxFaces(LineTopologies box3d_faces, const ObjectPoints &box3d_vertices, //input
                           const Plane &plane, int polygon_number, //input
                           LineTopologies &polygon_edges, ObjectPoints &polygon_vertices) //output
{
    bool intersection = false;
    LineSegments3D clipped_polygon; //a triangle, rectangle, pentagon or a hexagon (3-6 vertices)
    int count=0;
    for(auto &face_edges : box3d_faces){ // cuboid has 6 faces and each face has 4 edges
        cout << "Box Face: " << ++count << endl; //DEBUG
        // fetch the exact vertices for this face
        ObjectPoints face_vertices = GetCorresponding_vertices(box3d_vertices, face_edges);
        LineSegment3D linesegment;
        if(Intersect_Plane_3DRectnagle(face_edges, face_vertices, plane, linesegment)){
            /// create new vertices and new topology,
            /// check if vertices are the same(within an epsilon dist) use one of them (later RemoveDoublePoints())
            clipped_polygon.push_back(linesegment);
            intersection = true;
        }
    }
    clipped_polygon.PointsWithTopology(polygon_vertices, polygon_edges, true);
    cout << "# polygon vertices:" << polygon_vertices.size() << endl;
    /// because linesegments have common vertices on the cuboid surface, the duplicates should be removed
    polygon_vertices.RemoveDoublePoints(polygon_edges, 0.01);
    cout << "# polygon vertices after edit:" << polygon_vertices.size() << endl;

    return intersection;
}

/// 1. fit planes to each segment in segments
/// 2. use Intersect_Plane_3DBoxFaces() to intersect the plane to the 3DBox
/// 3. get the created polygons of each segment as a result of intersection
/// NOTE: later we use these polygons to intersect them to each other obtain new polygons/faces
/// this is tested, it works. use it for intersecting planes with the bounding box of the data
void Intersect_Planes_3DBoxFaces(LaserPoints segments, int min_seg_size,
                                 LineTopologies box3d_faces,const ObjectPoints box3d_vertices,
                                 LineTopologies &polygons_edges, ObjectPoints &polygons_vertices,
                                 bool close, bool verbose)
{
    char* root_dir;
    root_dir = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out/";
    char str_root[500];
    strcpy (str_root, root_dir);
    int next_number; // initialized later
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(segments, SegmentNumberTag, root_dir);
    sort(segments_vec.begin(), segments_vec.end(), compare_lp_segmenttag);
    cout << "Number of segments: " << segments_vec.size() << endl;
    for (auto &segment : segments_vec){
        if(segment.size() < min_seg_size) continue; // to skip very small segments
        int seg_num = segment[0].SegmentNumber();
        if(seg_num < 0) continue; // to skip non valid segments
        // fit a plane
        Plane plane;
        LineTopologies polygon_edges;
        ObjectPoints polygon_vertices;
        plane = segment.FitPlane(seg_num);
        plane.Number() = seg_num;
        cout << "plane number: " << plane.Number() << endl;
        if(Intersect_Plane_3DBoxFaces(box3d_faces, box3d_vertices, plane, seg_num,
                                           polygon_edges, polygon_vertices)){
            //polygon_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices.objpts");
            //polygon_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges.top", false);
            // Store points and topology in one file
            if (polygons_vertices.empty()){
                next_number = 0;
            } else {
                next_number = (polygons_vertices.back ()).Number () + 1;
            }

            /// update the numbering for objectpoints of corners
            PointNumber pnumber;
            LineTopology poly_tmp_edges;
            for (int i=0; i < polygon_vertices.size() ; i++){
                pnumber = PointNumber(next_number + i);
                //corner = ObjectPoint(corners[i], pnumber, cov3d);
                polygon_vertices[i].NumberRef () = pnumber;
                polygons_vertices.push_back(polygon_vertices[i]);
                poly_tmp_edges.push_back(pnumber);  // making the linetopology
            }
            if(close) poly_tmp_edges.push_back(PointNumber(next_number)); // Close the polygon (for visualisation in pcm)
            if (poly_tmp_edges.IsClockWise(polygons_vertices)){
                poly_tmp_edges.MakeCounterClockWise(polygons_vertices);
            }else{
                poly_tmp_edges.MakeClockWise(polygons_vertices);
            }
            poly_tmp_edges.Number () = seg_num; //++line_number;
            polygons_edges.push_back (poly_tmp_edges);
        } else
            cout << "NO Intersection!!!" << endl;
    }
}

/// test intersct a plane and a 3DBox
void test_Intersect_Plane_3DBoxFaces(LaserPoints segment, LineTopologies box3d_faces,
                                      const ObjectPoints& box3d_vertices)
{
    Plane plane;
    LineTopologies polygon_edges;
    ObjectPoints polygon_vertices;
    plane = segment.FitPlane(segment[0].SegmentNumber());
    plane.Number() = segment[0].SegmentNumber();
    cout << "plane number: " << plane.Number() << endl;
    if(Intersect_Plane_3DBoxFaces(box3d_faces, box3d_vertices, plane, segment[0].SegmentNumber(),
                                       polygon_edges, polygon_vertices)){
        polygon_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_vertices.objpts");
        polygon_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_edges.top", false);
    } else
        cout << "NO Intersection!!!" << endl;
}

/// test intersct a plane and a rectangle
void test_Intersect_Plane_3DRectnagle(LaserPoints segment, LineTopologies bbox_faces, const ObjectPoints& bbox_vertices){
    //segment.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/seg1_crop.laser");
    //bbox_faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_edges.top", false);
    //bbox_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_vertices.objpts");
    LineSegments3D clipped_lines;
    int cnt =1;
    for(auto &face : bbox_faces){ // 6 faces each face has 4 edges
        cout << "\n***** FACE: " << cnt++ << " ******" << endl;
        PointNumberList pnumlist = face.PointNumberListReference();
        cout << "face list: ";
        for (auto &pnum : pnumlist)        //DEBUG
            cout << pnum.Number() << "-";
        cout << endl;
        LineSegment3D clipped_line;
        Plane plane = segment.FitPlane(segment[0].SegmentNumber());
        plane.Number() = segment[0].SegmentNumber();
        //// DEBUG ////
        /// draw the planes
        //void VisualizePlanes(LaserPoints segmented_laserpoints, int minsizesegment,
         //                    ObjectPoints &corners, LineTopologies &polygons, char* root, bool verbose)
        ObjectPoints planes_corners;
        LineTopologies planes_polygons;
        char *root = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out/";
        VisualizePlanes(segment, 10, planes_corners, planes_polygons, root, false);
        planes_corners.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/planes_corners.objpts");
        planes_polygons.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/planes_polygons.top", false);
        //////////////
        ObjectPoints face_vertices = GetCorresponding_vertices(bbox_vertices, face);
        if(Intersect_Plane_3DRectnagle(face, face_vertices, plane, clipped_line)){
            clipped_line.Label() = segment[0].SegmentNumber();
            clipped_lines.push_back(clipped_line);
        } else
            cout << "No intersection!!!" << endl;

        //// DUBUG /////////
        // draw the linesegment
        LineTopologies line_top;
        ObjectPoints line_vertices;
        clipped_line.PointsWithTopology(line_vertices, line_top, 0);
        line_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/line_vertices.objpts");
        line_top.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/line_top.top", false);
        ////////////////////
        // TODO: draw the plane rectangle to visualize it
    }
    // draw the linesegments
    LineTopologies linesegments_top;
    ObjectPoints linesegments_vertices;
    clipped_lines.PointsWithTopology(linesegments_vertices, linesegments_top, 1); // append???

    linesegments_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/linesegments_vertices.objpts");
    linesegments_top.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/linesegments_top.top", false);
}


// for a given list of polygons match each polygon topology and corresponding vertices and return them as vectors
// in this way by looping in the vector of polygons we can get the corresponding vertices from the vector of vertices
void TopologyAndVertices (const LineTopologies &topologies,const ObjectPoints &vertices,
                            vector<LineTopology> &topology_vec, vector<ObjectPoints> &vertices_vec){
    for (int i =0; i < topologies.size(); i++) {
        ObjectPoints vertices_points = GetCorresponding_vertices(vertices, topologies[i]);
        vertices_vec.push_back(vertices_points); // we dont need to erase vertices_points because it is regenerated everytime
        topology_vec.push_back(topologies[i]);
    }
}

ObjectPoints GetCorresponding_vertices(const ObjectPoints &vertices, const LineTopology &topology){
    PointNumberList pnumlist = topology.PointNumberListReference();
    ObjectPoints vertices_points;
    // warning: this doesn't chekc whether the vertex is already exist in vertices_points
    for (int p = 0; p < pnumlist.size(); p++) { // these are points for 1 polygon in topologies
        ObjectPoint vertex = vertices.GetPoint(pnumlist[p].NumberRef())->ObjectPointRef();
        vertices_points.push_back(vertex);
    }
    return vertices_points;
}

/// needs a test
bool Point_Inside_3DLineBounds(const Position3D &point, const LineSegment3D &lineseg3D, double margin)
{
  Position3D minimum, maximum, beginPoint, endPoint;
  beginPoint = minimum = lineseg3D.BeginPoint().Position3DRef();
  endPoint   = maximum = lineseg3D.EndPoint().Position3DRef();
  // switch the value of minimum and max if the end point has smaller x,y,z
  if(beginPoint.X() > endPoint.X()) minimum.X() = endPoint.X(); maximum.X() = beginPoint.X();
  if(beginPoint.Y() > endPoint.Y()) minimum.Y() = endPoint.Y(); maximum.Y() = beginPoint.Y();
  if(beginPoint.Z() > endPoint.Z()) minimum.Z() = endPoint.Z(); maximum.Z() = beginPoint.Z();

    // now check if the point is in the bounds of the linesegment
  if (point.X() < minimum.X() - margin) return(false);
  if (point.X() >= maximum.X() + margin) return(false);
  if (point.Y() < minimum.Y() - margin) return(false);
  if (point.Y() >= maximum.Y() + margin) return(false);
  if (point.Z() < minimum.Z() - margin) return(false);
  if (point.Z() >= maximum.Z() + margin) return(false);
  return(true);
}


/// from LineTopology()
///     /// Intersect a closed polygon (this) by a line.
/** A closed polygon defined by the line topology (this) and a set of object
    points is intersected in the XY-plane by a line. The intersection
    results into no, one or multiple line segments. Within this function,
    the edges of the polygon are converted to a set of line segments. These
    segments are kept in a static variable. In case of multiple calls to
    this one can opt to reuse these line segments to speed up computations.
    @param points             The points of the polygon
    @param line               The line to intersect with the polygon
    @param intersections      The computed intersecting line segments. This
                              vector is initialised if the last parameter
                              is 0.
    @param err_dist           The distance tolerance to decide about
                              intersections.
    @param min_length         The minimal length of intersecting line
                              segments to be returned.
    @param reuse_pol_segments Flag to reuse the polygon line segments of
                              a previous call. See explanation above.
    @return The number of intersecting line segments
*/
//int LineTopology::IntersectPolygonByLine(const ObjectPoints &points,
//                                         const Line2D &line,
//                                         LineSegments2D &intersections,
//                                         double err_dist, double min_length,
//                                         int reuse_pol_segments) const
//{
//  static LineSegments2D          segments;
//  LineSegments2D::const_iterator segment;
//  Positions2D                    int_points;
//  Position2D                     int_point;
//  Positions2D::iterator          pos, pos1, pos2;
//  ObjectPoint2D                  mid_point;
//  vector <double>                scalars;
//  vector <double>::iterator      scalar, scalar1, scalar2;
//  int                            old_num_intersections;

//// Clear old data and (re-)compute polygon segments from the point with topology
//  if (!reuse_pol_segments) {
//    if (!intersections.empty())
//      intersections.erase(intersections.begin(), intersections.end());
//    if (!segments.empty())
//      segments.erase(segments.begin(), segments.end());
//    segments = LineSegments2D(points, *this);
//  }
//  old_num_intersections = intersections.size();
//// Collect all intersection positions
//  for (segment=segments.begin(); segment!=segments.end(); segment++)
//    if (segment->Intersect(line, int_point, err_dist))
//      int_points.push_back(int_point);
//// Compute the scalars along the line for all intersection points
//  for (pos1=int_points.begin(); pos1!=int_points.end(); pos1++)
//    scalars.push_back(line.Scalar(*pos1));
//// Check all segments between two intersection points
//  while (int_points.size() >= 2) {
//    // Find the two smallest scalars
//    if (scalars[0] < scalars[1]) {
//      scalar1 = scalars.begin();  pos1 = int_points.begin();
//      scalar2 = scalar1 + 1;      pos2 = pos1 + 1;
//    }
//    else {
//      scalar2 = scalars.begin();  pos2 = int_points.begin();
//      scalar1 = scalar2 + 1;      pos1 = pos2 + 1;
//    }
//    for (scalar=scalars.begin()+2, pos=int_points.begin()+2;
//         scalar!=scalars.end();  scalar++, pos++) {
//      if (*scalar < *scalar1) {
//        scalar2 = scalar1;  pos2 = pos1;
//        scalar1 = scalar;   pos1 = pos;
//      }
//      else if (*scalar < *scalar2) {
//        scalar2 = scalar;   pos2 = pos;
//      }
//    }
//    // Check the segment length
//    if ((*scalar2 - *scalar1) >=  min_length) {
//      // Check if the mid point of the line segment is inside the polygon
//      mid_point.vect() = (*pos1 + *pos2) / 2.0;
//      if (mid_point.InsidePolygon(points, *this)) {
//        // Add the segment to the list of intersecting line segments
//        intersections.push_back(LineSegment2D(*pos1, *pos2));
//      }
//    }
//    // Remove the smallest scalar and the corresponding intersection point
//    scalars.erase(scalar1);
//    int_points.erase(pos1);
//  }
//  return(intersections.size() - old_num_intersections);
//}

