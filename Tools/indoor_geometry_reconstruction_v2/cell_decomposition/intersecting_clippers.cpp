#include "intersecting_clippers.h"
#include "face_selection.h"
#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
//#include <syslimits.h>
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"

// TODO: add cgal ransac shape detection for segmentation (done)
// TODO: add an int=nextnumber routine to cell decomposition to store the relation between segments and children faces (done, by linelabeltag)
// TODO: forceverticality to planes near vertical and also for horzontal planes
// TDOO: enlarge the bbox by a percetage so it is not very tight to the data (done)
// TODO: select faces belong to the same cell
// TODO: export cell decomposition to off or obj
// TODO: put the pipeline together

/// we use this instead of Planes bounding_cube() so we can scale the boundingbox
void bounding_cube(char *off_threedbox_out, LaserPoints lp, double scalefactor,
                   ObjectPoints &lp_vertices, LineTopologies &lp_faces){
//    char str_root[500];
//    strcpy (str_root, root_dir);
//    char *OFFout;
//    OFFout = strcat(str_root, "/cube_global.off");
    /// create a cuboid from lp_noSeg: (lp_vertices, lp_faces)
    Min3DBox_OFF_withscale(lp_vertices, lp_faces, lp, scalefactor, off_threedbox_out);
}


/// TODO: better to create a class for bounding_cube which should have:
/// 6faces, 6 planes, 12edges/3Dlinesegments, 8 vertices, centroid
/// methods: obtain edges of each face
// because planes are infinite, we want to bound them with the bound of the input laser
// create a cuboid with 6 faces around the input data as the bound planes
Planes bounding_cube(char *root_dir, LaserPoints &lp, ObjectPoints &lp_vertices,
        LineTopologies &lp_faces, double enlarge_size, std::map<int, Positions3D> &planes_pos_map) {
    char str_root[500];
    strcpy (str_root, root_dir);

    DataBoundsLaser dataBoundsLaser;
    dataBoundsLaser = lp.DeriveDataBounds(0);

/*  /// enlarge the databounds
    LaserPoint new_boundMin;
    new_boundMin.X() = dataBoundsLaser.Minimum().X() - enlarge_size;
    new_boundMin.Y() = dataBoundsLaser.Minimum().Y() - enlarge_size;
    new_boundMin.Z() = dataBoundsLaser.Minimum().Z() - enlarge_size;
    LaserPoint new_boundMax;
    new_boundMax.X() = dataBoundsLaser.Maximum().X() + enlarge_size;
    new_boundMax.Y() = dataBoundsLaser.Maximum().Y() + enlarge_size;
    new_boundMax.Z() = dataBoundsLaser.Maximum().Z() + enlarge_size;
    /// add new bounds to the laserpoints to enlarge it // this is not a good approach because it creates an axis-aligend bbox
    lp.push_back(new_boundMin);
    lp.push_back(new_boundMax);
    */

    ////lp.DeriveTIN(); is taken care of in Min3DBox_OFF()
    ////lp.EnclosingRectangle(0.10, lp_vertices, lp_faces);
    LaserPoints lp_noSeg;
    lp_noSeg = lp;
    lp_noSeg.RemoveAttributes();
    lp_noSeg.SetAttribute(SegmentNumberTag, 0); // for Min3DBox_OFF() all points should have one segment number
    char *OFFout;
    OFFout = strcat(str_root, "/threedbox.off");
    /// create a cuboid from lp_noSeg: (lp_vertices, lp_faces)
    Min3DBox_OFF(lp_vertices, lp_faces, lp_noSeg, 20, OFFout,
                 dataBoundsLaser.Minimum().Z(), dataBoundsLaser.Maximum().Z(), 0);
    //Min3DBox_OFF(lp_vertices, lp_faces, lp_noSeg, 20, OFFout, new_boundMin.Z(), new_boundMax.Z(), 0);

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
/// TODO: check if the polygon and plane are almost-prallel skip the intersection
/// intersecting a plane with the edges of a clipping rectangle (or polygon) to obtain the clipped_line segment
///  to obtain the line segment that cuts the rectangle
/// USAGE: look at Intersect_Plane_3DBoxFaces() for usage
bool Intersect_Plane_3DRectnagle(LineTopology clipperRectangle_edges,
                                 ObjectPoints clipperRectangle_vertices,
                                 const Plane& plane, LineSegment3D &clipped_line, bool verbose){
    // we intersect the plane with the edges of the clipping rectangle
    // get the edges as linesegments
    if(!clipperRectangle_edges.IsClosed()){
        PointNumber pointNumber(clipperRectangle_edges.front());
        clipperRectangle_edges.push_back(pointNumber); // close the polygon to make a polygon topology
    }
    LineSegments3D rect_edges(clipperRectangle_vertices, clipperRectangle_edges);

    DataBounds3D bounds3D = clipperRectangle_vertices.Bounds();
//    if(verbose){
//        cout << "bound Min:"  << bounds3D.Minimum();
//        cout << "bound Max:"  << bounds3D.Maximum() << endl;
//    }


    // intersect the plane with the edges
    Positions3D intersected_poses;
    //LaserPoints intersected_points;
    // for each edge of the clipping rectangle find the intersection point with the plane
    int cnt=0;
    for (auto &e : rect_edges){
        //if(verbose) cout << "EDGE: " << ++cnt << endl; //DEBUG
        // make a line3d from each edge
        Line3D edge_line(e.BeginPoint().Position3DRef(), e.EndPoint().Position3DRef());
        // intersect this edge_line with the plane
        Position3D intersected_pos;
        if(IntersectLine3DPlane(edge_line, plane, intersected_pos)){
            // TODO look at Use: polygon.IntersectPolygonByLine() for a better implementation

            /// check if the intersecte point is within the bound of the edge
           //if(verbose) cout << "intersection position:" << intersected_pos; // << endl later
           if(Point_Inside_3DLineBounds(intersected_pos, e, 0.01)){
               //cout << "point on the line segment" << endl;
              intersected_poses.push_back(intersected_pos);
              //intersected_points.push_back(intersected_pos);
           }
             /// this checks with all 4 vertices, maybe a better way is just to check with the linesegment points
             ///  this one if the cuboid of the data is almost axis-aligend, it results in some polygons overpass the boundary
 /*           if(bounds3D.Inside(intersected_pos, 0.01)){
                 //if(verbose) cout << " Inside!" << endl; //DEBUG
                intersected_poses.push_back(intersected_pos);
                intersected_points.push_back(intersected_pos);
            } //else if(verbose) cout << " Outside!" << endl; //DEBUG
            */
        } else
            continue;    //return false; // edge-line and plane are parallel
    }
    if(intersected_poses.size() == 0){
       //if(verbose) cout << "WARNINGS! NO INTERSECTION!!!" << endl;
       return false;
    }
    if(intersected_poses.size() == 1){
       //if(verbose) cout << "WARNINGS! intersected pose is just ONE!!!" << endl;
       return false;
    }
    clipped_line.Initialise();
    if(intersected_poses.size() == 2){
        //if(verbose) cout << "Successfull! intersected poses are TWO!!!" << endl;
        LineSegment3D clipped_line_temp(intersected_poses[0], intersected_poses[1]);
        clipped_line = clipped_line_temp;
    }

    if(intersected_poses.size() > 2){ /// this case shouldn't happen if function Point_Inside_3DLine() works properly
        //if(verbose) cout << "WARNINGS! intersected poses are more than TWO!!!" << endl;
        LineSegment3D clipped_line_temp(intersected_poses[0], intersected_poses[2]);// why 0 and 2 // this is very error prone if 0 and 2 have wrong orders
        clipped_line = clipped_line_temp;
        //return false;
    }
    //intersected_points.Write("intersected_points.laser", false);

    return true;
}


/// the input polygon is split to two new_polygons with a given line_segment in 3D space)
/// look at void SplitPolygons3DByPlane3D() for test
/// this function updates the list of polygon_points()
/// USAGE: this function is used in SplitPolygons3DByPlanes3D() for cell decomposition creation
bool SplitPolygon3DByLineSegment3D(ObjectPoints &polygon_points,
                                   LineTopology &polygon_edges,
                               const LineSegment3D &line_segment, double snap_dist,
                               LineTopologies &new_polygons)
{
    if(polygon_points.empty() || polygon_edges.empty())
        return false;
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

  int end_segment_index=-1;
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

/// splitting several polygons by one segment/plane
/// USAGE: we don't use this in the pipeline it is a practice example for void SplitPolygons3DByPlanes3D()
void SplitPolygons3DByPlane3D(ObjectPoints &polygons_v, LineTopologies &polygons_e, LaserPoints segment,
                                        ObjectPoints &new_polygons_v, LineTopologies &new_polygons_e)
{
    /// we assume segment has only one segment
    Plane plane = segment.FitPlane(segment[0].SegmentNumber());
    plane.Number() = segment[0].SegmentNumber(); // this is necessary
    /// try for several polygons and one plane
    for (auto &polygon : polygons_e){
        if(polygon.empty()){
            cout << "Warning: polygon is empty. Nothing to split!" << endl;
            continue;
        }
        if(polygon.Number() == plane.Number()) continue; // the same polygon and plane don't intersect
        LineSegment3D linesegment;
        if(Intersect_Plane_3DRectnagle(polygon, polygons_v, plane, linesegment))
        {
            LineTopologies new_poly_e;
            SplitPolygon3DByLineSegment3D(polygons_v, polygon, linesegment, 0.01, new_poly_e);
            new_polygons_e.insert(new_polygons_e.end(), new_poly_e.begin(), new_poly_e.end());
        } else
        {
            cout << "polygon " << polygon.Number() <<
                       " is not intersected by the plane!" << endl;
            new_polygons_e.push_back(polygon);
        }
    }
    new_polygons_v = polygons_v;
}

/// splitting polygons incrementally by several segments/planes
/// Also look at void HypothesisGenerator::pairwise_cut(Map* mesh) at https://github.com/LiangliangNan/PolyFit/blob/main/method/hypothesis_generator.cpp
/// USAGE: use this for cell decomposition
void SplitPolygons3DByPlanes3D(ObjectPoints &polygons_v, LineTopologies &polygons_e, LaserPoints segments, int min_seg_size,
                                        ObjectPoints &new_polygons_v, LineTopologies &new_polygons_e, bool verbose)
{
    /// create a list of planes from segments
    char* root_dir;
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(segments, SegmentNumberTag, root_dir);
    sort(segments_vec.begin(), segments_vec.end(), compare_lp_segmenttag);
    Planes planes;
    for (auto &segment : segments_vec){
        if(segment.size() < min_seg_size) continue;
        int seg_num = segment[0].SegmentNumber();
        if(seg_num < 0) continue; // to skip non valid segments
        // fit a plane
        Plane plane;
        plane = segment.FitPlane(seg_num);
        plane.Number() = seg_num;
        planes.push_back(plane);
    }
    cout << "NUmber of planes:  " << planes.size() << endl;
    cout << "NUmber of polygons:" << polygons_e.size() << endl;
    /// try for several polygons and one plane
    /// we assume all planes intersect the polygon
    for (auto &polygon : polygons_e){
        if(polygon.empty()){
            //cout << "Warning: polygon is empty. Nothing to split!" << endl;
            continue;
        }
        //LineTopology polygon = polygons_e[3]; //inx0=1, inx1=2, inx2=3, inx3=60
        if (verbose) cout << "polygon:" << polygon.Number() << " ----------------------------------------------" << endl;
        /// set the polygon number=segment number to the attribute tags
        polygon.SetAttribute(LineLabelTag, polygon.Number());
        LineTopologies polygons_to_be_cut;
        polygons_to_be_cut.push_back(polygon);
        Planes cutting_planes = planes;
        while(!cutting_planes.empty()){
            LineTopologies new_polygons, remained_polygons;
            Plane plane = *(cutting_planes.begin());
            if(polygon.Number() == plane.Number()) {
                cutting_planes.erase(cutting_planes.begin());
                if (verbose) cout << "  !X plane: " << plane.Number() << " //skip self spliting!" << endl;
                continue;
            }
            for (auto &poly_child : polygons_to_be_cut){
                if(poly_child.empty()){
                    //cout << "Warning: polygon_child is empty. Nothing to split!" << endl;
                    continue;
                }
                if (verbose) cout << "   X plane:" << plane.Number() << endl;
                LineSegment3D linesegment;
                //if(verbose) cout << "Poly child: "; poly_child.Print(); cout << endl;
                /// get the linesegment from intersectn of polyg_child and plane
                if(Intersect_Plane_3DRectnagle(poly_child, polygons_v, plane, linesegment))
                {
                    /// use the linesegment to split the poly_child to new_poly
                    LineTopologies new_poly_e;
                    SplitPolygon3DByLineSegment3D(polygons_v, poly_child, linesegment, 0.01, new_poly_e);

                    /// set the parent polygon number to children as a labeltag
                    new_poly_e.SetAttribute(LineLabelTag, polygon.Number());
                    //new_polygons = new_poly_e; // not '=' because it replaces the content
                    new_polygons.insert(new_polygons.end(), new_poly_e.begin(), new_poly_e.end());

                    //polygons_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_tmp_vertices.objpts");
                    //new_poly_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_tmp_edges.top", false);
                    //if(verbose) cout << "new polygons:" ; new_polygons.Print(); cout << endl;

                    if(new_poly_e.empty()){ // this is a hack if SplitPolygon3DByLineSegment3D() returns empty
                        remained_polygons.push_back(poly_child);
                        if (verbose) cout << "    polygon " << polygon.Number()<< "-" << poly_child.Number() <<" !X plane " << plane.Number()<< endl;
                        //if(verbose) cout << "remained_polygons:" ; remained_polygons.Print(); cout << endl;
                    }
                } else
                {
                    if (verbose) cout << "    polygon " << polygon.Number()<< "-" << poly_child.Number() <<" !X plane " << plane.Number()<< endl;
                    remained_polygons.push_back(poly_child);
                    //if(verbose) cout << "remained_polygons:" ; remained_polygons.Print(); cout << endl;
                }
            }
            polygons_to_be_cut = new_polygons; // we use '=' to replace the content with new_polygons
            polygons_to_be_cut.insert(polygons_to_be_cut.end(),
                                      remained_polygons.begin(), remained_polygons.end());
            //cout << "polygons_to_be_cut:" ; polygons_to_be_cut.Print(); cout << endl;
            //polygons_to_be_cut.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_to_be_cut.top", false);
            //new_polygons_e = polygons_to_be_cut;
            cutting_planes.erase(cutting_planes.begin());
        } // end while
        new_polygons_e.insert(new_polygons_e.end(), polygons_to_be_cut.begin(), polygons_to_be_cut.end());
    } //end for for several polygons
    /// renumber polygons from 0 to n
    /// polygons should already have the attribute of parent polygon as a LineLabelTag
    for (int i=0; i<new_polygons_e.size();i++)
    {
        new_polygons_e[i].Number()=i;
    }
    new_polygons_v = polygons_v;
}


/// intersecting a plane with 6 faces of a 3D box using Intersect_Plane_3DRectnagle() to obtain the polygon/face
/// intersection of a plane and a 3DBox can be a polygon: a triangle, rectangle, pentagon or a hexagon (6).
/// NOTE1: we drop very close vertices of the polygon <0.01m
/// NOTE2: faces of the 3Dbox DONOT need to be a closed polygon (0-1-2-3)
/// USAGE: use this to create the face/polygon of an input plane/segment which is bounded by the 3D bbox of the data
// WHY int polygon_number is required?
bool Intersect_Plane_3DBoxFaces(LineTopologies box3d_faces, const ObjectPoints &box3d_vertices, //input
                           const Plane &plane, int polygon_number, //input
                           LineTopologies &polygon_edges, ObjectPoints &polygon_vertices, bool verbose) //output
{
    //bool intersection = false;
    LineSegments3D line_segments; //makes a triangle, rectangle, pentagon or a hexagon (3-6 vertices)
    // TODO: return false if box is empty or is less than 4 vertices
    for(auto &face_edges : box3d_faces){ // cuboid has 6 faces and each face has 4 edges
        //cout << "Box Face: " << ++count << endl; //DEBUG
        // fetch the exact vertices for this face
        ObjectPoints face_vertices = GetCorresponding_vertices(box3d_vertices, face_edges);
        LineSegment3D linesegment;
        if(Intersect_Plane_3DRectnagle(face_edges, face_vertices, plane, linesegment, verbose)){
            /// create new vertices and new topology,
            /// check if vertices are the same(within an epsilon dist) use one of them (later RemoveDoublePoints())
            line_segments.push_back(linesegment);
            //intersection = true; // there is no need to return something here
        }
    }
    if(line_segments.empty()) return false;
    LineTopologies polygon_edges_tmp; ObjectPoints polygon_vertices_tmp;
    line_segments.PointsWithTopology(polygon_vertices_tmp, polygon_edges_tmp, true);
    if(polygon_vertices_tmp.empty() || polygon_edges_tmp.empty()) return false;

    /// because linesegments have common vertices on the cuboid surface, the duplicates should be removed
    polygon_vertices_tmp.RemoveDoublePoints(polygon_edges_tmp, 0.01);

    /// this renumber objectpoints after removing duplicates
    polygon_edges_tmp.ReNumber(polygon_vertices_tmp); // we do it with RenumberTopology_and_Vertices()

    /// now we should make a polygon from edges because some edges maybe are not ordered to form a polygon
    LineTopology polygon = ReOrderTopology(polygon_edges_tmp); // this doesn't change any point number, only order to make the polygon

    /// renumber the polygon and vertices to 0-1-2-3-...
    LineTopologies renumbered_e; ObjectPoints renumbered_v;
    RenumberTopology_and_Vertices(polygon, polygon_vertices_tmp, renumbered_e, renumbered_v);
    polygon_vertices = renumbered_v;
    polygon_edges = renumbered_e;

    return true;
}

/// reorder the topology to make sure a polygon is created
/// USAGE: in Intersect_Plane_3DBoxFaces() to make the clipping polygon
LineTopology ReOrderTopology(LineTopologies edges){
    LineTopology ordered_edges;
    /// first add the first edge to the new ordered topology
    LineTopology this_edge = edges[0];
    ordered_edges.push_back(this_edge[0].Number());
    ordered_edges.push_back(this_edge[1].Number());
    for(auto it=edges.begin(); it!=edges.end(); it++){
        int begin_num = this_edge[0].Number();
        int end_num = this_edge[1].Number();
        for (auto &e : edges){ /// here there is some repetition but because the number of vertices are not many we don't mind.
            if(e[0].Number() == end_num){
                ///then add the other end of this edge to the ordered list
                if (find(ordered_edges.begin(), ordered_edges.end(), e[1].Number()) ==
                        ordered_edges.end()) ordered_edges.push_back(e[1].Number());
                if (std::next(it) == edges.end())
                    ordered_edges.push_back(e[0].Number()); // only for the last edge
                /// update the current edge
                this_edge = e;
            }
            if(e[1].Number() == end_num){
               /// first make sure it is not the same edge
               if(e[0].Number() != begin_num){
                   if (find(ordered_edges.begin(), ordered_edges.end(), e[0].Number()) ==
                           ordered_edges.end()) ordered_edges.push_back(e[0].Number());
                   if (std::next(it) == edges.end())
                       ordered_edges.push_back(e[1].Number()); // only for the last edge
                   /// update the current edge
                   e.RevertNodeOrder(); // flip begin and end
                   this_edge = e;
               }
            }
        }
    }
//    cout << "New List: ";
//    for(auto &i : ordered_edges) cout << i.Number() << "-";
//    cout << endl;
    //LineTopologies reordered_topo;
    //reordered_topo.push_back(ordered_edges);

    return ordered_edges;
}

/// renumber vertices and topology to 0-1-2-3- ...
/// USAGE: in Intersect_Plane_3DBoxFaces() to make the clipping polygon
void RenumberTopology_and_Vertices(LineTopology edges, ObjectPoints vertices,
                                   LineTopologies &new_edges, ObjectPoints &new_vertices){
    LineTopology new_topo;
    ObjectPoints points_tmp;
    PointNumberList pnumlist = edges.PointNumberListReference();
    bool isclosed=false;
    bool last_number_ofclosed=false;
    if(edges.IsClosed()) isclosed=true;
    int next_number=0;
    for(auto it = pnumlist.begin(); it != pnumlist.end(); it++){
         ///check if is the last number of a closed polygon
        if(std::next(it) == pnumlist.end() && isclosed){
            next_number=0; //then next number is the first as first number
            last_number_ofclosed=true;
        }
        auto vertex_it = vertices.GetPoint(*it);
        ObjectPoint vertex = *vertex_it;
        vertex.Number() = next_number; /// updating the number
        if(!last_number_ofclosed) new_vertices.push_back(vertex);
        new_topo.push_back(next_number); /// updating the topology
        next_number++;
    }
    new_edges.push_back(new_topo);
}

/// 1. fit planes to each segment in segments
/// 2. use Intersect_Plane_3DBoxFaces() to intersect the plane to the 3DBox
/// 3. returns the created polygons of each segment as a result of intersection
/// NOTE: later we use these polygons to intersect them to each other obtain new polygons/faces
/// USAGE: use it for intersecting planes/segments with the bounding box of the data
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
        if(verbose) cout << "plane number: " << plane.Number() << endl;
        if(Intersect_Plane_3DBoxFaces(box3d_faces, box3d_vertices, plane, seg_num,
                                           polygon_edges, polygon_vertices, verbose)){
            //polygon_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices_tmp.objpts");
            //polygon_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges_tmp.top", false);
            /// Store points and topology in one file
            ObjectPoint corner;
            Covariance3D cov3d;
            cov3d = Covariance3D(0, 0, 0, 0, 0, 0);
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
                corner = ObjectPoint(polygon_vertices[i], pnumber, cov3d);
                //polygon_vertices[i].NumberRef () = pnumber;
                //polygons_vertices.push_back(polygon_vertices[i]);
                polygons_vertices.push_back(corner);
                poly_tmp_edges.push_back(pnumber);  // making the linetopology
            }
            if(!poly_tmp_edges.IsClosed() && close)
                poly_tmp_edges.push_back(PointNumber(next_number)); // Close the polygon (for visualisation in pcm)
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
    // warning: this doesn't check whether the vertex is already exist in vertices_points
    for (int p = 0; p < pnumlist.size(); p++) { // these are points for 1 polygon in topologies
        ObjectPoint vertex = vertices.GetPoint(pnumlist[p].NumberRef())->ObjectPointRef();
        vertices_points.push_back(vertex);
    }
    return vertices_points;
}

Plane make_vertical_plane(double verticality_angle_threshold, Vector3D &normal,
                       LaserPoints &s, Plane &plane, bool verbose) {

    //char str_root[500];
    //auto *root = (char*) "/home/shayan/Drive_D/data/test/out/";
    //strcpy (str_root, root);
    if(verbose){
        Vector3D vecZ = Vector3D(0,0,1);
        double segment_angle = Angle (normal, vecZ) ; /// this gives the rotation angle respecting Z-axis
        printf ("segment number and angle: %d and %.1f\n",
                s[0].SegmentNumber (), segment_angle * 180.0 / PI);
    }
    /// if plane is almostVertical make it perfect vertical
    Plane vertPlane = plane; // this associate the plane points to vertPlane
    if(plane.IsVertical(verticality_angle_threshold * PI / 180)){ // ToRadian
        //Plane vertPlane (s.Mean(), Vector3D(normal.X(), normal.Y(), 0.0)); // this causes the plane loses the associated points from the segment
        // NOTE: just overriding the plane.Normal() with a new normal doesn't work, it should be recalculated like below
        Vector3D newNormal (normal.X(), normal.Y(), 0.0); // normal.Z=0 makes the plane vertical
        double len=newNormal.SqLength();
        if( len==0.0 )
        {
            vertPlane.Normal()=Vector3D( 0.0, 0.0, 1.0 );
            vertPlane.Distance() = s.Mean().Z();
        }
        vertPlane.Normal()   = newNormal/sqrt(len);                 // set the new normal
        vertPlane.Distance() = (newNormal/sqrt(len)).DotProduct( s.Mean() );   // set the dist from origin
        vertPlane.Number () = s[0].SegmentNumber ();            // set plane number
      /// DEBUG /// export the projected points on the new plane
//        LaserPoints projected_segment;
//        for (auto &p : s) /// for each point in segment
//            projected_segment.push_back (vertPlane.Project (p.Position3DRef ()));
//        strcpy (str_root, root);
//        projected_segment.Write(strcat (str_root, "proj_segment1.laser"), false);
        /// End of DEBUG
    }
    return vertPlane;
}

/// needs a test
bool Point_Inside_3DLineBounds(const Position3D &point, const LineSegment3D &lineseg3D, double margin)
{
  //Position3D minimum, maximum;
  Position3D beginPoint, endPoint;
  beginPoint = lineseg3D.BeginPoint().Position3DRef();
  endPoint   = lineseg3D.EndPoint().Position3DRef();

  double min_x = min(beginPoint.X(), endPoint.X());
  double min_y = min(beginPoint.Y(), endPoint.Y());
  double min_z = min(beginPoint.Z(), endPoint.Z());
  Position3D minimum (min_x, min_y, min_z);

  double max_x = max(beginPoint.X(), endPoint.X());
  double max_y = max(beginPoint.Y(), endPoint.Y());
  double max_z = max(beginPoint.Z(), endPoint.Z());
  Position3D maximum (max_x, max_y, max_z);

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





