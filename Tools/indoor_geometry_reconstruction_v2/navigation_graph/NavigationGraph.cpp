//
// Created by NikoohematS on 12-3-2019.
//

#include "NavigationGraph.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <map>
#include "KNNFinder.h"
#include "visualization_tools.h"


/// check if two TINMesh are the same
/// They are the same if their nodes are the same
bool MeshEqual(TINMesh& a, TINMesh& b)
{

    if(a.Nodes () == b.Nodes ()) return true;

    PointNumber *pNumber_a = nullptr, *pNumber_b=nullptr;
    for(int i=0; i<3; i++){
        if (pNumber_a[i].Number () == pNumber_b[i].Number ())
            return true;
    }
    return false;
}


/// we use this function in constrained delaunay to remove points inside the holes
LaserPoints Label_points_inside_polygons (LaserPoints &laserPoints, LineTopologies &polygons,
                                       ObjectPoints &vertices){

    LaserPoints lp_out;
    lp_out  = laserPoints;
    if(!lp_out.HasAttribute (LabelTag))
        lp_out.SetAttribute (LabelTag, 0);
    int i=0;
    for (auto &point : lp_out) point.SetPointNumber (i++);
    ///  find points inside the holes, because they make a mess with the order of points in TIN
    PointNumberList points_insidePolygon_List;
    LaserPoints points_inside_polygon;
    lp_out.Select(points_inside_polygon, vertices, polygons); // important function, output: points_inside_polygons
    for (auto &p : points_inside_polygon)
        points_insidePolygon_List.push_back (p.GetPointNumber ());

    ///now label(=2) them and filter out , label 1 is reserved for points near edges
    for(auto &point : lp_out){
        if(point.Attribute (LabelTag) != 0) continue; /// if it has another label we dont change it
        PointNumber pNumber;
        pNumber = point.GetPointNumber ();
        /// check if point number is in the points_insidePolygon_List then change its label to 2 if is not 1.
        auto point_number_it = find (points_insidePolygon_List.begin (), points_insidePolygon_List.end (), pNumber);
        if(point_number_it != points_insidePolygon_List.end ()){ /// if point is found in the list
            point.SetAttribute (LabelTag, 2);
        }
    }
    return lp_out;
}

/// check if points in the laserpoints are close to the edges then label them 1 otherwise 0.
/// the closeness distance is defined by dist_threshold. If planimetric is true the dist and
/// the line is calcualted in 2D. The output is labeled points.
LaserPoints Label_points_near_polygonEdges (LaserPoints &laserPoints, LineTopologies &polygons,
                            ObjectPoints &vertices, double dist_threshold, bool planimetric){
    LaserPoints lp_out;
    lp_out  = laserPoints;
    lp_out.SetAttribute (LabelTag, 0);
    int i=0;
    for (auto &point : lp_out) point.SetPointNumber (i++);
    PointNumberList labeled_pointsList; /// these are points near the edge
    /// for each polygon find the closest laserpoints near edges and label them to remove later
    int temp_num1=0; //debug
    int temp_num2=1; //debug
    LineTopologies edges_temp; //debug
    ObjectPoints vertices_temp; //debug
    for (auto &polygon : polygons){
        /// a polygon is like: 0 1 2 3 4 0
        /// check if the polygon is closed
        if(!polygon.IsClosed ()) {
            printf("WARNING: Polygon %d is not closed. \n", polygon.Number ());
            continue;
        }

        if (polygon.size () < 3) continue;
        LineTopology::iterator node1;
        //LineTopology::iterator node_start; /// this is the first point in the polygon
        //node_start = polygon.begin ();
        printf ("Polygon: # %d \n", polygon.Number ());
        for (node1 = polygon.begin (); node1 != polygon.end (); node1++){
            ObjectPoint *begin_point, *end_point;
            temp_num1++; //debug
            temp_num2++; //debug
            begin_point = vertices.GetPoint (node1->Number ());
/*            /// When we reach to the end of nodes -> not necessary becasue the last node is the first node in a closed polygon
            if ((node1 != polygon.end ()) && nextLocal (node1) == polygon.end ()){ /// the last iterator in the polygon
                end_point = vertices.GetPoint (node_start->Number ()); /// we close the polygon (last edge)
                printf("edge %d -- %d \n", node1->Number (), node_start->Number ());
            }*/
            /// fetch the second node of the edge
            LineTopology edge_temp; //debug
            LineTopology::iterator node2;
            node2 = node1 +1;
            if(node2 != polygon.end ()){
                end_point = vertices.GetPoint (node2->Number ());
            } else continue;
            printf("edge %d -- %d \n", node1->Number (), node2->Number ());

            /// reconstruct a 3Dline from two vertices of the edges in the polygon
            Line3D tempLine;
            tempLine = Line3D(begin_point->Position3DRef (), end_point->Position3DRef ());
            Line2D tempLine2D;
            if(planimetric){
                tempLine2D = Line2D(begin_point->Position2DOnly (), end_point->Position2DOnly ());
/*              //DEBUG
 *              ObjectPoint begin_p, end_p;
                begin_p = *begin_point;
                end_p = *end_point;
                begin_p.Number () = temp_num1;
                end_p.Number () = temp_num2;
                vertices_temp.push_back (begin_p);
                vertices_temp.push_back (end_p);
                edge_temp.push_back (PointNumber(temp_num1));
                edge_temp.push_back (PointNumber(temp_num2));
                edges_temp.push_back (edge_temp);*/
            }
            /// set label=1 to points near edges
            for (auto &p : lp_out){
                /*                if(p.GetPointNumber () == 10){
                    printf("pause\n");
                }*/
                /// check if p label is not already changed
                auto point_number_it = find (labeled_pointsList.begin (), labeled_pointsList.end (), p.GetPointNumber ());
                if(point_number_it != labeled_pointsList.end ()) continue ; // if point is already labeled then continue
                double dist_to_point;
                if(!planimetric) dist_to_point = tempLine.DistanceToPoint (p.Position3DRef ());
                if(planimetric) dist_to_point = tempLine2D.DistanceToPoint (p.Position2DOnly ());
                //printf("dist to point: %.2f \n", dist_to_point);
                if (dist_to_point < dist_threshold) {
                    p.SetAttribute (LabelTag, 1);
                    labeled_pointsList.push_back (p.GetPointNumber ());
                }
            }
        }
    }

    //for DEBUG
    //edges_temp.Write("E:/publication_data/Navigation_graph/Haaksbergen/out/edges_temp.top", false);
    //vertices_temp.Write("E:/publication_data/Navigation_graph/Haaksbergen/out/vertices_temp.objpts");
    //lp_out.Write("E:/publication_data/Navigation_graph/Haaksbergen/out/laserPoints_labeled.laser", false);
    return lp_out ;
}

/// bounds are 2D minimum_rectangles from segments (SegmentNumberTag), e.g. walls, in the laser points
/// inputs are: segmented_lp and the bound segmentNumber
/* NOTE: this function uses "LaserPoints Label_points_near_polygonEdges" to remove points near edges, the threshold
 * is hard coded. */
/// if "calculate_in2D" is false the TIN is calcualted in 3D.
/// "bounds_holes_vertices" are TIN vertices.
TIN Create_Constrained_Delaunay (LaserPoints &segmented_lp, int bound_segmentNumber,
         LineTopologies &bounds_holes_edges,  ObjectPoints &bounds_holes_vertices,
         ObjectPoints &holes_centers, ObjectPoints &all_points, char* root,
         double subsample_threhsold, bool planimetric){

    char rootStr[500];

    segmented_lp.RemoveDoublePoints();
    cout << "nr of points after RemoveDoublePoints: " << segmented_lp.size() << endl;

    LineTopology polygon_line;
    ObjectPoints objectpoints;
    bool is_bounding_segment;
    LaserPoints bound_laserpoints;
    int center_num=0;
    int polygon_num =0;

    vector <int>                 segment_numbers;
    vector <int>::iterator       segment_number;
    segment_numbers = segmented_lp.AttributeValues(SegmentNumberTag);
    for (segment_number = segment_numbers.begin(); segment_number != segment_numbers.end(); segment_number++) {
        polygon_num++;
        LaserPoints seg_laser_points;
        seg_laser_points = segmented_lp.SelectTagValue (SegmentNumberTag, *segment_number);

        LaserPoints lp;
        DataBoundsLaser db;

        lp = seg_laser_points;
        db = lp.DeriveDataBounds (0);
        is_bounding_segment = false;
        /// check if the lp is a bounding segment
        if (lp[0].SegmentNumber () == bound_segmentNumber) {
            is_bounding_segment = true;
        }
        if (is_bounding_segment) bound_laserpoints = lp; // we use it later for objectpoints to make TIN
        //ObjectPoint obj_pnt;
        //LineTopologies polygon_lines;

        /// Find the minimum enclosing rectangle
        lp.DeriveTIN ();
        if (!polygon_line.empty ()) polygon_line.erase (polygon_line.begin (), polygon_line.end ());
        lp.EnclosingRectangle (0.1, objectpoints, polygon_line); // corners and polygon_line are output of the function

        int next_number; /// the next_number is the number after the last number in the bounds_vertices file
        if (objectpoints.empty ()) next_number = 4; /// later next_number-4 =0
        else next_number = (objectpoints.end () - 1)->Number () + 1;


        /// add z values to 4 corners to make upper rectangle
        for (int i=next_number -4; i < next_number ; i++){
            if(planimetric){
                objectpoints[i].Z() = 0.0;  // lower rectangle
            } else
                objectpoints[i].Z() = db.Minimum ().GetZ();
            bounds_holes_vertices.push_back (objectpoints[i]);
        }
        /// If lp is not a bounding polygon then it is a hole object, add the center of holes to the holes_centers
        if(!is_bounding_segment){
            center_num++;
            /// calculate a point inside the hole rectangles
            ObjectPoint center_point, rectangle_corner1, rectangle_corner3; // two opposite corners of the rectangle
            rectangle_corner1 = objectpoints[next_number -4];
            rectangle_corner3 = objectpoints[next_number -2];
            Line3D line3d;
            line3d.AddPoint (Position3D(rectangle_corner1), false);
            line3d.AddPoint (Position3D(rectangle_corner3), true);

            Position3D center_pos;
            center_pos = line3d.CentreOfGravity ();

            center_point.X () = center_pos.X ();
            center_point.Y () = center_pos.Y ();
            center_point.Z () = center_pos.Z ();
            center_point.Number () = center_num;
            holes_centers.push_back (center_point);
            LaserPoints points_inside_hole;
            bound_laserpoints.Select(points_inside_hole, bounds_holes_vertices, bounds_holes_edges);
        }

        /// set different label for bounds and holes polygons
        if(is_bounding_segment){
            polygon_line.SetAttribute (BuildingPartNumberTag, 1); /// later we use this for removing points inside polygon
        } else{
            polygon_line.SetAttribute (BuildingPartNumberTag, 2);
        }
        polygon_line.Number () = polygon_num;
        bounds_holes_edges.push_back(polygon_line);

    }

    strcpy (rootStr,root);
    bounds_holes_vertices.Write(strcat(rootStr, "vertices.objpts"));

    ///Reduce the bound segment points for TIN, If knn=0, a TIN is used for reduction
    /// and the distance is only computed in 2D.
    int bound_size = bound_laserpoints.size ();
    bound_laserpoints.ReduceData (subsample_threhsold, 0, false);
    printf("bound segment size: %d -> %d \n", bound_size, bound_laserpoints.size ());
/*    strcpy (rootStr,root);
    bound_laserpoints.Write(strcat(rootStr, "bound_laserpoints.laser"), false);*/

    /// label(=1) and filter out points near the edges
    LaserPoints labeled_lPoints;
    double point_near_edge_dist;
    //point_near_edge_dist = subsample_threhsold / 4;
    point_near_edge_dist = 0.10;
    labeled_lPoints = Label_points_near_polygonEdges (bound_laserpoints, bounds_holes_edges,
            bounds_holes_vertices, point_near_edge_dist, true); /// this should be always in 2D

    /// select the hole polygons to select points inside them
    LineTopologies hole_polygons;
    for(auto &polygon : bounds_holes_edges) {
        if (polygon.Attribute (BuildingPartNumberTag) == 2)
            hole_polygons.push_back (polygon);
    }

     ///  find points inside the holes, because they make a mess with the order of points in TIN
    LaserPoints labeled_lPoints2;
    labeled_lPoints2 = Label_points_inside_polygons (labeled_lPoints, hole_polygons, bounds_holes_vertices);

    strcpy (rootStr,root);
    labeled_lPoints2.Write(strcat(rootStr, "labeled_lPoints.laser"), false);

    /// select points which aren't near the edges for TIN and are not inside the polygon. Then their label should be 0
    LaserPoints filtered_points;
    filtered_points = labeled_lPoints2.SelectTagValue (LabelTag, 0);

    /// convert the bound segment points to object points starting from the last point of "bounds_and_holes vertices"
    Covariance3D          covariance;
    int                   num;
    //num = (all_objectpoints.end() - 1)->Number() + 1;
    ObjectPoints          bound_points;
    LaserPoints::const_iterator laserpoint;
    PointNumber           number;
    /// num is initialized by the last number in bounds and holes. Centers are a separate objectpoint file
    for (laserpoint=filtered_points.begin(), num=(bounds_holes_vertices.end() - 1)->Number() + 1;
         laserpoint!=filtered_points.end(); laserpoint++, num++) {
        LaserPoint p;
        p = *laserpoint;
        if(planimetric) p.Z () = 0.0;

        number.Number() = num;
        bound_points.push_back(p.ConstructObjectPoint(number,covariance));
    }

    /// add bound_points to bounds_and_holes vertices
    bounds_holes_vertices.insert(bounds_holes_vertices.end (), bound_points.begin (), bound_points.end ());

    //bounds_holes_vertices.RemoveDoublePoints(bounds_holes_edges, 0.01);
    //bounds_holes_edges.ReNumber(bounds_holes_vertices, 0, 0); /// this may change the order of numbering

    /* VERY important function*/
    TIN tin;
    tin = bounds_holes_vertices.Triangulate (bounds_holes_edges, holes_centers); /// The main function
    printf ("TIN size: %d\n", static_cast<int>(tin.size ()));

    LineTopologies map_tin_lines;
    map_tin_lines = LineTopologies(tin);

    strcpy (rootStr,root);
    map_tin_lines.Write(strcat (rootStr, "map_tin_lines.top"), false);
    strcpy (rootStr,root);
    bounds_holes_vertices.Write(strcat(rootStr, "map_tin_points.objpts")); /// this contains TIN points and the order should not be modifed
    /// otherwise the TIN Edges after importing in pcm are not correct. The order is polygons+laserpoints without centers

    /// add centers and bounds_holes_vertices into one file called all_points and store
    all_points = bounds_holes_vertices;
    int center_new_number;
    center_new_number = (all_points.end() - 1)->Number(); /// last number
    for (auto &cnt_p : holes_centers){
        center_new_number++; /// this is next_number
        cnt_p.Number () = center_new_number;
        all_points.push_back (cnt_p);
    }

    return tin;
}

/// calculate the centroid of triangles
LaserPoints Create_Triangle_Centroids (TIN &tin, ObjectPoints &vertices){

    LaserPoints triangle_centers;
    int triangle_cnt=0;
    for(auto &mesh : tin){
        triangle_cnt++;
        PointNumber *meshNumber;
        meshNumber = mesh.Nodes ();
        //printf ("Mesh nodes: %d -> %d %d %d \n", meshNumber->Number (), /// meshnumber is suspicious, becasue it is sometimes repeated
        //        meshNumber[0].Number (), meshNumber[1].Number (), meshNumber[2].Number ());

        /// getting the centroid of the triangle
        int num1, num2, num3;
        num1 = meshNumber[0].Number ();
        num2 = meshNumber[1].Number ();
        num3 = meshNumber[2].Number ();

        ObjectPoint *p1, *p2, *p3;
        p1 = vertices.GetPoint (num1);
        p2 = vertices.GetPoint (num2);
        p3 = vertices.GetPoint (num3);

        Position3D pos1, pos2, pos3;
        pos1 = p1->Position3DRef ();
        pos2 = p2->Position3DRef ();
        pos3 = p3->Position3DRef ();

        Position3D triangle_centroid;
        triangle_centroid.X () = (pos1.X () + pos2.X () + pos3.X ()) / 3;
        triangle_centroid.Y () = (pos1.Y () + pos2.Y () + pos3.Y ()) / 3;
        triangle_centroid.Z () = (pos1.Z () + pos2.Z () + pos3.Z ()) / 3;

        triangle_centers.push_back (triangle_centroid);
        //printf ("Triangle: %d ", triangle_cnt);
    }
    printf ("# Triangle: %d \n", triangle_cnt);
    //printf ("\n");

    return triangle_centers;
}

/// calculate the centroid of triangles and then connect neighbor centroids
/// Output1: the output is a graph in the form of a pair of std::pair<ObjectPoints, LineTopologies> &centroids_Graph)
/// Output2: LaserPoints of centroids

///
/// \param tin
/// \param vertices
/// \param centroids_Graph
/// \return
LaserPoints Create_TIN_Centroids_Graph (TIN &tin, ObjectPoints &vertices,
                    std::pair<ObjectPoints, LineTopologies> &centroids_Graph){

    LaserPoints triangle_centers;
    //std::pair<PointNumber, Position3D> tin_centroids_pair;
    //std::map<PointNumber, ObjectPoint> tin_centroids_map; /// this map keeps the mesh number and its corresponding center objpoint
    std::map<int, ObjectPoint> tin_centroids_map;
    std::map<int, TINMesh> mesh_map;
    PointNumber centroid_pointNumber = 0;
    Covariance3D cov(0, 0, 0, 0, 0, 0);
    printf("TIN HighestNodeNumber: %d \n", (tin.HighestNodeNumber ()).Number ()) ;
    int mesh_counter=0;
    for(auto &mesh : tin){

        /// make a map of mesh and its counter (ID)
        mesh_map.insert(std::pair<int, TINMesh> (mesh_counter, mesh));

        PointNumber *meshNumber;
        meshNumber = mesh.Nodes ();
        printf ("Mesh nodes: %d -> %d %d %d \n", mesh_counter,
                meshNumber[0].Number (), meshNumber[1].Number (), meshNumber[2].Number ());

        /// getting the centroid of the triangle
        int num1, num2, num3;
        num1 = meshNumber[0].Number ();
        num2 = meshNumber[1].Number ();
        num3 = meshNumber[2].Number ();

        ObjectPoint *p1, *p2, *p3;
        p1 = vertices.GetPoint (num1);
        p2 = vertices.GetPoint (num2);
        p3 = vertices.GetPoint (num3);

        Position3D pos1, pos2, pos3;
        pos1 = p1->Position3DRef ();
        pos2 = p2->Position3DRef ();
        pos3 = p3->Position3DRef ();

        Position3D triangle_centroid;
        triangle_centroid.X () = (pos1.X () + pos2.X () + pos3.X ()) / 3;
        triangle_centroid.Y () = (pos1.Y () + pos2.Y () + pos3.Y ()) / 3;
        triangle_centroid.Z () = (pos1.Z () + pos2.Z () + pos3.Z ()) / 3;

        /// we construct a objpoint from the centroid
        LaserPoint p(triangle_centroid);
        ObjectPoint centroid_objpnt;
        centroid_objpnt = p.ConstructObjectPoint (centroid_pointNumber, cov);

        //tin_centroids_map.insert (std::pair <PointNumber, ObjectPoint> (meshNumber->Number (), centroid_objpnt));
        tin_centroids_map.insert (std::pair <int, ObjectPoint> (mesh_counter, centroid_objpnt));
        triangle_centers.push_back (triangle_centroid);
        //printf ("Triangle: %d ", triangle_cnt);
        centroid_pointNumber++;
        mesh_counter++;
    }
    printf ("# Triangle: %d \n", mesh_counter);
    //printf ("\n");

    /// DEBUG: print the map to see the output
/*    for(auto &m : tin_centroids_map){
        printf("Mesh Number: %d, Mesh Center: %d \n", (m.first).Number (), (m.second).Number ());
    }*/

    LineTopologies centroidsGraph_edges;
    ObjectPoints centroidsGraph_nodes;
    /// loop again through the meshes and connect neighbor centroids
    vector <int> visited_centroid;
    int mesh_counter_key=0;
    for (auto &mesh : tin){
        /// get mesh neighbors

/*        auto meshKey_it = mesh_map.find(mesh_counter_key);
        if(meshKey_it != mesh_map.end ()){
            /// look for this mesh inside the centroid map using the key
                //TODO find this mesh in the mesh_map
            // Then find the centroid of its nb in the other map
            for(auto &m : tin_centroids_map){
                //TODO find this mesh in Centroid map
            }
        }*/

        auto mesh_it = tin_centroids_map.find((mesh_counter_key));
        if(mesh_it != tin_centroids_map.end ()){
            ObjectPoint centroid;
            centroid = mesh_it->second;
            centroidsGraph_nodes.push_back (centroid);
            PointNumber centroid_pNumber;
            centroid_pNumber = centroid.Number ();
            /// we add this node to the visited list, so later we dont connect through neighbor edges again.
            visited_centroid.push_back (centroid_pNumber.Number ());
            MeshNumber *nb_meshes;
            nb_meshes = mesh.Neighbours ();
            LineTopology edge;
            edge.push_back (centroid_pNumber);
            int nbMesh;
           //bool isEdge = false;
            /// get the neighbor meshes, there are less than 3 nb always.
            for(int i=0; i<3; i++){
                /// neighbor mesh numbers
                nbMesh = nb_meshes[i].Number ();
                /// if nb mesh is -1 (empty) go to the next one
                if(nbMesh == -1) continue;
                /// find the centroid in the tin_centroids_map
                auto nbMesh_it = tin_centroids_map.find(nbMesh);
                if(nbMesh_it != tin_centroids_map.end()){
                    ObjectPoint nbMesh_pos;
                    nbMesh_pos = nbMesh_it->second;
                    PointNumber nbCentroid_pNumber;
                    /// if the centroid is already visited, the edges are established, not necessary to add again.
                    if ( std::find(visited_centroid.begin(), visited_centroid.end(), nbCentroid_pNumber.Number ())
                            != visited_centroid.end() )
                        continue;
                    /// add nb centroid to the edge and make the edge
                    edge.push_back (nbCentroid_pNumber);
                    centroidsGraph_edges.push_back (edge);
                    //isEdge = true;
                }
            } /// end of three neighbors

        }
        mesh_counter_key++;
    }

    centroids_Graph.first = centroidsGraph_nodes;
    centroids_Graph.second = centroidsGraph_edges;

    return triangle_centers;
}

/* The result of this function is the same as "LaserPoints Create_Triangle_Centroids ()" */
/// This is a planimetric calculation, intersecting two 2D median lines. However, if Z is required,
/// the centroid Z from three vertices is calculated and added to the median centroid. Otherwise Z is zero.
LaserPoints Create_Triangle_Medians_Intersection (TIN &tin, ObjectPoints &vertices, bool calculate_Z){

    LaserPoints triangle_centers;
    int triangle_cnt=0;
    PointNumber pointNumber=0;
    for(auto &mesh : tin){
        triangle_cnt++;
        pointNumber++;
        PointNumber *meshNumber;
        meshNumber = mesh.Nodes ();
        //printf ("Mesh nodes: %d -> %d %d %d \n", meshNumber->Number (), /// meshnumber is suspicious, becasue it is sometimes repeated
        //        meshNumber[0].Number (), meshNumber[1].Number (), meshNumber[2].Number ());

        ObjectPoint *p1, *p2, *p3;
        p1 = vertices.GetPoint (meshNumber[0].Number ());
        p2 = vertices.GetPoint (meshNumber[1].Number ());
        p3 = vertices.GetPoint (meshNumber[2].Number ());

        Position3D pos1, pos2, pos3;
        pos1 = p1->Position3DRef ();
        pos2 = p2->Position3DRef ();
        pos3 = p3->Position3DRef ();

        /// calculate two edges of the trinagle
        Line3D tempLine3D_23, tempLine3D_13;
        tempLine3D_13.AddPoint (pos1, false);
        tempLine3D_13.AddPoint (pos3, true);

        tempLine3D_23.AddPoint (pos2, false);
        tempLine3D_23.AddPoint (pos3, true);

        /// calculate the mid points
        Position3D mid_pos23, mid_pos13;
        mid_pos23 = tempLine3D_23.CentreOfGravity ();
        mid_pos13 = tempLine3D_13.CentreOfGravity ();

        /// make the median from pos1 to the middle of opposite edge
        Line2D median1, median2;
        median1 = Line2D(pos1.Position2DOnly (), mid_pos23.Position2DOnly ());
        median2 = Line2D(pos2.Position2DOnly (), mid_pos13.Position2DOnly ());

        Position2D triangle_centroid2D;
        Intersection2NonParallelLines(median1, median2, triangle_centroid2D);

        Position3D triangle_centroid3D;
        /// add z value
        if(calculate_Z){
            double triangle_centroid_Z;
            triangle_centroid_Z = (pos1.Z () + pos2.Z () + pos3.Z ()) / 3;
            /// make the 3D centroid
            triangle_centroid3D.X () = triangle_centroid2D.X ();
            triangle_centroid3D.Y () = triangle_centroid2D.Y ();
            triangle_centroid3D.Z () = triangle_centroid_Z;
        } else{
            triangle_centroid3D.X () = triangle_centroid2D.X ();
            triangle_centroid3D.Y () = triangle_centroid2D.Y ();
            triangle_centroid3D.Z () =0.0;
        }

        triangle_centers.push_back (triangle_centroid3D);
       // printf ("Triangle: %d ", triangle_cnt);
    }
    printf ("# Triangles: %d \n", triangle_cnt);
    //printf ("\n");

/*    TINEdges tinEdges;
    tinEdges = TINEdges(tin);
    //tinEdges.VRML_Write (file);
    TINEdges::const_iterator        edgeset;
    int                             point_number;
    PointNumberList::const_iterator nbnode;
    vertices.Sort ();
    //point_number = vertices[0].Number ();
    /// loop in all tinedges
    for (point_number= vertices.begin ()->Number (), edgeset=tinEdges.begin();
            edgeset!=tinEdges.end(); point_number++, edgeset++){
        for (nbnode=edgeset->begin(); nbnode!=edgeset->end(); nbnode++){
            if (point_number < nbnode->Number()){  // Only check each edge once
                //fprintf(vrml, "IndexedLineSet { coordIndex [%i,%i,-1] }\n",
                //        point_number, nbnode->Number());
                Position2D pos1, pos2, pos_midpoint;
                pos1 = (vertices.GetPoint (point_number))->Position2DOnly ();
                pos2 = (vertices.GetPoint (nbnode->Number ()))->Position2DOnly ();
                pos_midpoint.X () = (pos1.X () + pos2.X ()) /2;
                pos_midpoint.Y () = (pos1.Y () + pos2.Y ()) /2;
                //Line3D edge3d;
                //edge3d = Line3D(pos1, pos2);
                Line2D line2D, line_perpBisecrot;
                line_perpBisecrot = line2D.PerpendicularLine (pos_midpoint);
                /// get the neighbor Edges or TINs
            }

        }

    }*/
    return triangle_centers;
}

void Triangulate_test(ObjectPoints &points_and_vertices, LineTopologies &edges, ObjectPoints &centers, char*root){

    char rootStr[500];
    ObjectPoint p1, p2, p3, p4, p5, p6 , p7, p8, p9, p10, p11, p12, p13, p14, p15;
   /// bound polygon
    p1 = ObjectPoint(1.0, 1.0, 0.0, 0, 0,0,0,0,0,0);    points_and_vertices.push_back (p1);
    p2 = ObjectPoint(2.0, 8.0, 0.0, 1, 0,0,0,0,0,0);    points_and_vertices.push_back (p2);
    p3 = ObjectPoint(12.0, 9.0, 0.0, 2, 0,0,0,0,0,0);   points_and_vertices.push_back (p3);
    p4 = ObjectPoint(11.0, 2.0, 0.0, 3, 0,0,0,0,0,0);    points_and_vertices.push_back (p4);
    /// hole
    p5 = ObjectPoint(4.0, 4.0, 0.0, 4, 0,0,0,0,0,0);    points_and_vertices.push_back (p5);
    p6 = ObjectPoint(5.0, 6.0, 0.0, 5, 0,0,0,0,0,0);    points_and_vertices.push_back (p6);
    p7 = ObjectPoint(9.0, 7.0, 0.0, 6, 0,0,0,0,0,0);    points_and_vertices.push_back (p7);
    p8 = ObjectPoint(8.5, 5.0, 0.0, 7, 0,0,0,0,0,0);    points_and_vertices.push_back (p8);

    /// some extra points
    p9 = ObjectPoint(6.0, 3.0, 0.0, 8, 0,0,0,0,0,0);  points_and_vertices.push_back (p9);
    p10 = ObjectPoint(3.0, 5.0, 0.0, 9, 0,0,0,0,0,0); points_and_vertices.push_back (p10);
    p11 = ObjectPoint(6.5, 7.5, 0.0, 10, 0,0,0,0,0,0);  points_and_vertices.push_back (p11);
    p12 = ObjectPoint(10.0, 6.0, 0.0, 11, 0,0,0,0,0,0); points_and_vertices.push_back (p12);
    p13 = ObjectPoint(7.0, 11.0, 0.0, 12, 0,0,0,0,0,0); points_and_vertices.push_back (p13);

    /// hole center
    p14 = ObjectPoint(7.0, 5.5, 0.0, 13, 0,0,0,0,0,0);   //points_and_vertices.push_back (p14);

    centers.push_back (p14);

    LineTopology bound_edges;
    bound_edges.push_back (p1.NumberRef ());
    bound_edges.push_back (p2.NumberRef ());
    bound_edges.push_back (p3.NumberRef ());
    bound_edges.push_back (p4.NumberRef ());
    bound_edges.push_back (p1.NumberRef ()); // close the polygon
    //bound_edges.MakeCounterClockWise (points_and_vertices); // doesn't matter in triangulation result

    LineTopology hole_edges;
    hole_edges.push_back (p5.NumberRef ());
    hole_edges.push_back (p6.NumberRef ());
    hole_edges.push_back (p7.NumberRef ());
    hole_edges.push_back (p8.NumberRef ());
    hole_edges.push_back (p5.NumberRef ()); // close the polygon
    //hole_edges.MakeCounterClockWise (points_and_vertices);

    edges.push_back (bound_edges);
    edges.push_back (hole_edges);



    TIN tin;
    tin = points_and_vertices.Triangulate (edges, centers);

    LineTopologies map_tin_lines;
    map_tin_lines = LineTopologies(tin);

    strcpy (rootStr,root);
    map_tin_lines.Write(strcat (rootStr, "map_tin_lines.top"), false);
    strcpy (rootStr,root);
    points_and_vertices.Write(strcat(rootStr, "map_tin_points.objpts"));
    strcpy (rootStr,root);
    centers.Write(strcat(rootStr, "centers.objpts"));

    strcpy (rootStr,root);
    edges.Write(strcat (rootStr, "edges.top"), false);

/*    FILE *file;
    strcpy (rootStr,root);
    file = fopen(strcat(rootStr,"TIN.vrml"), "w");
    tin.VRML_Write (file);*/
    strcpy (rootStr,root);
    tin.Write(strcat (rootStr, "TIN.tin"));

    /// convert obj points to laser points
    LaserPoints laserpoints;
    for (auto &p : points_and_vertices) laserpoints.push_back (p.pos ());

    strcpy (rootStr,root);
    laserpoints.Write(strcat (rootStr, "laserpoints.laser"), false);
}

