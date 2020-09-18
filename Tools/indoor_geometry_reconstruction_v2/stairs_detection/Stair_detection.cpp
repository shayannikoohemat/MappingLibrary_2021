//
// Created by NikoohematS on 13-2-2019.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <map>
#include "Buffer.h"
#include "Buffers.h"
#include "indoor_reconstruction.h"
#include "post_processing.h"
#include "visualization_tools.h"
#include "AdjGraph.h"


struct MinimumRectangle{
    ObjectPoints rect_corners;
    LineTopology rect_edges;
    int rect_number{}; // e.g. segment number
    double rect_area{};
    int rect_pointSize{};
};


/// this function in a bigger point clouds find the points that are fitting to a stair ramp
/// a valid stair ramp should has an angle between 25 to 50 degrees and should not be smaller than 1 sq.meter.
LaserPoints Coarse_Stair_Search(LaserPoints &laserpoints, int min_ramp_size, double min_ramp_angle,
        double max_ramp_angle, double plane_inliers, char* root, bool do_segmentation){

    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    /// segmenting laserpoints
    if (do_segmentation){
        seg_parameter -> MaxDistanceInComponent()  = 0.3;
        seg_parameter -> SeedNeighbourhoodRadius() = 1.0;
        seg_parameter -> MaxDistanceSeedPlane()    = 0.13; // MaX Distance to the Plane
        seg_parameter -> GrowingRadius()           = 1.0;
        seg_parameter -> MaxDistanceSurface()      = 0.13;

        /// if segmentation crashes set the compatibility of generated exe to windows7
        printf("segmentation process... \n ");
        laserpoints.SurfaceGrowing(*seg_parameter);
    }

    /// make a list of segments
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(laserpoints, SegmentNumberTag);

    /// sort segments by size
    sort(segments_vec.begin (), segments_vec.end (), compare_lp_size); // descending

    char str_root[500];
    FILE *segments_infofile;
    strcpy (str_root,root);
    segments_infofile = fopen(strcat(str_root, "segments_infofile.txt"),"w");
    fprintf(segments_infofile, "segment_nr, segment_pointsize, centroid_height, segment_angle \n");

/*    /// steps for collecting inliers to the plane
    LaserPoints selected_points_to_plane;
    for (auto i=0; i < laserpoints.size (); i++) laserpoints[i].SetPointNumber (i);
    vector<int> pnumbers;*/

    LaserPoints valid_ramps_lp;
    for (auto &s : segments_vec){

        if(s.size () > min_ramp_size){
            /// fit a plane to the segment
            Plane plane;
            plane = s.FitPlane (s[0].SegmentNumber ());
            Vector3D normal;
            Vector3D z(0,0,1);
            normal = plane.Normal ();
            double angle;
            angle = Angle (normal, z) * 180.0 / PI;
            if (angle > 90.0) angle = 180.0 - angle;
            fprintf (segments_infofile, "%d,%d,%.2f, \n", s[0].SegmentNumber (), s.size (), angle);

            if (angle <= max_ramp_angle && angle >= min_ramp_angle){
                valid_ramps_lp.AddPoints (s);
                /// collect inliers to the plane
                /* slow process and bad results */
/*                for(auto &p : laserpoints){
                    int pnumber = p.GetPointNumber ();
                    if (find(pnumbers.begin (), pnumbers.end (), pnumber) !=pnumbers.end ()) continue;
                    if(fabs(plane.Distance (p.Position3DRef ())) <= plane_inliers) {
                        p.SegmentNumber () = s[0].SegmentNumber ();
                        selected_points_to_plane.push_back (p);
                        pnumbers.push_back (pnumber);
                    }
                }*/
            }
        }
    }
    strcpy (str_root,root);
    laserpoints.Write(strcat(str_root,"lp_segmented.laser"), false);

    //strcpy (str_root,root);
    //selected_points_to_plane.Write(strcat(str_root,"selected_points_to_plane.laser"), false);

    return valid_ramps_lp;
}

/* This function with the adj graph is not used, becasue the graph was complicated. Instead the LongestPathGraph is used*/
/// this function make a graph of segmented stair point clouds
void Stair_Modeling (LaserPoints &laserPoints, int min_segment_size, double dist_threshold,
        double angle_threshold, char *root, bool verbose){

    char str_root[500];
    strcpy (str_root, root);

    if (!laserPoints.HasAttribute (SegmentNumberTag)){
        printf ("Error: Laserpoints should have SegmentNumberTag! \n");
        EXIT_FAILURE;
    }

    /// make a list of segments
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(laserPoints, SegmentNumberTag);

    vector<int> segment_nr_vec;
    segment_nr_vec = laserPoints.AttributeValues (SegmentNumberTag);

    /// make a map of planes
    /// make a map of segment_numbers and segments,
    /// make a map of min_rectangles and their segmentNumbers
    /// fit a plane to each segment (if there is no plane) using PlaneFitting function and least square
    std::map<int, LaserPoints> segments_map;
    std::map<int, Plane> planes_map;
    std::map<int, MinimumRectangle> minRectangles_map;
    for(auto &s : segments_vec){
        int seg_nr = s[0].SegmentNumber ();
        if(s.HasAttribute (SegmentNumberTag) && s.size () > min_segment_size){
            /// make a map of segment number and its corresponding laserpoints
            segments_map.insert (std::pair<int, LaserPoints> (seg_nr, s));
            /// fit a plane to the segment points and make a map
            Plane plane;
            plane = s.FitPlane (seg_nr);
            plane.Number () = seg_nr;
            planes_map.insert (std::pair <int, Plane> (seg_nr, plane));
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
    sort(segments_vec.begin (), segments_vec.end (), compare_lp_size );


    Adj_Graph adj_graph;
    //Node node;
    //node.node_number = s[0].SegmentNumber ();
    //adj_graph.Nodes.push_back (node);

    /// compare segments two by two for checking the adjacency and making the graph
    FILE *statfile;
    strcpy (str_root,root);
    statfile = fopen(strcat(str_root, "adjacency_info.txt"),"w");
    fprintf(statfile, "seg1, seg2, angle, distance \n");
    //std::map<int, LaserPoints>::iterator seg_m_it1;
    vector<int> node_numbers;
    //Node node1, node2; /// these two should be defined outside the loop and initialize inside the loop
    for (auto seg_m_it1 = segments_map.begin (); seg_m_it1 != segments_map.end (); seg_m_it1++) {
        int seg_no1 = seg_m_it1->first;
        LaserPoints segment1 = seg_m_it1->second;
        Plane plane1;
        Node node1;
        /// if this node is already made in previous steps then fetch it
        if(std::find(node_numbers.begin(), node_numbers.end(), seg_no1) != node_numbers.end()){
            node1 = adj_graph.getNode (seg_no1);
        } else {
            node1.node_number = seg_no1;
        }

        auto plane1_it = planes_map.find (seg_no1);
        if (plane1_it != planes_map.end ()) {
            plane1 = plane1_it->second;
        }

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
            vector <double> dist_v;
            dist_v = segment1.Distance (segment2, 1.0);
            double distance=0.0;
            if (!dist_v.empty ()){
                distance = *min_element (dist_v.begin (), dist_v.end ());
            }

            double angle;
            /// calculate planes' normal
            Vector3D normal1, normal2;
            normal1 = (plane1).Normal();
            normal2 = (plane2).Normal();
            /// plane angle is calculated by angle between their normals
            angle = (acos(normal1.DotProduct(normal2) / (normal1.Length() * normal2.Length()))) * 180.0 / PI;

            /// making the graph based on the proximity
            if (distance <= dist_threshold){
                Node node2;
                if(std::find(node_numbers.begin(), node_numbers.end(), seg_no2) != node_numbers.end()){
                    node2 = adj_graph.getNode (seg_no2);
                } else {
                    node2.node_number = seg_no2;
                }
                node1.add_adjacent_Node (node2); /// here the list of adjacentNodes is correct
                node2.add_adjacent_Node (node1);

                /// make the edge
                Edge edge;
                //edge.make_edge (node1.node_number, node2.node_number);
                edge.angle_prop = angle;
                edge.distance_prop = distance;
                edge.edge_p.first = node1.node_number;
                edge.edge_p.second = node2.node_number;

                /// add nodes to the graph
                if(!(std::find(node_numbers.begin(), node_numbers.end(), node2.node_number) != node_numbers.end())){
                    adj_graph.addNode (node2);
                    node_numbers.push_back (node2.node_number);
                } else{
                    //update the list of adjacentNodes in adjGraph
                    adj_graph.updateNode (node2);
                }

                if(!(std::find(node_numbers.begin(), node_numbers.end(), node1.node_number) != node_numbers.end())){
                    adj_graph.addNode (node1);
                    node_numbers.push_back (node1.node_number);
                } else{
                    adj_graph.updateNode (node1);
                }
                /// add edge
                adj_graph.addEdge (edge);

            } else continue; /// if two segments are not nearby go to the next segment

        }
    }

    /// print the graph in DOT format
    adj_graph.PrintGraph (adj_graph);

    Edge e1;
    e1.edge_p.first = 11;
    e1.edge_p.second = 8;
    Node n1, new_node;
    n1 = adj_graph.getNode (11);
    new_node.node_number =8;
    adj_graph.updateNode (n1);
    adj_graph.updateNode (new_node);


    e1.weight = 1.0;
    adj_graph.Edges.push_back (e1);

    adj_graph.addEdge (e1);

    /// how to access an edge
/*    Edge e1;
    e1 = adj_graph.getEdge (27,24);
    cout << e1.edge_p.first << " and " << e1.edge_p.second << endl;
    cout << e1.angle_prop << endl;*/

    /// analysis the graph for the stairs pattern
    /// a stair with two steps should form a graph with at least 4 nodes and three edges, where the nodes make and
    ///angle of 90+-(angle_tolerance) degrees (e.g. 80.0 < angle < 100.0 degress). If such a graph is detected in the data,
    /// then the graph grows from both end nodes two form more steps until there is no step left.

    /// 1.first : Give weight 1.0 to the edges if the angle of their nodes is within the threhsold, otherwise weight 0.0
    for (auto &edge : adj_graph.Edges) {
        //if (edge.angle_prop >= 80.0 && edge.angle_prop <= 100.0){ /// this can be replaced with checking if the intersection line is horizontal
            // TODO Check if one node center is higher than the other one
            edge.weight = 1.0;
        //} else edge.weight = 0.0;
    }

    ///



    //for (auto &node : adj_graph.Nodes) node.visited = false;
    vector <int> path_list;
    map<int, vector<int>> path_w_map;
    int max_cost=0; /// max cost for a valid stair graph should be 1+1+1 =3 representing 3edges and 4nodes.


    vector <int> visited;
    //Node source = adj_graph.Nodes[0];
    Node source = adj_graph.getNode (13);
    visited.push_back (source.node_number);
    //source.visited = true; /// this doesn't update the node in the adjGraph
    path_list.push_back (source.node_number);
    printf("source: %d \n", source.node_number);
    std::stack<int> node_stack;
    node_stack.push (source.node_number);
    while(!node_stack.empty ()){
        int top_node;
        top_node = node_stack.top (); /// access the top item in the stack
        source = adj_graph.getNode (top_node); /// now source is the top node in the stack
        //source.visited = true; /// this is becasue in the adj graph visited is not updated
        node_stack.pop (); /// removes the top element
        if(!source.adjacent_nodes.empty ()){
            for (auto &adj_node : source.adjacent_nodes){
                Edge stair_edge;
                stair_edge = adj_graph.getEdge (source.node_number, adj_node);
                Node curr_adj_node;
                curr_adj_node = adj_graph.getNode (adj_node);
                /// if the current node is not visited
                if (!(std::find(visited.begin (), visited.end (), curr_adj_node.node_number) != visited.end ())) {
                    if (stair_edge.weight == 1.0) {
/*                      node_is_stair = true;
                        if (std::find(stairNodes.begin(), stairNodes.end(), adj_node) != stairNodes.end())
                            continue; /// if the value is already added continue to the next one
                        stairNodes.push_back (adj_node);*/
                        max_cost++;
                       // printf ("Max cost: %d \n", max_cost);
                        path_list.push_back (adj_node); /// add the adj node to the path

                       // printf("Adj Node: %d \n", adj_node);
                        node_stack.push (adj_node);
                        visited.push_back (curr_adj_node.node_number);

                    }
                }
            }
            // TODO make the path list
            path_w_map.insert (std::make_pair (max_cost, path_list));
            //max_cost  = 0;
           // path_list.clear ();
        }
    }


    for ( auto &m : path_w_map){
        for (auto &i : m.second){
            cout << "Path: " ;
            cout << " " << i << " -- ";
        }
        printf ("\n");
    }

/*    vector <int> stairNodes;
    for (auto &node : adj_graph.Nodes){
        bool node_is_stair = false;
        node.visited = true;
        if (!node.adjacent_nodes.empty ()) {
            for (auto &adj_node : node.adjacent_nodes){
                Edge stair_edge;
                stair_edge = adj_graph.getEdge (node.node_number, adj_node);

                if (stair_edge.angle_prop >= 80.0 && stair_edge.angle_prop <= 100.0){
                    node_is_stair = true;
                    if (std::find(stairNodes.begin(), stairNodes.end(), adj_node) != stairNodes.end())
                        continue; /// if the value is already added continue to the next one
                    stairNodes.push_back (adj_node);
                }

            }
        }
        if(node_is_stair){
            if (!(std::find(stairNodes.begin(), stairNodes.end(), node.node_number) != stairNodes.end())) /// avoid repeated segments
                stairNodes.push_back (node.node_number);
        }



    }*/

/*    LaserPoints stairSegments;
    for (auto &stair : stairNodes){
        /// find th segment in the segments_map with the segment number
        LaserPoints segment;
        auto seg_it = segments_map.find(stair);
        if (seg_it != segments_map.end ()){
            segment = seg_it ->second; /// this returns the segment in the map
        }
        stairSegments.AddPoints (segment);
    }
    strcpy (str_root,root);
    stairSegments.Write(strcat(str_root, "stairSegments.laser"), false);*/

}

