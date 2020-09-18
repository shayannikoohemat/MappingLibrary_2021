//
// Created by NikoohematS on 13-2-2019.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_ADJGRAPH_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_ADJGRAPH_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_ADJGRAPH_H


#include <iostream>
#include <vector>

/// this adj graph structure is not well designed and better not to use. Instead use Longest_Path_Graph.h/cpp

struct Node {
    int node_number{}; /// segment number
    std::vector<int> adjacent_nodes;
    void add_adjacent_Node(Node n){ adjacent_nodes.push_back (n.node_number); }
    bool visited;
};

struct Edge{

    std::pair<int , int > edge_p;
/*    void make_edge(int u, int v) {
        std::pair<int , int > edge = std::make_pair(u, v);
    }

    void make_edge(Node u, Node v) {
        std::pair<int , int > edge = std::make_pair(u.node_number, v.node_number);
    }*/

    //vector<int> edge;
    double angle_prop{};
    double distance_prop{};
    double weight{};
    int type_connection{}; // 1.wall-wall, 2.wall-ceiling, 3.wall-floor, ... depends on the intersection line is vertical or horizontal

};

struct Adj_Graph {
    std::vector<Node> Nodes;
    std::vector<Edge> Edges;
    void addNode(const Node &v){
        Nodes.push_back (v);
    }

    void addEdge(const Edge &e){
        Edges.push_back (e);
    }

    Node getNode (int n){
        Node node;
        for (auto &v : Nodes){
            if (v.node_number == n) {
                node = v;
            }
        }
        return node;
    }

    Edge getEdge (int u, int v){
        Edge edge;
        for (auto &e : Edges){
            if (e.edge_p.first == u && e.edge_p.second == v) edge = e;
            if (e.edge_p.first == v && e.edge_p.second == u) edge = e;
        }
        return edge;
    }

/*    void deleteNode (Node n){
        this->Nodes.erase(std::remove(this->Nodes.begin(), this->Nodes.end(), n.node_number), this->Nodes.end());
    }*/

    bool updateNode (Node n){
        /// first find the node using the nodenumber
        vector <int> node_numbers;
        for (auto &v : Nodes) node_numbers.push_back (v.node_number);
        auto it = std::find(node_numbers.begin(), node_numbers.end(), n.node_number);
        if(it != node_numbers.end ()){
            auto index = std::distance(node_numbers.begin(), it);
            Nodes.erase (Nodes.begin () + index);
            addNode (n);
            return true;
        } else
            return false;

    }

    /// printing in DOT Graph format
    void PrintGraph (Adj_Graph adj_graph){
        printf("\n PRINT the GRAPH in DOT format \n");
        cout << "graph adjGraph { " << endl;
        for (auto &e : adj_graph.Edges){

            cout << e.edge_p.first;
            cout << " -- " << e.edge_p.second;
            cout << " [distance = " << e.distance_prop << ", " ;
            cout <<  " angle = " << e.angle_prop << "];" ;
            printf("\n");
        }
        cout << "}" ;
        printf("\n");
    }

};