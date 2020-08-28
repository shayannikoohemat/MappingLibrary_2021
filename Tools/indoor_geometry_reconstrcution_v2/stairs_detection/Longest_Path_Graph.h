//
// Created by NikoohematS on 18-2-2019.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_LONGEST_PATH_GRAPH_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_LONGEST_PATH_GRAPH_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_LONGEST_PATH_GRAPH_H


#include <bits/stdc++.h>
using namespace std;

// This class represents an undirected graph using adjacency list
class Graph
{
    int V;              // No. of vertices
    list<int> *adj;     // Pointer to an array containing adjacency lists
    bool isCyclicUtil(int v, bool visited[], int parent);
public:
    Graph(int V);              // Constructor
    void addEdge(int v, int w);// function to add an edge to graph
    void longestPathLength(int start_node);  // prints longest path of the tree
    //pair<pair<int, int>, vector<int>> bfs(int u); // function returns maximum distant
    pair<int, int> bfs(int u, vector<int> &pred);  // function returns maximum distance node
    // from u with its distance and predecessors
    bool isCyclic();   // returns true if there is a cycle

    //void dfs(int u);
};



