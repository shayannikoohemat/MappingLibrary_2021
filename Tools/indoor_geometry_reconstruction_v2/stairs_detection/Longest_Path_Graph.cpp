//
// Created by NikoohematS on 18-2-2019.
//

#include <stdio.h>
#include <vector>
#include <algorithm>
#include <vector>
#include <queue>
#include <iostream>
using namespace std;

#include "Longest_Path_Graph.h"
//source: https://www.geeksforgeeks.org/longest-path-undirected-tree/
//https://stackoverflow.com/questions/54768946/printing-the-longest-path-in-an-undirected-graph/54770406#54770406


Graph::Graph(int V)
{
    this->V = V;
    adj = new list<int>[V];
}

void Graph::addEdge(int v, int w)
{
    adj[v].push_back(w);    // Add w to v’s list.
    adj[w].push_back(v);    // Since the graph is undirected
}

//  method returns farthest node and its distance from node u
pair<int, int> Graph::bfs(int u, vector<int> &pred) // Both pred and pair are return values of the BFS
{
    //  mark all distance with -1
    vector<int> dis(V, -1);  //int dis[V];
    pred.reserve (V);
    //int pred[V];
    //memset(dis, -1, sizeof(dis));

    queue<int> q;
    q.push(u);

    //  distance of u from u will be 0
    dis[u] = 0;
    pred[u] = {u}; /// predecessor

    while (!q.empty())
    {
        int t = q.front();
        q.pop();
        //cout << "current node:" << t << endl;

        //  loop for all adjacent nodes of node-t
        for (auto it = adj[t].begin(); it != adj[t].end(); it++)
        {
            int v = *it;
            //cout << "adjacent node:" << v << endl;
            // push node into queue only if
            // it is not visited already
            if (dis[v] == -1)  /// if color white
            {
                q.push(v);

                // make distance of v, one more
                // than distance of t
                dis[v] = dis[t] + 1;
                //cout << "level of adjacent node:" << dis[v] << endl;
                /// make the array of parents
                pred[v] = t; // store the predecessor of v
                //cout << "parent of adjacent node:" << t << endl;
            }
        }
    }

    int maxDis = 0;
    int nodeIdx;

    //  get farthest node distance and its index
    for (int i = 0; i < V; i++)
    {
        if (dis[i] > maxDis)
        {
            maxDis = dis[i];
            nodeIdx = i;
        }
    }
   return make_pair (nodeIdx, maxDis);
}

//  method prints longest path of given tree
void Graph::longestPathLength(int start_node)
{
    pair<int, int> t1, t2;
    vector<int> pred1, pred2;

    // first bfs to find one end point of
    // longest path
    t1 = bfs(start_node, pred1);

    //  second bfs to find actual longest path
    //t2 = bfs(t1.first);
    t2 = bfs(t1.first, pred2);

    cout << "Longest path is from " << t1.first << " to "
         << t2.first << " of length " << t2.second << endl;

    // Backtrack from t2.first to t1.first
    cout << "Path: " ;
    for (int t = t2.first; t != t1.first; t = pred2[t])
            cout << t << " ";
    cout << t1.first << endl;
}

// A recursive function that uses visited[] and parent to detect
// cycle in subgraph reachable from vertex v.
//https://www.geeksforgeeks.org/detect-cycle-undirected-graph/
bool Graph::isCyclicUtil(int v, bool visited[], int parent)
{
    // Mark the current node as visited
    visited[v] = true;

    // Recur for all the vertices adjacent to this vertex
    list<int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i)
    {
        // If an adjacent is not visited, then recur for that adjacent
        if (!visited[*i])
        {
            if (isCyclicUtil(*i, visited, v))
                return true;
        }

            // If an adjacent is visited and not parent of current vertex,
            // then there is a cycle.
        else if (*i != parent)
            return true;
    }
    return false;
}

// Returns true if the graph contains a cycle, else false.
bool Graph::isCyclic()
{
    // Mark all the vertices as not visited and not part of recursion
    // stack
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++)
        visited[i] = false;

    // Call the recursive helper function to detect cycle in different
    // DFS trees
    for (int u = 0; u < V; u++)
        if (!visited[u]) // Don't recur for u if it is already visited
            if (isCyclicUtil(u, visited, -1))
                return true;

    return false;
}

// use BFS for cycle
///https://www.geeksforgeeks.org/detect-cycle-in-an-undirected-graph-using-bfs/