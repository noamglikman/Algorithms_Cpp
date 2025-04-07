//noamglikman1@gmail.com
#include <iostream>
#include "Algorithms.h"
#include "graph.h"
#include "Queue.h"
#include "UnionFind.h"
#include <limits>
#include <climits>
#include <unistd.h> 


namespace Graph{

    // DFS - Visit helper function for depth-first search
// This function recursively visits vertices starting from the given vertex v and builds a DFS tree.
// It marks the vertices with different colors during the traversal: 
//   0 - unvisited, 1 - visiting, 2 - visited. 
// Parameters:
//   - g: The graph being traversed.
//   - dfs_tree: The DFS tree being built.
//   - v: The current vertex to visit.
//   - color: Array to track the state (unvisited, visiting, visited) of vertices.
// Returns:
//   - A graph representing the DFS tree.
graph Algorithms::dfs_visit(graph& g, graph& dfs_tree, int v, int color[]) {
    // Mark the vertex v as visiting (color 1)
    color[v] = 1;
     // Check if the source vertex is within the valid range
     if (v < 0 || v >= g.getNumVertices()) {
        throw std::out_of_range("Vertex index out of range");
    }
    // Get the list of neighbors for the current vertex v
    NodeG* neighbor = g.getAdjList(v);

    // Traverse through all the neighbors of v
    while (neighbor != nullptr) {
        int adjVertex = neighbor->vertex;

        // If the neighbor vertex has not been visited, recursively visit it
        if (color[adjVertex] == 0) {
            // Add an edge from v to the adjacent vertex adjVertex in the DFS tree
            dfs_tree.addEdgeNotBoth(v, adjVertex, neighbor->weight); 
            // Recur for the adjacent vertex
            dfs_visit(g, dfs_tree, adjVertex, color);
            std::cout << "Visiting vertex " << v << std::endl;
            // Optionally print the DFS tree here
            //dfs_tree.print_graph();
        }

        // Move to the next neighbor
        neighbor = neighbor->next;
    }

    // Mark the vertex v as visited (color 2)
    color[v] = 2;

    // Return the DFS tree
    return dfs_tree;
}

// DFS - Depth-First Search on the graph starting from a source vertex.
// This function initializes DFS and constructs the DFS tree.
// It calls the dfs_visit helper function to visit all vertices reachable from the source vertex.
// Parameters:
//   - g: The graph to traverse.
//   - src: The source vertex to start the DFS.
// Returns:
//   - A graph representing the DFS tree.
graph Algorithms::dfs(graph& g, int src) {
   
    // Get the number of vertices in the graph
    int numVertices = g.getNumVertices();

    // If there are no vertices, return an empty graph
    if (numVertices == 0) return graph(0);  

    // Create an array to track the colors (states) of the vertices
    int* color = new int[numVertices];

    // Initialize all vertices as unvisited (color 0)
    for (int i = 0; i < numVertices; ++i) {
        color[i] = 0;
    }

    // Create an empty graph to represent the DFS tree
    Graph::graph dfs_tree(numVertices);

    // Start the DFS traversal from the source vertex
    dfs_visit(g, dfs_tree, src, color);

    // Clean up the color array after the traversal is complete
    delete[] color;

    // Return the constructed DFS tree
    return dfs_tree; 
}


    // BFS - Breadth-First Search on the graph starting from a given vertex.
    // This function performs BFS traversal and returns the BFS tree while also checking if the graph is connected.
    // Parameters:
    //   - g: The graph to traverse.
    //   - startNum: The starting vertex for BFS.
    //   - connected: A boolean pointer that indicates whether the graph is connected.
    // Returns:
    //   - A graph representing the BFS tree.
    graph Algorithms::bfs(graph& g, int startNum, bool* connected) {
    // Get the number of vertices in the graph
    int numVertices = g.getNumVertices();

    // Check if the starting vertex is within the valid range
    if (startNum < 0 || startNum >= numVertices) {
        throw std::out_of_range("Vertex index out of range");
    }

    // Get the adjacency list for the starting vertex
    NodeG* node = g.getAdjList(startNum);

    // Initialize priority counter (used for node order during BFS)
    int priarity = 0;
    
    // If the graph has no vertices, return an empty graph
    if (numVertices == 0) return graph(0);

    // Create arrays for coloring (visited/unvisited) and distance tracking
    int* color = new int[numVertices];   // 0 - unvisited, 1 - visiting, 2 - visited
    int* d = new int[numVertices];       // Stores the distance from the starting vertex

    // Initialize color and distance arrays
    for (int i = 0; i < numVertices; ++i) {
        color[i] = 0;  // Initially, all vertices are unvisited
        d[i] = std::numeric_limits<int>::max(); // Distance is initially set to infinity
    }

    // Set the distance of the starting vertex to 0
    d[startNum] = 0;

    // Create a queue for BFS traversal
    Queue qReg;
    // Initialize the BFS tree (graph) to store the BFS traversal result
    Graph::graph bfs_tree(numVertices);
    
    // Enqueue the starting vertex with its priority
    qReg.enqueue(new NodeG(startNum, priarity, node->next));
    priarity++;
    color[startNum] = 1;  // Mark the starting vertex as visiting
    
    // Print starting point for debugging
    std::cout << "Starting BFS from vertex " << startNum << std::endl;
    
    // Perform the BFS traversal as long as the queue is not empty
    while (!qReg.isEmpty()) {
        // Dequeue a vertex u from the queue
        NodeG* u = qReg.dequeue();
        std::cout << "Dequeued vertex: " << u->vertex << std::endl;

        // Get the neighbors of vertex u
        NodeG* neighbor = g.getAdjList(u->vertex);
        if (neighbor == nullptr) {
            // If there are no neighbors, print a message and continue with the next vertex
            std::cout << "No neighbors for vertex " << u->vertex << std::endl;
            continue; 
        }

        // Process all neighbors of vertex u
        while (neighbor != nullptr) {
            int v = neighbor->vertex;
            // Create a new node for the neighbor v with the current priority
            NodeG* vNode = new NodeG(v, priarity);
            priarity++;

            // If the neighbor v has not been visited, enqueue it
            if (color[v] == 0) {
                qReg.enqueue(vNode);
                // Add an edge from u to v in the BFS tree
                bfs_tree.addEdgeNotBoth(u->vertex, v, neighbor->weight);
                // Set the distance of v from the starting vertex u
                d[v] = d[u->vertex] + 1;
                std::cout << "Enqueueing neighbor: " << v << " from vertex " << u->vertex << std::endl;
                // Mark v as visiting
                color[v] = 1;
            } else {
                // If the neighbor has already been visited, delete the node to prevent memory leaks
                delete vNode;
            }
            // Move to the next neighbor
            neighbor = neighbor->next;
        }

        // Mark vertex u as visited
        color[u->vertex] = 2;
        // Free memory for the dequeued vertex
        delete u;
    }

    // Variable to check if the graph is connected
    int k = 0;
    const int INF = std::numeric_limits<int>::max();

    // If the 'connected' pointer is not null, check if all vertices are reachable
    if (connected != nullptr) {
        for (int i = 0; i < numVertices; i++) {
            // If any vertex has an infinite distance, the graph is not connected
            if (d[i] == INF) {
                *connected = false;
                std::cout << "The graph is not connected" << std::endl;
                k = 1;
                break;
            }
        }
        // If no vertices are unreachable, the graph is connected
        if (k == 0) {
            *connected = true;
            std::cout << "The graph is connected" << std::endl;
        }
    }

    // Free the allocated memory for color and distance arrays
    delete[] color;
    delete[] d;

    // Return the BFS tree representing the traversal result
    return bfs_tree;
    }

    // Destructor for Algorithms class
    Algorithms::~Algorithms() {
    }

    
    // Dijkstra's algorithm - Finds the shortest path from a source vertex to all other vertices in a graph.
// This function computes the shortest path using Dijkstra's algorithm and throws an error if there are negative weights.
// Parameters:
//   - g: The graph to find shortest paths in.
//   - src: The source vertex from which to calculate shortest paths.
// Returns:
//   - A graph representing the shortest paths from the source vertex.
graph Algorithms::dijkstra(graph &g, int src) {
    // Constant representing infinity (used for unvisited vertices)
    const int INF = std::numeric_limits<int>::max();
    int numVertices = g.getNumVertices();

    // If there are no vertices in the graph, return an empty graph
    if (numVertices == 0) return graph(0); 

    // Check if the graph contains any edges with negative weights and throw an error if found
    for (int u = 0; u < g.getNumVertices(); ++u) {
        NodeG* neighbor = g.getAdjList(u);
        while (neighbor != nullptr) {
            // If negative weight is found, throw an exception
            if (neighbor->weight < 0) {
                throw std::invalid_argument("Dijkstra cannot run on graphs with negative weights");
            }
            neighbor = neighbor->next;
        }
    }

    // Array to store the shortest distance from the source to each vertex
    int* d = new int[numVertices];
    
    // Initialize all distances to infinity, except for the source vertex
    for (int i = 0; i < numVertices; ++i) {
        d[i] = INF;
    }
    d[src] = 0;  // Distance to the source vertex is 0

    // Create a priority queue (min-heap) to process nodes with the smallest known distance first
    Graph::Queue q;
    for (int i = 0; i < numVertices; ++i) {
        // Enqueue each vertex with its current distance
        Graph::NodeG* node = new Graph::NodeG(i, d[i]);
        q.enqueue(node);
    }

    // Main loop for Dijkstra's algorithm
    while (!q.isEmpty()) {
        // Dequeue the node with the smallest distance (current node)
        Graph::NodeG* currentNode = q.dequeue();
        int u = currentNode->vertex;

        // If the current node's distance is infinity, stop the algorithm
        if (d[u] == INF) {
            break;
        }

        // Process all neighbors of the current node
        Graph::NodeG* neighbor = g.getAdjList(u);
        while (neighbor) {
            int v = neighbor->vertex;
            int weight = neighbor->weight;

            // Relaxation step: If a shorter path to v is found, update its distance
            if (d[u] != INF && d[v] > d[u] + weight) {
                d[v] = d[u] + weight;
                
                // Dequeue and enqueue the updated node to reflect the new distance
                q.dequeue();  
                Graph::NodeG* updatedNode = new Graph::NodeG(v, d[v]);
                q.enqueue(updatedNode); 
            }
             
            // Move to the next neighbor
            neighbor = neighbor->next;
        }
    }

    // Print the results - distances from the source vertex to all other vertices
    std::cout << "Results for source " << src << ":\n";
    for (int i = 0; i < numVertices; ++i) {
        std::cout << "Distance from " << src << " to " << i << " is " << d[i] << std::endl;
    }

    // Clean up memory by deleting the distance array
    delete[] d;

    // Return the graph (the function doesn't modify the graph itself, just calculates distances)
    return g;
}


    // Kruskal's algorithm - Finds a Minimum Spanning Tree (MST) of a graph using Kruskal's algorithm.
// This function adds edges to the MST by considering edges in increasing order of weight, 
// and using the Union-Find data structure to prevent cycles.
// Parameters:
//   - g: The graph to find the MST of.
// Returns:
//   - A graph representing the Minimum Spanning Tree (MST).
graph Algorithms::kruskal(graph& g) {
    int numVertices = g.getNumVertices();
    
    // Check if the graph is empty and return an empty graph if true
    if (numVertices == 0) return graph(0);
    
    // Create a Union-Find data structure to track connected components
    UnionFindSet uf(numVertices);
    
    // Create a graph to store the Minimum Spanning Tree (MST)
    graph mst(numVertices);
    
    // Create an Algorithms object to use other algorithms like BFS
    Algorithms alg;

    bool connected = true;

    // Check if the graph is connected using BFS. If not, Kruskal's algorithm cannot run on disconnected graphs.
    if (g.getAdjList(0) == nullptr) {
        return graph(1); // If there is no adjacency list for vertex 0, return an empty graph
    }
    
    // Perform a BFS from vertex 0 to check if the graph is connected
    alg.bfs(g, 0, &connected);
    if (!connected) {
        throw std::invalid_argument("Kruskal's algorithm cannot run on disconnected graphs");
    }

    // Maximum number of edges possible in a simple graph with 'numVertices' vertices
    const int maxEdges = numVertices * (numVertices - 1) / 2;
    
    // Variable to keep track of the number of edges found
    int edgeCount = 0;
    
    // Define a structure to represent an edge with two vertices and a weight
    struct Edge {
        int u;     // One vertex of the edge
        int v;     // The other vertex of the edge
        int weight; // The weight of the edge
    };
    
    // Create an array to store the edges
    Edge edges[maxEdges];

    // Collect all edges from the graph and store them in the 'edges' array
    for (int i = 0; i < numVertices; i++) {
        NodeG* neighbor = g.getAdjList(i);
        while (neighbor != nullptr) {
            // Add each edge once by ensuring i < neighbor->vertex (to avoid duplicates)
            if (i < neighbor->vertex) { 
                edges[edgeCount++] = {i, neighbor->vertex, neighbor->weight};
            }
            neighbor = neighbor->next;
        }
    }

    // Sort the edges in increasing order of weight using bubble sort
    for (int i = 0; i < edgeCount - 1; i++) {
        for (int j = 0; j < edgeCount - i - 1; j++) {
            // Swap edges if they are in the wrong order
            if (edges[j].weight > edges[j + 1].weight) {
                Edge temp = edges[j];
                edges[j] = edges[j + 1];
                edges[j + 1] = temp;
            }
        }
    }

    // Initialize a variable to track the number of edges added to the MST
    int edgesUsed = 0;

    // Process edges one by one and add them to the MST if they do not form a cycle
    for (int i = 0; i < edgeCount && edgesUsed < numVertices - 1; i++) {
        int u = edges[i].u;   // One vertex of the edge
        int v = edges[i].v;   // The other vertex of the edge
        int weight = edges[i].weight; // The weight of the edge

        // Get the UnionFind nodes for the two vertices of the edge
        NodeUnion* nu = uf.nodes[u];
        NodeUnion* nv = uf.nodes[v];

        // If the vertices are in different sets (i.e., no cycle), add the edge to the MST
        if (uf.findUnion(nu) != uf.findUnion(nv)) {
            std::cout << "Adding edge (" << u << ", " << v << ") with weight: " << weight << std::endl;
            mst.addEdgeNotBoth(u, v, weight); // Add the edge to the MST
            uf.union_(nu, nv); // Merge the two sets to avoid cycles
            edgesUsed++; // Increment the count of edges used in the MST
        }
    }

    // Return the MST as a graph
    return mst;
}

    // Prim's algorithm - Finds a Minimum Spanning Tree (MST) of a graph using Prim's algorithm.
// This function starts from an arbitrary vertex and adds edges to the MST by choosing the minimum weight edge 
// that connects a vertex in the MST to a vertex outside the MST.
// It uses a priority queue (or a simple queue here) to maintain the minimum weight edges.

graph Algorithms::prim(graph& g) {
    int v = g.getNumVertices();  // Get the number of vertices in the graph.
    
    // If the graph has no vertices, return an empty graph.
    if (v == 0) return graph(0);

    // Initialize arrays to store the key, parent, and inMST values for each vertex.
    int* key = new int[v];       // Stores the minimum weight edge that connects the vertex to the MST.
    int* parent = new int[v];    // Stores the parent vertex of each vertex in the MST.
    bool* inMST = new bool[v];   // Stores whether a vertex is included in the MST or not.
    if (g.getAdjList(0) == nullptr) {
        return graph(1); // If there is no adjacency list for vertex 0, return an empty graph
    }
    // Create an Algorithms object to use other algorithms like BFS
    Algorithms alg;

    bool connected = true;

    // Perform a BFS from vertex 0 to check if the graph is connected
    alg.bfs(g, 0, &connected);
    if (!connected) {
        throw std::invalid_argument("Kruskal's algorithm cannot run on disconnected graphs");
    }
    // Initialize all vertices with default values.
    for (int i = 0; i < v; ++i) {
        key[i] = INT_MAX;   // Set the key to infinity for all vertices.
        parent[i] = -1;     // No parent for any vertex initially.
        inMST[i] = false;   // All vertices are initially not in the MST.
    }

    // The first vertex is arbitrarily selected as the start of the MST with key = 0.
    key[0] = 0;

    // Initialize the queue for Prim's algorithm (using a simple queue here).
    Queue q;

    // Create a node for the starting vertex (vertex 0) and enqueue it.
    NodeG* startNode = new NodeG(0, 0);
    q.enqueue(startNode);

    // While the queue is not empty, continue to find the MST.
    while (!q.isEmpty()) {
        // Dequeue a vertex 'u' from the queue.
        NodeG* uNode = q.dequeue(); 
        int u = uNode->vertex;  // Get the vertex number from the dequeued node.
        delete uNode;           // Free memory for the dequeued node.

        // Mark the vertex 'u' as included in the MST.
        inMST[u] = true;

        // Get all adjacent vertices of 'u'.
        NodeG* adj = g.getAdjList(u);
        
        // Check all neighbors of vertex 'u'.
        while (adj != nullptr) {
            int v = adj->vertex;     // Get the adjacent vertex.
            int weight = adj->weight; // Get the weight of the edge.

            // If the adjacent vertex is not in the MST and has a smaller key value, update it.
            if (!inMST[v] && weight < key[v]) {
                key[v] = weight;    // Update the key value to the smaller weight.
                parent[v] = u;      // Set the parent of 'v' as 'u'.

                // Create a new node for the adjacent vertex with its updated key and enqueue it.
                NodeG* neighborNode = new NodeG(v, key[v]);
                q.enqueue(neighborNode); 
            }
            adj = adj->next; // Move to the next neighbor.
        }
    }

    // After the algorithm completes, create a new graph for the MST.
    graph mst(v);

    // Add the edges of the MST to the new graph using the parent array.
    for (int i = 1; i < v; ++i) {
        mst.addEdgeNotBoth(parent[i], i, key[i]); // Add the edge to the MST graph.
    }

    // Clean up dynamically allocated memory for key, parent, and inMST arrays.
    delete[] key;
    delete[] parent;
    delete[] inMST;

    // Return the MST.
    return mst;
}
}

