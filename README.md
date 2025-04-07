//noamglikman1@gmail.com
# Assigment 1- System Programming 2 -Algorithm implementation with CPP

* **Run Example:**
make clean
make
./main

* **Test run:**
make run_tests

* **Project stracture:**
with run "tree" in terminal:
```bash
.
├── Algorithms.cpp
├── Algorithms.h
├── Algorithms.o
├── Queue.cpp
├── Queue.h
├── Queue.o
├── README.md
├── UnionFind.cpp
├── UnionFind.h
├── UnionFind.o
├── doctest.h
├── graph.cpp
├── graph.h
├── graph.o
├── main
├── main.cpp
├── main.o
├── makefile
├── test
├── test.cpp
└── test.o

### General Explantation of the code ##
I implemented the algorithms alongside the implementation of the priority queue and union-find.
I will explain how I implemented each of the data structures at the signature level to clarify what each pointer and variable represents within the algorithms.
I implemented the priority queue so that every time an element is added, it is placed in the correct position. This way, when we remove an element from the queue, we always remove the node with the minimum weight. Each element in the queue contains data, nodeGnext, and functions for insertion, removal, and checking if the queue is empty or not.

What is NodeG?
It is a node in the graph, each having a unique node number. For example, if there are 5 nodes, they will be numbered from 0 to 5 in ascending order. Additionally, it contains a weight value and a pointer to its neighboring node. In the graph class, there is also a field that points to an array of pointers for all the adjacent nodes of each node, which allows us to access the adjacency list for any given node, essentially creating the adjacency matrix.

So far, we have a graph node called NodeG, an object in the queue called Node, and an object in the union-find called NodeUnion.

In addition, the graph class has two functions for adding edges—one for directed edges and one for undirected edges, as needed. In the algorithms, I added an undirected edge, but if needed, I can simply change the function call to the directed one, called addEdge.

## Explanation of Algorithms:##

# 1 BFS (Breadth-First Search):
This algorithm performs a breadth-first search on a graph, starting from a given node, and explores all the nodes accessible from the starting node while building a BFS tree that represents the traversal order.

Steps of the algorithmBFS (Breadth-First Search):
Initial Checks:
- If the graph has zero nodes, return an empty graph.
- If the starting node is out of bounds, throw an exception.

Initialization:
- color[]: An array marking the state of the nodes (0 = not visited, 1 = visited, 2 = finished).
- d[]: An array to store the distance from the starting node (initialized to infinity, except for the starting node, which is set to 0).

Creating the Queue and BFS Tree:
- The queue contains the nodes waiting to be visited. It starts with the starting node, which is inserted with priority for visiting.
- Every time a node is removed from the queue, check its unvisited neighbors and insert them into the queue.

Performing BFS Traversal:
- For each node visited (removed from the queue), check all its neighbors. If a neighbor hasn’t been visited, add it to the queue, update its distance, and mark it as 'visited.'
- Build the BFS tree by adding edges between the current node and its neighbors.

Connectivity Check (if needed):
- If a connected variable is given, check if all nodes are reachable from the starting node. If not, indicate that the graph is disconnected.

Completion:
- The algorithm returns the BFS tree, representing the nodes and edges explored during the traversal.

# 2 DFS (Depth-First Search):
I implemented the algorithm with two functions. The first is dfs, which initializes an array color (with white = 0) and creates a DFS tree. It then calls the function dfs_visit with the relevant parameters to begin the algorithm.
The dfs_visit function colors the starting node as gray = 1, explores all its neighbors, and recursively visits the required nodes.
The code implements the Depth-First Search (DFS) algorithm on a graph. The dfs function is the main function that starts the traversal from a given source vertex (src). It creates an empty DFS tree (a new graph) and initializes a color array to track the state of each vertex: 0 if it hasn’t been visited yet, 1 if it is currently being visited, and 2 if the visit is finished. Then, it calls the recursive function dfs_visit, which goes through each neighbor of the given vertex. If the neighbor hasn't been visited yet (white), a directed edge is added from the current vertex to the neighbor in the DFS tree, and the neighbor is visited recursively. Each vertex is marked gray at the start of its visit and black at the end. Finally, the constructed DFS tree is returned—a new graph containing only the edges used to reach the vertices during the traversal.

# 3 Dijkstra's Algorithm:
Initial Checks:
- If the graph contains 0 nodes, return an empty graph.
- Check if the graph contains any negative weights. If so, throw an exception because Dijkstra cannot operate on graphs with negative weights.

Initialization:
- d[]: An array storing the shortest distance from the source node to each other node. All distances are initialized to infinity, except for the source node, which is set to 0.

Creating the Queue:
- For each node, create a NodeG object containing the node and its distance (d[i]), and add it to the priority queue.

Graph Traversal:
- Each time the queue is not empty, remove the node with the shortest distance.
- If the distance of a node is infinity, it means all nodes have been visited, so the loop stops.
- For each neighbor of the current node, if its distance can be updated (i.e., if d[v] > d[u] + weight), update its distance and reinsert the node into the queue with the new distance.

Results Output:
- At the end, print the distances from the source node to all other nodes.

Completion:
- The algorithm frees the memory used by d[] and returns the graph.

# 4 Kruskal's Algorithm:
Initial Checks:
- If the graph is empty (no nodes), return an empty graph.
- Check if the graph is connected using BFS from the first node. If it’s not connected, throw an exception since Kruskal can only operate on connected graphs.

Variable Initialization:
- Union-Find (Disjoint Set Union, DSU) is used to prevent cycles while constructing the MST. Each node starts in its own set, and over time, 'union' operations are performed between sets.
- An array of edges is created to store the edges of the graph, each containing the two nodes and the weight of the edge.

Edge Preparation:
- A loop runs over all nodes in the graph, creating an array of edges by checking all neighbors.

Sorting the Edges:
- The array of edges is sorted by their weight in ascending order using bubble sort.

Building the MST:
- The edges are processed in sorted order, and each edge is added to the MST only if the nodes it connects are in different sets. If they are in the same set, it would create a cycle, so it is not added.
- After adding an edge, a 'union' operation is performed on the sets of the two connected nodes.

Completion:
- The algorithm returns the MST, representing the minimum spanning tree of the original graph.

# 5  Prim's Algorithm:
Initial Checks:
- If the graph has 0 nodes, return an empty graph.

Variable Initialization:
- Three arrays are created:
- key[]: Stores the smallest weight for each node to be connected to the MST. Initially set to infinity for all nodes, except for the first node (set to 0).
- parent[]: Stores the previous node on the path to the MST for reconstructing the tree.
- inMST[]: A boolean array marking whether a node is already in the MST.

Starting the Algorithm:
- The first node (node 0) is initially part of the MST, with a key value of 0.

Running the Algorithm:
- In each iteration, the node with the smallest key value is removed from the priority queue and added to the MST.
- For each neighbor of the current node, if the neighbor isn’t already in the MST and its key value is greater than the current edge weight, update the key value and reinsert the neighbor into the queue with the new key.

Building the MST:
- After the algorithm completes, the parent[] array represents the connections between nodes in the MST. Using this array, the edges are added to the MST.

Completion:
- The algorithm returns the MST as a new graph.

.


# Test Cases Overview


## Prim's Algorithm – Edge Cases and General Cases
- Empty Graph  
- Single Vertex Graph  
- Small Graph – 3 Vertices  
- Graph with Negative Weights  

## BFS Algorithm
- Basic BFS on a connected graph  
- Empty Graph  
- BFS with invalid start node  
- BFS with disconnected graph (connectivity flag not checked)  
- BFS not connected (connectivity flag checked)

## Sanity Check
- Simple check: `1 == 1`

## DFS Algorithm
- DFS on a simple linear graph  
- Empty Graph

## Kruskal's Algorithm
- General Case (5 vertices)  
- Small Graph – 3 Vertices  
- Empty Graph
- Single Vertex Graph  
- Graph with One Edge  
- Graph that is not connected (should throw)

## Dijkstra's Algorithm – Edge Cases and General Cases
- Empty Graph  
- Small Graph – 3 Vertices  
- Dijkstra throws on negative weights

## Queue Tests
- Empty Queue  
- Single Element Queue  
- Multiple Elements Queue  
- Dequeue from Empty Queue (should throw)  
- Enqueue and Dequeue

## Union-Find Algorithm
- Union-Find Basic Functionality  
- Union-Find with Multiple Unions

## Graph Class
- Graph Creation and Edge Addition  
- Graph Removal of Edges (check edge removal)  
- Graph with No Edges  
- Graph with Negative Weights
