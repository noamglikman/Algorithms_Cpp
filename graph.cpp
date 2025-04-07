// noamglikman1@gmail.com
#include "graph.h"

namespace Graph {
    
// Constructor: Initializes a graph with the given number of vertices
graph::graph(int numVertices) : numVertices(numVertices) {
    adjList = new NodeG*[numVertices]; // Allocate memory for adjacency list
    for (int i = 0; i < numVertices; ++i) {
        adjList[i] = nullptr; // Initialize all pointers to nullptr
    }
}

// Destructor: Frees memory used by the adjacency list
graph::~graph() {
    if (adjList == nullptr) return; // If already null, skip deletion

    // For each vertex, delete its adjacency list
    for (int i = 0; i < numVertices; ++i) {
        NodeG* current = adjList[i];
        while (current != nullptr) {
            NodeG* temp = current;
            current = current->next;
            delete temp; // Delete each node in the list
        }
    }
    delete[]adjList; // Delete the array of pointers
}

// Returns the number of vertices in the graph
int graph::getNumVertices(){
    return numVertices;
}

// Adds a directed edge from src to dest with a given weight (used for tree edges only)
void graph::addEdgeNotBoth(int src, int dest, int weight) {
    // Validate vertex indices
    if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
        throw std::out_of_range("Vertex index out of range");
    }

    // Check if the edge already exists (only from src to dest)
    NodeG* current = adjList[src];
    while (current != nullptr) {
        if (current->vertex == dest) {
            throw std::invalid_argument("Edge already exists from src to dest");
        }
        current = current->next;
    }

    // Add edge from src to dest (insert at beginning of list)
    NodeG* newNode = new NodeG(dest, weight, adjList[src]);
    adjList[src] = newNode;

    std::cout << "Adding edge (" << src << ", " << dest << ") to tree" << std::endl;
}

// Adds an undirected edge between src and dest with a given weight
void graph::addEdge(int src, int dest, int weight) {
    // Validate vertex indices
    if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
        throw std::out_of_range("Vertex index out of range");
    }

    // Check if the edge already exists from src to dest
    NodeG* current = adjList[src];
    while (current != nullptr) {
        if (current->vertex == dest) {
            throw std::invalid_argument("Edge already exists between vertices");
        }
        current = current->next;
    }

    // Add edge from src to dest (insert at beginning of list)
    NodeG* newNode = new NodeG(dest, weight, adjList[src]);
    adjList[src] = newNode;

    // Add edge from dest to src (since graph is undirected)
    NodeG* newNode2 = new NodeG(src, weight, adjList[dest]);
    adjList[dest] = newNode2;
}

// Removes an undirected edge between src and dest
void graph::removeEdge(int src, int dest) {
    if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
        throw std::out_of_range("Vertex out of range");
    }

    // Remove edge from src to dest
    NodeG* curr = adjList[src];
    NodeG* prev = nullptr;
    bool found = false;
    while (curr != nullptr) {
        if (curr->vertex == dest) {
            found = true;
            if (prev == nullptr) {
                adjList[src] = curr->next;
            } else {
                prev->next = curr->next;
            }
            delete curr; // Free memory
            break;
        }
        prev = curr;
        curr = curr->next;
    }

    // Remove edge from dest to src
    curr = adjList[dest];
    prev = nullptr;
    while (curr != nullptr) {
        if (curr->vertex == src) {
            if (prev == nullptr) {
                adjList[dest] = curr->next;
            } else {
                prev->next = curr->next;
            }
            delete curr;
            break;
        }
        prev = curr;
        curr = curr->next;
    }

    // If no edge was found at all, throw exception
    if (!found) {
        throw std::invalid_argument("Edge does not exist");
    }
}

// Prints the entire graph as adjacency list
void graph::print_graph() const {
    for (int i = 0; i < numVertices; ++i) {
        std::cout << i << ": ";
        NodeG* curr = adjList[i];
        while (curr != nullptr) {
            std::cout << "(" << curr->vertex << ", weight=" << curr->weight << ") ";
            curr = curr->next;
        }
        std::cout << "\n"; // Newline after each vertex
    }
}

// Calculates and returns the sum of all edge weights in the graph
int graph::getTotalWeight() {
    int totalWeight = 0;

    for (int u = 0; u < numVertices; ++u) {
        NodeG* adj = getAdjList(u);
        while (adj != nullptr) {
            totalWeight += adj->weight;
            adj = adj->next;
        }
    }
    return totalWeight;
}

// Returns the weight of a specific edge (u,v), or throws exception if not found
int graph::getEdgeWeight(int u, int v) {
    NodeG* adj = getAdjList(u); 
    while (adj != nullptr) {
        if (adj->vertex == v) {
            return adj->weight; 
        }
        adj = adj->next;
    }
    throw std::runtime_error("Edge does not exist between the given vertices");
}

}

// }
