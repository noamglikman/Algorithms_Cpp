// noamglikman1@gmail.com
#include "UnionFind.h"
#include <iostream>

namespace Graph {

    // Constructor: Initializes the Union-Find set with the given number of vertices
    UnionFindSet::UnionFindSet(int numVertices) {
        size = numVertices;
        nodes = new NodeUnion*[numVertices];     // Allocate an array of pointers to NodeUnion
        for (int i = 0; i < numVertices; i++) {
            nodes[i] = new NodeUnion(i);         // Create a new NodeUnion for each vertex
        }
    }

    // Find operation with path compression:
    // Recursively finds the root of the set and flattens the tree for faster future queries
    NodeUnion* UnionFindSet::findUnion(NodeUnion* node) {
        if (node->parent != node) {
            node->parent = findUnion(node->parent);  // Path compression: update parent directly to root
        }
        return node->parent;  // Return the root of the set
    }

    // Union operation by rank:
    // Combines the sets containing node1 and node2, attaching the smaller tree under the larger one
    void UnionFindSet::union_(NodeUnion* node1, NodeUnion* node2) {
        NodeUnion* root1 = findUnion(node1);  // Find root of the first node
        NodeUnion* root2 = findUnion(node2);  // Find root of the second node

        if (root1 == root2) return;  // Already in the same set, nothing to do

        // Attach the tree with lower rank under the one with higher rank
        if (root1->rank > root2->rank) {
            root2->parent = root1;
        } else if (root1->rank < root2->rank) {
            root1->parent = root2;
        } else {
            root2->parent = root1;  // If ranks are equal, choose one arbitrarily and increase its rank
            root1->rank++;
        }
    }

    // Returns the NodeUnion corresponding to a given index
    NodeUnion* UnionFindSet::getNode(int index) {
        return nodes[index];
    }

    // Destructor: Frees all dynamically allocated memory
    UnionFindSet::~UnionFindSet() {
        for (int i = 0; i < size; i++) {
            delete nodes[i];       // Delete each individual NodeUnion
        }
        delete[] nodes;            // Delete the array itself
    }

}
