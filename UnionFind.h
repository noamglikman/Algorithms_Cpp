//noamglikman1@gmail.com
#ifndef UNIONFIND_H
#define UNIONFIND_H

namespace Graph {

    struct NodeUnion {
        int data;
        NodeUnion* parent;
        int rank;

        NodeUnion(int value) {
            data = value;
            parent = this; 
            rank = 0;
        }
    };

    class UnionFindSet {
    private:
       
        int size;

    public:
        NodeUnion** nodes; 
        UnionFindSet(int numVertices);
        ~UnionFindSet();
        UnionFindSet(const UnionFindSet& other) = delete; // Disable copy constructor
        UnionFindSet& operator=(const UnionFindSet& other) = delete; // Disable assignment operator
        NodeUnion* findUnion(NodeUnion* node);
        void union_(NodeUnion* node1, NodeUnion* node2);
        NodeUnion* getNode(int index); 
    };

}
#endif
