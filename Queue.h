//noamglikman1@gmail.com
#ifndef QUEUE_H
#define QUEUE_H
#include "graph.h"

namespace Graph{

    struct Node {
        int data;
        NodeG* next;
        Node(int val) : data(val), next(nullptr) {}
    };

    class Queue {
    private:
        NodeG* front;
        NodeG* rear;

    public:
        Queue();              // Constructor
        ~Queue();             // Destructor

        void enqueue(NodeG* newNode); // Add element
        NodeG* dequeue();          // Remove element
        bool isEmpty() const;   // Check if empty
    };

}

#endif // QUE
