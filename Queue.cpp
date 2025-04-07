// noamglikman1@gmail.com
#include "Queue.h"
#include <stdexcept>
#include "graph.h"

namespace Graph {

    // Queue constructor: initializes an empty queue with front and rear pointers set to nullptr
    Queue::Queue() : front(nullptr), rear(nullptr) {}

    // Queue destructor: deallocates all the nodes in the queue to prevent memory leaks
    Queue::~Queue() {
        while (front!=nullptr) {
            NodeG* temp=front;         // Temporarily store the current front
            front=front->next;         // Move front pointer to the next node
            delete temp;               // Delete the previous front node
        }
    }

    // Checks if the queue is empty (i.e., front pointer is null)
    bool Queue::isEmpty() const {
        return front == nullptr;
    }

    // Inserts a node into the queue in a way that maintains the order based on node weight (min-priority)
    void Queue::enqueue(NodeG* newNode) {

        // If the queue is empty or the new node has a smaller weight than the front node
        if (front == nullptr || newNode->weight < front->weight) {
            newNode->next = front;     // Insert the new node at the front
            front = newNode;           // Update front to the new node

            if (rear == nullptr) {     // If the queue was empty, also update rear
                rear = newNode;
            }
            return;
        }
    
        // Traverse the queue to find the correct position for the new node
        NodeG* current = front;
        while (current->next != nullptr && current->next->weight < newNode->weight) {
            current = current->next;   // Move to the next node
        }
    
        // Insert the new node after 'current' and before 'current->next'
        newNode->next = current->next;
        current->next = newNode;
    
        // If the new node was inserted at the end, update the rear pointer
        if (newNode->next == nullptr) {
            rear = newNode;
        }
    }

    // Removes and returns the node with the smallest weight (front of the queue)
    NodeG* Queue::dequeue() {
        if (isEmpty()) {
            throw std::out_of_range("Queue is empty");  // Throw exception if queue is empty
        }
    
        NodeG* temp = front;       // Store the current front
        front = front->next;       // Move front pointer to the next node
    
        if (front == nullptr) {
            rear = nullptr;        // If the queue is now empty, also reset rear
        }
        return temp;               // Return the removed node
    }

}
