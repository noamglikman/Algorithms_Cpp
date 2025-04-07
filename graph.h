//noamglikman1@gmail.com
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
using namespace std;

namespace Graph {
    typedef struct NodeG{
        int vertex;
        int weight;
        NodeG* next;
       

        NodeG(int v, int w, NodeG* n =nullptr) : vertex(v), weight(w), next(n) {}
    }NodeG;
    
    class graph{
        private:
            int numVertices;
            NodeG** adjList;
        public:
            graph(int vertices);
            ~graph();
            graph&operator=(const graph&other){
                if (this != &other) {
                    for (int i = 0; i < numVertices; ++i) {
                        NodeG* current = adjList[i];
                        while (current != nullptr) {
                            NodeG* temp = current;
                            current = current->next;
                            delete temp;
                        }
                    }
                    delete[] adjList;

                    numVertices = other.numVertices;
                    adjList = new NodeG*[numVertices];
                    
                    for(int i=0; i<numVertices; ++i) {
                        NodeG* current = other.adjList[i];
                        NodeG** copy = &adjList[i];
                        *copy = nullptr;
                        
                        while(current != nullptr) {
                            *copy = new NodeG(current->vertex, current->weight);
                            current = current->next;
                            copy = &((*copy)->next);
                        }
                    }
                }
                return *this;
            }
            void addEdgeNotBoth(int src, int dest, int weight);
            int getEdgeWeight(int u, int v);
            int getTotalWeight();
            void addEdge(int src, int dest, int weight = 1);
            void removeEdge(int src, int dest);
            void print_graph() const;
            int getNumVertices();
            NodeG** getAdjList() {
                return adjList; 
            }
            NodeG* getAdjList(int v) const {
                if (v < 0 || v >= numVertices) { 
                    throw string( "Error: Vertex  is out of bounds!" );
                }
                return adjList[v]; 
            }
            graph(const graph& other) : numVertices(other.numVertices) {
                adjList = new NodeG*[numVertices];
                
                for(int i=0; i<numVertices; ++i) {
                    NodeG* current = other.adjList[i];
                    NodeG** copy = &adjList[i];
                    *copy = nullptr;
                    
                    while(current != nullptr) {
                        *copy = new NodeG(current->vertex, current->weight);
                        current = current->next;
                        copy = &((*copy)->next);
                    }
                }
            }
            
    };
    
    }
#endif