//noamglikman1@gmail.com
#include <iostream>
#include "Queue.h"
#include "Algorithms.h"
#include "graph.h"
#include "UnionFind.h"

int main() {
    using namespace Graph;


    std::cout << "\nTesting Graph...\n";
    graph g(5);
    g.addEdge(0, 1, 10);
    g.addEdge(0, 4, 20);
    g.addEdge(1, 2, 30);
    g.addEdge(1, 3, 40);
    g.addEdge(1, 4, 50);
    g.addEdge(2, 3, 60);
    g.addEdge(3, 4, 70);
    g.print_graph();
    

    try{
        // Create a graph with 5 vertices
        graph g(5);

        // Add some edges
        g.addEdgeNotBoth(0, 1, 10);
        g.addEdgeNotBoth(0, 2, 20);
        g.addEdgeNotBoth(1, 3, 30);
        g.addEdgeNotBoth(3, 4, 40);

        // Print the graph
        std::cout << "Graph adjacency list:\n";
        g.print_graph();

        // Get total weight
        std::cout << "Total weight of graph: " << g.getTotalWeight() << "\n";

        // Get weight of specific edge
        std::cout << "Weight between 0 and 2: " << g.getEdgeWeight(0, 2) << "\n";

        // Remove an edge
        std::cout << "Removing edge between 0 and 2...\n";
        g.removeEdge(0, 2);

        // Print graph again
        std::cout << "Graph after removing edge:\n";
        g.print_graph();
        } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        }






    std::cout << "\nTesting QUEUE...\n";
    Graph::graph gq(5);

    // יצירת צמתים מסוג NodeG עם ערכים שונים
    Graph::NodeG* node1 = new Graph::NodeG(1, 10, nullptr);
    Graph::NodeG* node2 = new Graph::NodeG(2, 5, nullptr);
    Graph::NodeG* node3 = new Graph::NodeG(3, 15, nullptr);
    Graph::NodeG* node4 = new Graph::NodeG(4, 7, nullptr);

    // יצירת תור חדש
    Graph::Queue queue;

    // הוספת צמתים לתור
    std::cout << "Enqueueing nodes..." << std::endl;
    queue.enqueue(node1);
    queue.enqueue(node2);
    queue.enqueue(node3);
    queue.enqueue(node4);

    // הדפסת התור אחרי כל ההוספות
    std::cout << "Dequeueing nodes..." << std::endl;
    while (!queue.isEmpty()) {
        Graph::NodeG* frontNode = queue.dequeue();
        std::cout << "Dequeued node with vertex " << frontNode->vertex 
                  << " and weight " << frontNode->weight << std::endl;
        delete frontNode;  // דאג למחוק את הצומת לאחר השימוש בו
    }



     std::cout << "\nTesting DFS...\n";
     Graph::Algorithms algo;
     Graph::graph dfsTree = algo.dfs(g, 0);
     dfsTree.print_graph();
     std::cout<<"\nTesting DFS if its not connected graph...\n";
        Graph::graph g1(4);
        g1.addEdge(0, 1, 1);
        g1.addEdge(2, 3, 1);
        g1.print_graph();


    std::cout << "\nTesting Dijkstra...\n";
    Graph::graph dijkstraTree = algo.dijkstra(g, 2);
    dijkstraTree.print_graph();


    std::cout << "\nTesting UnionFind...\n";
    NodeUnion* a = new NodeUnion(1);
    NodeUnion* b = new NodeUnion(2);
    NodeUnion* c = new NodeUnion(3);
    NodeUnion* d = new NodeUnion(4);
    UnionFindSet uf(4);
    uf.union_(a, b);
    uf.union_(c, d);
    uf.union_(b, c);
    if (uf.findUnion(a) == uf.findUnion(d)) {
        std::cout << "Union-Find works correctly!" << std::endl;
    } else {
        std::cout << "Error in Union-Find implementation!" << std::endl;
    }
    

    graph gb(10);
    gb.addEdge(0, 1, 10);
    gb.addEdge(0, 4, 20);
    gb.addEdge(1, 2, 5);
    gb.addEdge(1, 3, 6);
    gb.addEdge(1, 4, 50);
    gb.addEdge(2, 5, 60);
    gb.addEdge(3, 6, 70);
    gb.addEdge(4, 5, 80);
    gb.addEdge(5, 6, 90);
    //gb.addEdge(6, 7, 100);
    //gb.addEdge(7, 8, 110);
    gb.addEdge(8, 9, 120);
    //gb.addEdge(9, 0, 130);
    bool connected;
    std::cout << "\nTesting BFS...\n";
    Graph::graph bfsTree = algo.bfs(gb, 0,&connected);

    bfsTree.print_graph();
    


    std::cout << "\nTesting kruskal's Algorithm...\n";
    Algorithms algo1;
    graph g5(4);
    g5.addEdge(0, 1, 10);
    g5.addEdge(0, 2, 20);
    g5.addEdge(2, 3, 30);
    g5.addEdge(1, 3, 40);
    
    graph mst = algo1.kruskal(g5);
    mst.print_graph();


    std::cout << "\nTesting Prim's Algorithm...\n";

    //create a graph with 5 vertices
    Graph::graph gp(5);

    // Add some edges
    gp.addEdge(0, 1, 2);
    gp.addEdge(0, 3, 6);
    gp.addEdge(1, 2, 3);
    gp.addEdge(1, 3, 8);
    gp.addEdge(1, 4, 5);
    gp.addEdge(2, 4, 7);
    gp.addEdge(3, 4, 9);


    Algorithms alg;
    Graph::graph mst2 = alg.prim(gp);

    // Print the graph
    std::cout << "Minimum Spanning Tree (MST) edges: \n";
    for (int i = 0; i < mst2.getNumVertices(); ++i) {
        Graph::NodeG* adj = mst2.getAdjList(i);
        while (adj != nullptr) {
            std::cout << i << " -- " << adj->vertex << " with weight " << adj->weight << "\n";
            adj = adj->next;
        }
    }
return 0;
}
