//noamglikman1@gmail.com
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "graph.h"
#include "Algorithms.h"
#include "Queue.h"
#include "UnionFind.h"
using namespace Graph;
TEST_CASE("Prim's Algorithm - Edge Cases and General Cases") {
    SUBCASE("Empty Graph") {
        Algorithms algo;
        graph g(0);
        graph mst = algo.prim(g);
        CHECK(mst.getNumVertices() == 0);
    }
    SUBCASE("Single Vertex Graph") {
        Algorithms algo;
        graph g(1);
        graph mst = algo.prim(g);
        CHECK(mst.getNumVertices() == 1);
    }
    SUBCASE("Small Graph - 3 Vertices") {
        Algorithms algo;
        graph g(3);
        g.addEdge(0, 1, 10);
        g.addEdge(1, 2, 5);
        g.addEdge(0, 2, 15);
        graph mst = algo.prim(g);
        CHECK(mst.getNumVertices() == 3);
        CHECK(mst.getTotalWeight() == 15);
    }
    
    SUBCASE("Graph with Negative Weights") {
        Algorithms algo;
        graph g(4);
        g.addEdge(0, 1, -2);
        g.addEdge(1, 2, -3);
        g.addEdge(2, 3, -1);
        g.addEdge(3, 0, -4);
        g.addEdge(0, 2, -5);
        graph mst = algo.prim(g);
        CHECK(mst.getNumVertices() == 4);
        CHECK(mst.getTotalWeight() == -12);
    }
    SUBCASE("Graph that is not connected") {
        Algorithms algo;
        graph g(4);
        g.addEdge(0, 1, 10);
        g.addEdge(2, 3, 20);
        CHECK_THROWS_AS(algo.prim(g), std::invalid_argument);
    }
}
    
TEST_CASE("BFS Algorithm") {
        std::cout << "Starting BFS from vertex " << std::endl;
        Algorithms algo;
        graph g(4);
        g.addEdge(0, 1, 1);
        g.addEdge(1, 2, 1);
        g.addEdge(2, 3, 1);
        
        NodeG* startNode = g.getAdjList(0);  // בחירת הקודקוד 0 כתחילת ה-BFS
        Graph::graph bfsTree = algo.bfs(g, 0);
        CHECK(bfsTree.getNumVertices() == 4);
        
        SUBCASE("Empty Graph") {
            Algorithms algo;
            graph g(0);
            CHECK_THROWS_AS(algo.bfs(g, 0), std::out_of_range);
            
        }
        SUBCASE("BFS with invalid start node"){
                graph g(4); 
                Algorithms algo;
                CHECK_THROWS_AS(algo.bfs(g, 5), std::out_of_range);
        }
        SUBCASE("BFS with disconnected graph") {
            Algorithms algo;
            graph g(4);
            g.addEdge(0, 1, 1);
            g.addEdge(2, 3, 1);
            bool connected;
            NodeG* startNode = g.getAdjList(0);  
            Graph::graph bfsTree = algo.bfs(g, 0,&connected);
            CHECK(connected == false);
        }
        SUBCASE("BFS not connected") {
            Algorithms algo;
            graph g(4);
            g.addEdge(0, 1, 1);
            g.addEdge(2, 3, 1);
            bool connected;
            NodeG* startNode = g.getAdjList(0);  
            Graph::graph bfsTree = algo.bfs(g, 0,&connected);
            CHECK(connected == false);
        }
}
TEST_CASE("Sanity Check") {
    std::cout << "Doctest is running!" << std::endl;
    CHECK(1 == 1);
}
TEST_CASE("DFS Algorithm") {
    Algorithms algo;
    graph g(4);
    g.addEdge(0, 1, 1);
    g.addEdge(1, 2, 1);
    g.addEdge(2, 3, 1);
    
    graph dfsTree = algo.dfs(g, 0);
    
    CHECK(dfsTree.getNumVertices() == 4);
    SUBCASE("Empty Graph") {
        Algorithms algo;
        graph g(0);
        graph mst = algo.dfs(g,0);
        CHECK(mst.getNumVertices() == 0);
    }
    SUBCASE("DFS whit not exist vertex") {
        Algorithms algo;
        graph g(4);
        CHECK_THROWS_AS(algo.dfs(g, 5), std::out_of_range);
    }
 }
TEST_CASE("Kruskal Algorithm") {
    Algorithms algo;
    graph g(5);
    g.addEdge(0, 1, 10);
    g.addEdge(0, 2, 20);
    g.addEdge(2, 3, 30);
    g.addEdge(1, 4, 20);
    g.addEdge(1, 3, 40);
    
    g.print_graph();
    
    graph mst = algo.kruskal(g);
    
    CHECK(mst.getNumVertices() == 5);
    mst.print_graph();
    CHECK(mst.getTotalWeight() ==80 );
    
    SUBCASE("Small Graph - 3 Vertices") {
        Algorithms algo;
        graph g(3);
        g.addEdge(0, 1, 10);
        g.addEdge(1, 2, 5);
        g.addEdge(0, 2, 15);
        graph mst = algo.kruskal(g);
        CHECK(mst.getNumVertices() == 3);
        CHECK(mst.getTotalWeight() == 15);
    }
    SUBCASE("Empty Graph") {
        Algorithms algo;
        graph g(0);
        graph mst = algo.kruskal(g);
        CHECK(mst.getNumVertices() == 0);
    }
    SUBCASE("Single Vertex Graph") {
        Algorithms algo;
        graph g(1);
        graph mst = algo.kruskal(g);
        CHECK(mst.getNumVertices() == 1);
    }

    SUBCASE("Graph with One Edge") {
        Algorithms algo;
        graph g(2);
        g.addEdge(0, 1, 5);
        graph mst = algo.kruskal(g);
        CHECK(mst.getNumVertices() == 2);
        CHECK(mst.getTotalWeight() == 5);
    }
    SUBCASE("Graph that is not connected") {
        Algorithms algo;
        graph g(4);
        g.addEdge(0, 1, 10);
        g.addEdge(2, 3, 20);
        CHECK_THROWS_AS(algo.kruskal(g), std::invalid_argument);
    }
 }
TEST_CASE("Dijkstra's Algorithm - Edge Cases and General Cases") {
    Algorithms algo;

    SUBCASE("Empty Graph") {
        graph g(0);
        graph shortestPaths = algo.dijkstra(g, 0);
        CHECK(shortestPaths.getNumVertices() == 0);
    }
    SUBCASE("Small Graph - 3 Vertices") {
        graph g(3);
        g.addEdge(0, 1, 10);
        g.addEdge(1, 2, 5);
        g.addEdge(0, 2, 15);

        graph shortestPaths = algo.dijkstra(g, 0);
        
        CHECK(shortestPaths.getNumVertices() == 3);
        CHECK(shortestPaths.getEdgeWeight(0, 1) == 10);
        CHECK(shortestPaths.getEdgeWeight(1, 2) == 5);
        CHECK(shortestPaths.getEdgeWeight(0, 2) == 15);
    }
    SUBCASE("Dijkstra throws on negative weights") {
        graph g(3);
        g.addEdge(0, 1, 4);
        g.addEdge(1, 2, -5);

        Algorithms algo;
        CHECK_THROWS_AS(algo.dijkstra(g,0), std::invalid_argument);
    }
}

TEST_CASE("QUEUEUE") {
    SUBCASE("Empty Queue") {
        Graph::Queue q;
        CHECK(q.isEmpty() == true);
    }
    SUBCASE("Single Element Queue") {
        Graph::Queue q;
        NodeG* node = new NodeG(1, 10);
        q.enqueue(node);
        CHECK(q.isEmpty() == false);
        CHECK(q.dequeue()->vertex == 1);
    }
    SUBCASE("Multiple Elements Queue") {
        Graph::Queue q;
        NodeG* node1 = new NodeG(1, 10);
        NodeG* node2 = new NodeG(2, 20);
        NodeG* node3 = new NodeG(3, 30);

        q.enqueue(node1);
        q.enqueue(node2);
        q.enqueue(node3);

        CHECK(q.dequeue()->vertex == 1);
        CHECK(q.dequeue()->vertex == 2);
        CHECK(q.dequeue()->vertex == 3);
    }
    SUBCASE("Dequeue from Empty Queue") {
        Graph::Queue q;
        CHECK_THROWS(q.dequeue());
    }
    SUBCASE("Enqueue and Dequeue") {
        Graph::Queue q;
        NodeG* node1 = new NodeG(1, 10);
        NodeG* node2 = new NodeG(2, 20);

        q.enqueue(node1);
        q.enqueue(node2);

        CHECK(q.dequeue()->vertex == 1);
        CHECK(q.dequeue()->vertex == 2);
    }
}
TEST_CASE("Union-Find Algorithm") {
    SUBCASE("Union-Find Basic Functionality") {
        UnionFindSet uf(5);
        NodeUnion* a = new NodeUnion(1);
        NodeUnion* b = new NodeUnion(2);
        NodeUnion* c = new NodeUnion(3);

        uf.union_(a, b);
        CHECK(uf.findUnion(a) == uf.findUnion(b));
        CHECK(uf.findUnion(a) != uf.findUnion(c));
    }
    SUBCASE("Union-Find with Multiple Unions") {
    UnionFindSet uf(5);
    NodeUnion* a = new NodeUnion(1);
    NodeUnion* b = new NodeUnion(2);
    NodeUnion* c = new NodeUnion(3);
    NodeUnion* d = new NodeUnion(4);

    uf.union_(a, b);
    uf.union_(c, d);
    uf.union_(b, c);

    CHECK(uf.findUnion(a) == uf.findUnion(d));

}
}
TEST_CASE("graph") {
    SUBCASE("Graph Creation and Edge Addition") {
        graph g(5);
        g.addEdge(0, 1, 10);
        g.addEdge(1, 2, 20);
        g.addEdge(2, 3, 30);
        g.addEdge(3, 4, 40);

        CHECK(g.getNumVertices() == 5);
        CHECK(g.getEdgeWeight(0, 1) == 10);
        CHECK(g.getEdgeWeight(1, 2) == 20);
        CHECK(g.getEdgeWeight(2, 3) == 30);
        CHECK(g.getEdgeWeight(3, 4) == 40);
    }
    SUBCASE("Graph Removal of Edges") {
        graph g(5);
        g.addEdge(0, 1, 10);
        g.addEdge(1, 2, 20);
        g.removeEdge(0, 1);

        CHECK_THROWS(g.getEdgeWeight(0, 1));
    }
    SUBCASE("Graph with No Edges") {
        graph g(5);
        CHECK(g.getNumVertices() == 5);
        CHECK_THROWS(g.getEdgeWeight(0, 1));
    }
    SUBCASE("Graph with Negative Weights") {
        graph g(4);
        g.addEdge(0, 1, -2);
        g.addEdge(1, 2, -3);
        g.addEdge(2, 3, -1);

        CHECK(g.getEdgeWeight(0, 1) == -2);
        CHECK(g.getEdgeWeight(1, 2) == -3);
        CHECK(g.getEdgeWeight(2, 3) == -1);
    }
    SUBCASE("Graph with Self-Loop") {
        graph g(3);
        g.addEdge(0, 0, 5);
        g.addEdge(1, 2, 10);

        CHECK(g.getEdgeWeight(0, 0) == 5);
        CHECK(g.getEdgeWeight(1, 2) == 10);
    }
    SUBCASE("Graph with Multiple Edges") {
        graph g(3);
        g.addEdge(0, 1, 10); 
        CHECK_THROWS_AS(g.addEdge(0, 1, 5), std::invalid_argument);
    }
    SUBCASE("Graph with Invalid Vertex") {
        graph g(3);
        CHECK_THROWS(g.addEdge(0, 3, 10));
        CHECK_THROWS(g.addEdge(3, 0, 10));
    }
    SUBCASE("Graph with No Vertices") {
        graph g(0);
        CHECK(g.getNumVertices() == 0);
    }
    SUBCASE("Graph with One Vertex") {
        graph g(1);
        CHECK(g.getNumVertices() == 1);
        CHECK_THROWS(g.addEdge(0, 1, 10));
    }
    SUBCASE("Graph with Two Vertices") {
        graph g(2);
        g.addEdge(0, 1, 10);
        CHECK(g.getNumVertices() == 2);
        CHECK(g.getEdgeWeight(0, 1) == 10);
    }
}