//noamglikman1@gmail.com
#ifndef ALGORITHMS_HEADER
#define ALGORITHMS_HEADER
#include <iostream>
#include "graph.h"

namespace Graph{
  class Algorithms
  {
  public:
    Algorithms(int numVer);
    ~Algorithms();
    Algorithms() {} // בנאי ריק

    graph bfs(graph& g,int startNum,bool* connected=nullptr);
    graph dfs_visit(graph& g, graph& dfs_tree, int v, int color[]);
    graph dfs(graph& g, int src);
    graph dijkstra(graph& g,int src);
    graph prim(graph& g);
    graph kruskal(graph& g);

  };

}
#endif
