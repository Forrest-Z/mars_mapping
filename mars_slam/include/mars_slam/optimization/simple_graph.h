#ifndef SIMPLE_GRAPH_H
#define SIMPLE_GRAPH_H

#include <iostream>
#include <queue>
#include <vector>
#include <ros/ros.h>

typedef struct
{
    int a;
    int b;
}NeighborPair;

class SimpleGraph
{
public:
    explicit SimpleGraph();
    SimpleGraph(const SimpleGraph&) = delete;
    ~SimpleGraph(){}
    void addEdge(const int& a,const int& b);
    int dist(const int& source,const int& target);

private:
    std::vector<std::vector<int>> graph;
};

#endif 