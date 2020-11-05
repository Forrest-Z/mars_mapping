#include "mars_slam/optimization/simple_graph.h"

SimpleGraph::SimpleGraph(){}

void SimpleGraph::addEdge(const int& a,const int& b)
{
    if(a >= graph.size())
    {
        graph.resize(a+1);
    }

    if(b >= graph.size())
    {
        graph.resize(b+1);
    }

    bool same = false;
    for(unsigned int i = 0;i < graph[a].size();i++)
    {
        if(graph[a][i] == b){
            same = true;
            break;
        }
    }

    if(same)
        return; 

    graph[a].push_back(b);
    graph[b].push_back(a);

}

int SimpleGraph::dist(const int& source,const int& target)
{
    int total = graph.size();
    std::vector<bool> visited_array;
    std::vector<int> dist_array;
    visited_array.resize(total);
    dist_array.resize(total);

    for(int i = 0;i < total;i++)
        visited_array[i] = false;

    visited_array[source] = true;
    dist_array[source] = 0;
    
    int final_dist;

    std::queue<int> queue;
    queue.push(source);
    bool found = false;

    while(queue.size() > 0 && !found)
    {   
        int cur = queue.front();
        queue.pop();

        for(int i = 0; i < graph[cur].size();i++)
        {
            if(!visited_array[graph[cur][i]])
            {
                visited_array[graph[cur][i]] = true;
                dist_array[graph[cur][i]] = dist_array[cur] + 1;
                queue.push(graph[cur][i]);
            
                if(graph[cur][i] == target)
                {
                    found = true;
                    final_dist = dist_array[graph[cur][i]];
                    break;
                }
            }
        }

    }

    return final_dist;
}