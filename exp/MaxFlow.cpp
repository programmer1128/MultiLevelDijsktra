#include<bits/stdc++.h>

void findMaxFlow(std::vector<std::vector<std::pair<int,int>>>&graph)
{
     
}

void takeInput()
{
     std::ifstream file("input.txt");
     int n,m;
     file>>n>>m;
     std::vector<std::vector<std::pair<int,int>>>graph(n,std::vector<std::pair<int,int>>(m));

     for(int i=0;i<m;i++)
     {
         int u,v,cap;
         file>>u>>v>>cap;
         graph[u].push_back(std::make_pair(v,cap));
         graph[v].push_back(std::make_pair(u,cap));
     }
}

int main()
{
     takeInput();
     return 0;
}