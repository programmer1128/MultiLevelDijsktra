#include<bits/stdc++.h>

int findMaxFlow(std::vector<std::vector<std::pair<int,int>>>&graph
     ,std::vector<std::vector<int>>&capacity,std::vector<bool>&visited,int node,int minCapacity)
{
     if(node==graph.size()-1)
     {
         return minCapacity;
     }
     visited[node]=true;
     for(std::pair<int,int>neigh:graph[node])
     {
         if(!visited[neigh.first]&&capacity[node][neigh.first])
         {
             //this is a valid edge so we can take this into the augmented path
             int minFlow=std::min(minCapacity,capacity[node][neigh.first]);
             int pushFlow=findMaxFlow(graph,capacity,visited,neigh.first,minFlow);

             //after we get the push flow, we check if it is not int max we subtract it
             if(pushFlow>0)
             {
                 //update the capacity matrix for the residual edge and selected edge
                 capacity[node][neigh.first]-=pushFlow;
                 capacity[neigh.first][node]+=pushFlow;

                 return pushFlow;
             }
         }
     }
     //visited[node]=false;
     return 0;
}

void findMaxFlowInNetwork(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity)
{
     int n=graph.size();
     int sum=0;
     while(true)
     {
         std::vector<bool>visited(n,false);

         int pushFlow=findMaxFlow(graph,capacity,visited,0,INT_MAX);

         if(pushFlow==0)
         {
             //this denotes that we can no longer push through the network
             break;
         }
         sum+=pushFlow;
     }
     std::cout<<sum<<"\n";
}


void takeInput()
{
     std::ifstream file("input.txt");
     int test;
     file>>test;

     while(test--)
     {
         int n,m;
         file>>n>>m;

         std::vector<std::vector<std::pair<int,int>>>graph(n,std::vector<std::pair<int,int>>());
         std::vector<std::vector<int>>capacity(n,std::vector<int>(n,0));
         for(int i=0;i<m;i++)
         {
             int u,v,cap;
             file>>u>>v>>cap;
             //since we are applying the optimised version of the dfs here we can keep the residual
             //edges in the same graph. so we will treat the edges as 
             graph[u].push_back(std::make_pair(v,cap));
             graph[v].push_back(std::make_pair(u,cap));
             capacity[u][v]=cap;
         }
         findMaxFlowInNetwork(graph,capacity);
     }
}

int main()
{
     takeInput();
     return 0;
}