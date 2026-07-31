#include<bits/stdc++.h>

bool edmondKarp(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity
     ,int n,int &sum)
{
     //queue for the bfs
     std::queue<int>queue;
     std::vector<bool>visited(n,false);
     //parent array to trace the augmenting path back
     std::vector<int>parent(n,-1);
     bool found=false;
     //offer source to the queue
     queue.push(0);
     visited[0]=true;
     while(!queue.empty())
     {
         //get all the nodes at the current level
         int current=queue.size();
         while(current--)
         {
             int node=queue.front();
             queue.pop();

             if(node==graph.size()-1)
             {
                 found=true;
                 queue=std::queue<int>();
                 break;
             }

             for(std::pair<int,int>&neigh:graph[node])
             {
                 if(!visited[neigh.first]&&capacity[node][neigh.first]>0)
                 {
                     //mark node as visited
                     visited[neigh.first]=true;
                     //mark the parent of node
                     parent[neigh.first]=node;
                     //push to queue
                     queue.push(neigh.first);
                 }
             }
         }
     }

     if(!found)
     {
         //all the augmenting paths with capacity more than zero have been explored
         return false;
     }

     //find the min capacity along the path by retracing the path
     int node=graph.size()-1;
     int minAllowedFlow=INT_MAX;
     std::vector<int>path;
     path.push_back(node);
     while(node!=0)
     {
         minAllowedFlow=std::min(capacity[parent[node]][node],minAllowedFlow);
         node=parent[node];
         path.push_back(node);
     }

     //update the capacities of the path
     for(int i=path.size()-1;i>0;i--)
     {
         capacity[path[i]][path[i-1]]-=minAllowedFlow;
         capacity[path[i-1]][path[i]]+=minAllowedFlow;
     }
     sum+=minAllowedFlow;
     return true;
}

void findMaxFlow(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity
     ,int n,int m)
{
     int sum=0;
     while(edmondKarp(graph,capacity,n,sum)){}

     std::cout<<sum<<"\n";
}

void takeInput()
{
     std::freopen("input.txt","r",stdin);

     int test;
     std::cin>>test;
     while(test--)
     {
         int n,m;
         std::cin>>n>>m;
         std::vector<std::vector<std::pair<int,int>>>graph(n);
         std::vector<std::vector<int>>capacity(n,std::vector<int>(n,0));
         for(int i=0;i<m;i++)
         {
             int u,v,wt;
             std::cin>>u>>v>>wt;
             //normal edge
             graph[u].push_back(std::make_pair(v,wt));
             //residual edge
             graph[v].push_back(std::make_pair(u,0));
             capacity[u][v]=wt;
             capacity[v][u]=0;
         }    
         findMaxFlow(graph,capacity,n,m);
     }
}

int main()
{
     takeInput();
     return 0;
}