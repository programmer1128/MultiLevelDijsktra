#include<bits/stdc++.h>

//instead of using a BFS we can also use a priority queue to find the paths with the most
//edge weights first and then use those paths for the flow. in this way, we always try to maximise
//the flow first
bool fattestPathFirst(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity
     ,int n,int &sum)
{
     //since the priority queue uses a template as a argument for the comparator, we cannot use
     //lambda function as they do not have objects.
     //instead we use declare type
     auto cmp=[&](std::pair<int,int>dist1,std::pair<int,int>dist2)-> bool
     {
         return dist1.second<dist2.second;
     } ;

     std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>,decltype(cmp)> queue(cmp);
     std::vector<int>parent(n,-1);
     std::vector<int>maxFlowThroughNode(n,-1);
     maxFlowThroughNode[0]=INT_MAX;
     queue.push(std::make_pair(0,INT_MAX));
     bool found=false;

     while(!queue.empty())
     {
         std::pair<int,int>node=queue.top();
         queue.pop();
         if(node.first==graph.size()-1)
         {
             found=true;
             break;
         }
         if(node.second<maxFlowThroughNode[node.first])
         {
             continue;
         }
         for(std::pair<int,int>neigh:graph[node.first])
         {
             int flow=capacity[node.first][neigh.first];
             
             if(flow>0)
             {
                 //this is the allowed flow through the node
                 int allowedFlow=std::min(flow,node.second);
                 
                 //now if the allowed flow is more than  the prev allowed flow, then only
                 //we consider sending flow through this network
                 if(allowedFlow>maxFlowThroughNode[neigh.first])
                 { 
                     maxFlowThroughNode[neigh.first]=allowedFlow;
                     parent[neigh.first]=node.first;
                     queue.push(std::make_pair(neigh.first,allowedFlow));
                 }
             }
         }
     }

     if(!found)
     {
         //there is no augmenting path left to sink
         return false;
     }
    
     int minAllowedFlow=INT_MAX;
     //if there is a path, then we need to trace it using the parent array
     std::vector<int>path;
     int node=n-1;
     path.push_back(node);

     //calculate the min allowed flow
     while(node!=0)
     {
         minAllowedFlow=std::min(capacity[parent[node]][node],minAllowedFlow);
         node=parent[node];
         path.push_back(node);    
     }

     //update the capacities
     for(int i=path.size()-1;i>0;i--)
     {
         capacity[path[i]][path[i-1]]-=minAllowedFlow;
         capacity[path[i-1]][path[i]]+=minAllowedFlow;
     }
     sum+=minAllowedFlow;
     return true;
}


//now fattest path first presents us 



void findMaxFlow(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity
     ,int n,int m)
{
     int sum=0;
     while(fattestPathFirst(graph,capacity,n,sum))
     {
         
     }
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