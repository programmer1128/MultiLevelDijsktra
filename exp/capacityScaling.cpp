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

//now fattest path first presents us with an extra added complexity that is the overhead of the 
//priority queue. so we need to reduce that. here is where capacity scaling comes in. in capacity
//scaling we find the max capacity of the edge in the graph then we allot a value delta=2^(p)
//where p= ln max/ln2 . doing this we find the augmented paths in the graph, that have a capacity
//of atleast >=delta and then we reduce delta to a half everytime
//if we look carefully then we can see that the max flow in the graph at any delta state is at max
//E*2delta. so in one phase if we apply dfs to find paths of capacity atleast delta, then there will
//be atmost 2E*delta/delta paths= 2E paths

bool capacityScaling(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity
     ,int delta,int &sum)
{
     int n=graph.size();
     std::vector<bool>visited(n,false);
     std::vector<int>parent(n,-1);
     std::vector<int>path;
     std::queue<std::pair<int,int>>queue;
     
     queue.push(std::make_pair(0,INT_MAX));
     visited[0]=true;
     int minAllowedFlow=-1;
     bool found=false;
     
     while(!queue.empty())
     {
         //get all the nodes at the current level
         int current=queue.size();
         while(current--)
         {
             int node=queue.front().first;
             if(node==n-1)
             {
                 found=true;
                 queue=std::queue<std::pair<int,int>>();
                 break;
             }
             int currFlow=queue.front().second;
             queue.pop();
             for(std::pair<int,int>&neigh:graph[node])
             {
                 if(!visited[neigh.first]&&capacity[node][neigh.first]>=delta)
                 {
                     int allowedFlow=std::min(currFlow,capacity[node][neigh.first]);
                     parent[neigh.first]=node;
                     visited[neigh.first]=true;
                     queue.push(std::make_pair(neigh.first,allowedFlow));
                 }
             }
         }
     }
     if(!found)
     {
         return false;
     }
     int minFlow=INT_MAX;
     int node=n-1;
     path.push_back(node);

     //find the augmented path
     while(node!=0)
     {
         minFlow=std::min(minFlow,capacity[parent[node]][node]);
         node=parent[node];
         path.push_back(node);
     }
     //std::cout<<"hello\n";
     //update the capacities
     for(int i=path.size()-1;i>0;i--)
     {
         capacity[path[i]][path[i-1]]-=minFlow;
         capacity[path[i-1]][path[i]]+=minFlow;
     }
     sum+=minFlow;
     return true;
}


void maxFlowCapacityScaling(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity)
{
     //for applying capacity scaling we first need to find the max edge capacity. we can apply simple bfs
     //for that
     int n=graph.size();
     int maxEdge=-1;
     for(std::pair<int,int>&neigh:graph[0])
     {
         maxEdge=std::max(maxEdge,neigh.first);
     }
     //now that we have the max edge, we need to find the delta
     int power=(int)std::log2(maxEdge);
     int delta=std::pow(2,power);
     int sum=0;
     while(delta!=0)
     {
         while(capacityScaling(graph,capacity,delta,sum)){}
         delta=delta/2;
     }
     std::cout<<sum<<"\n";
} 


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
         //findMaxFlow(graph,capacity,n,m);
         maxFlowCapacityScaling(graph,capacity);
     }
}

int main()
{
     takeInput();
     return 0;
}