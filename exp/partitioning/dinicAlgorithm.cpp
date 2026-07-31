#include<bits/stdc++.h>

int maxFlow(std::vector<std::vector<int>>&graph,std::vector<std::vector<int>>&capacity
     ,std::vector<int>&deadEnd,int node,int pushedFlow)
{
     if(node==graph.size()-1||pushedFlow==0)
     {
         return pushedFlow;
     }
     //visited[node]=true;
     for(int &i=deadEnd[node];i<graph[node].size();i++)
     {
         int neigh=graph[node][i];
         if(capacity[node][neigh]>0)
         {
             int allowedFlow=std::min(pushedFlow,capacity[node][neigh]);
             int flow=maxFlow(graph,capacity,deadEnd,neigh,allowedFlow);

             if(flow>0)
             {
                 //this is a valid augmenting path
                 //update the capacities
                 //decrease normal edge capacity
                 capacity[node][neigh]-=flow;
                 //increase residual edge capacity
                 capacity[neigh][node]+=flow;

                 //return flow of this path
                 return flow;
             }
         }
     }
     return 0;
}

bool buildLevelGraph(std::vector<std::vector<std::pair<std::pair<int,int>,char>>>&graph
     ,std::vector<std::vector<int>>&levelGraph,std::vector<std::vector<int>>&capacity)
{
     //apply BFS on the graph, to explore the levels. and accordingly add the edges
     int n=graph.size();
     std::vector<int>levels(n,-1);
     std::queue<int> queue;
     queue.push(0);
     levels[0]=0;
     bool found=false;

     while(!queue.empty())
     {
         int current=queue.size();
         while(current--)
         {
             int node=queue.front();
             queue.pop();
             if(node==n-1)
             {
                 found=true;
                 queue=std::queue<int>();
                 break;
             }
             for(std::pair<std::pair<int,int>,char>&neigh:graph[node])
             {
                 std::pair<int,int>neighNode=neigh.first;
                 if(capacity[node][neighNode.first]>0)
                 {
                     //this may be a valid edge so we need to check the levels first
                     if(levels[neighNode.first]==-1)
                     {
                         //this is the first time we are seeing the node so it will have the level+1
                         //of parent
                         levels[neighNode.first]=levels[node]+1;
                         queue.push(neighNode.first);
                     }
                     if(levels[neighNode.first] == levels[node] + 1)
                     {
                         levelGraph[node].push_back(neighNode.first);
                     }
                 }  
             }
         }
     }
     return found;
}

void dinic(std::vector<std::vector<std::pair<std::pair<int,int>,char>>>&graph,std::vector<std::vector<int>>&capacity)
{
     //in dinic algorithm we need to figure out the level graph first. in the level graph, we have
     //to take original edges only and those edges should have capacity more than 0. for such a 
     //level graph, we continue to find the augmented paths. when we can no longer find augmented paths
     //for a level graph, we stop. then we again make a level graph based on the updated capacities
     //we keep doing this till we can no longer reach the node sink
     int sum=0;
     while(true)
     {
         std::vector<std::vector<int>>levelGraph(graph.size());
         std::vector<int>deadEnds(graph.size(),0);
         //build the level graph
         if(!buildLevelGraph(graph,levelGraph,capacity))
         {
             //if level graph does not include the sink node, that means no valid path
             //exists to the sink. we need to break
             break;
         }
         //if there is a valid level graph, then we need to keep applying DFS to it till the
         //max flow of the level graph is zero
         while(true)
         {
             //std::vector<bool>visited(graph.size(),false);
             int flow=maxFlow(levelGraph,capacity,deadEnds,0,INT_MAX);
             if(flow==0)
             {
                 break;
             }
             sum+=flow;
         }
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
         std::vector<std::vector<std::pair<std::pair<int,int>,char>>>graph(n);
         std::vector<std::vector<int>>capacity(n,std::vector<int>(n,0));
         for(int i=0;i<m;i++)
         {
             int u,v,wt;
             std::cin>>u>>v>>wt;
             //normal edge
             std::pair<int,int>normalEdge=std::make_pair(v,wt);
             std::pair<int,int>resEdge=std::make_pair(u,wt);
             graph[u].push_back(std::make_pair(normalEdge,'N'));
             //residual edge
             graph[v].push_back(std::make_pair(resEdge,'R'));
             capacity[u][v]=wt;
             capacity[v][u]=0;
         }    
         //findMaxFlow(graph,capacity,n,m);
         //maxFlowCapacityScaling(graph,capacity);
         dinic(graph,capacity);
     }
}

int main()
{
     takeInput();
     return 0;
}