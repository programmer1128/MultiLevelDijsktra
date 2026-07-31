#include<bits/stdc++.h>

typedef struct Node
{
     int nodeId;
     double lat;
     double lon;
     char type;
     std::string location;
     Node() : nodeId(0), lat(0.0), lon(0.0), type(' '), location("") {}
     Node(int nodeId,double lat,double lon,std::string location)
     {
         this->nodeId=nodeId;
         this->lat=lat;
         this->lon=lon;
         this->location=location;
     }
     Node(char t)
     { 
         this->type=t;
     }
}Node;

//finds the max flow
int maxFlow(std::unordered_map<int,std::vector<int>>&graph,std::unordered_map<int,std::unordered_map<int,int>>&capacity
     ,std::unordered_map<int,int>&deadEnd,int node,int pushedFlow)
{
     if(node==-2||pushedFlow==0)
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

//finds the level graph required for the dinic algorithm
bool findLevelGraph(std::unordered_map<int,std::vector<Node>>&graph,std::unordered_map<int,std::vector<int>>&levelGraph
     ,std::unordered_map<int,std::unordered_map<int,int>>&capacity)
{
     int n=graph.size();
     std::unordered_map<int,int>levels;
     std::queue<int>queue;
     levels[-1]=0;
     queue.push(-1);
     bool found=false;
     while(!queue.empty())
     {
         int current=queue.size();
         while(current--)
         {
             int nodeId=queue.front();
             queue.pop();
             if(nodeId==-2)
             {
                 //this is the sink
                 found=true;
                 continue;
             }
             for(Node &node:graph[nodeId])
             {
                 if(capacity[nodeId][node.nodeId]<=0)
                 {
                     continue;
                 }
                 if(levels.find(node.nodeId)==levels.end())
                 {
                     //we are seeing this node for the first time
                     levels[node.nodeId]=levels[nodeId]+1;
                     //add this node to the edge graph
                     levelGraph[nodeId].push_back(node.nodeId);
                     queue.push(node.nodeId);
                 }
                 else if(levels[node.nodeId]==levels[nodeId]+1)
                 {
                     //this is also a valid edge
                     levelGraph[nodeId].push_back(node.nodeId);
                 }
             }
         }
     }

     return found;
}

void dinic(std::unordered_map<int,std::vector<Node>>&graph
     ,std::unordered_map<int,std::unordered_map<int,int>>&capacity)
{
     //now we apply dinic algorithm to the graph and find the partitions
     //in dinic algorithm we find the levels graph by applying a BFS and then apply the 
     //max flow on that level graph
     int n=graph.size();
     while(true)
     {
         std::unordered_map<int,std::vector<int>>levelGraph;
         if(!findLevelGraph(graph,levelGraph,capacity))
         {
             break;
         }
         std::unordered_map<int,int>deadEnd;
         for(const auto& key:graph)
         {
             deadEnd[key.first]=0;
         }
         while(true)
         {
             int flow=maxFlow(levelGraph,capacity,deadEnd,-1,INT_MAX);
             if(flow==0)
             {
                 break;
             }
         }
     }


}


void partitionGraph(std::unordered_map<int,std::vector<Node>>&graph,
                    std::vector<Node>&nodes, std::unordered_map<int,Node>&nodeMap, int n)
{
     auto cmp=[&](Node a,Node b)->bool { return a.lon > b.lon; };
     std::sort(nodes.begin(), nodes.end(), cmp);
     
     int virtualSourceId = -1;
     int virtualSinkId = -2;
     Node sinkNode(-2, 0.0, 0.0, "Sink");

     // Define how many nodes to pin on each extreme side
     int k = std::max(1, (int)(nodes.size() / 3)); 

     for(int i = 0; i < k; i++) 
     {
         graph[virtualSourceId].push_back(nodes[i]);
     }
     
     // 2. Connect the BOTTOM k sink candidate nodes to the Virtual Sink
     for(int i = nodes.size() - k; i < nodes.size(); i++) 
     {
         graph[nodes[i].nodeId].push_back(sinkNode);
     }

     std::unordered_map<int, std::unordered_map<int, int>> capacity;
     for(const auto& key : graph) 
     {
         for(Node &node : graph[key.first]) 
         {
             capacity[key.first][node.nodeId] = 1;
             capacity[node.nodeId][key.first] = 1;
         }
     }

     //virtaul source nodes to selected real nodes as source and max capacity
     for(int i = 0; i < k; i++) 
     {
         int u = nodes[i].nodeId;
         capacity[virtualSourceId][u] = INT_MAX;
         capacity[u][virtualSourceId] = 0;
     }
     //virtual sink nodes to selected real sink nodes for max capacity flow
     for(int i = nodes.size() - k; i < nodes.size(); i++) 
     {
         int u = nodes[i].nodeId;
         capacity[u][virtualSinkId] = INT_MAX;
         capacity[virtualSinkId][u] = 0;
     }
     
     dinic(graph,capacity);

     //final BFS to see which of the paths are reachable
     std::unordered_map<int,bool>reachable;
     std::queue<int>queue;

     queue.push(-1);
     reachable[-1]=true;

     while(!queue.empty())
     {
         int u=queue.front();
         queue.pop();
         for(const Node& node:graph[u])
         {
             int v=node.nodeId;
             if(capacity[u][v]>0&&!reachable[v])
             {
                 reachable[v]=true;
                 queue.push(v);
             }
         }
     }

     for (const auto& nodes : graph) 
     {
         int u = nodes.first;
         if (u == -1 || u == -2) 
         {
             continue; 
         }
         for (const Node& node : graph[u]) 
         {
             int v = node.nodeId;   
             if (v == -1 || v == -2) 
             {
                 continue;
             }
             if (reachable[u] && !reachable[v]) 
             {
                 std::cout << "Partition boundary found between " << u << " and " << v << "\n";
             }
         }
     }
}


void takeInput()
{
     std::freopen("inertialFlowInput.txt","r",stdin);
     int n,m;
     std::cin>>n>>m;
     std::unordered_map<int,std::vector<Node>>graph;
     std::vector<bool>visited(n,false);
     std::unordered_map<int,Node>nodeMap;
     std::vector<Node> nodeArray;

     for(int i=0;i<m;i++)
     {
         int u,v;
         double latU,lonU,latV,lonV;
         std::cin>>u>>v>>latU>>lonU>>latV>>lonV;
         if(!visited[u])
         {
             visited[u]=true;
             Node nodeU(u,latU,lonU,"u");
             nodeMap[u]=nodeU;
             nodeArray.push_back(nodeU);
         }
         if(!visited[v])
         {
             visited[v]=true;
             Node nodeV(v,latV,lonV,"v");
             nodeMap[v]=nodeV;
             nodeArray.push_back(nodeV);
         }
         
         graph[u].push_back(nodeMap[v]);
         graph[v].push_back(nodeMap[u]);
     }
     partitionGraph(graph,nodeArray,nodeMap,n);
}

int main()
{
     takeInput();
     return 0;
}

/*
dinic(graph,capacity);

     //final BFS to see which of the paths are reachable
     std::unordered_map<int,bool>reachable;
     std::queue<int>queue;

     queue.push(-1);
     reachable[-1]=true;

     while(!queue.empty())
     {
         int u=queue.front();
         queue.pop();
         for(const Node& node:graph[u])
         {
             int v=node.nodeId;
             if(capacity[u][v]>0&&!reachable[v])
             {
                 reachable[v]=true;
                 queue.push(v);
             }
         }
     }

     for (const auto& nodes : graph) 
     {
         int u = nodes.first;
         if (u == -1 || u == -2) 
         {
             continue; 
         }
         for (const Node& node : graph[u]) 
         {
             int v = node.nodeId;   
             if (v == -1 || v == -2) 
             {
                 continue;
             }
             if (reachable[u] && !reachable[v]) 
             {
                 std::cout << "Partition boundary found between " << u << " and " << v << "\n";
             }
         }
     }*/