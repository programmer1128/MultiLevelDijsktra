#include<bits/stdc++.h>


const inline void display(const std::vector<std::pair<int,char>>&path)
{
      std::cout<<"printing path\n";
      for(const auto&r:path)
      {
           std::cout<<"("<<r.first<<","<<r.second<<") ";
      }
      std::cout<<"\n";
}

int sum=0;

//brute force dfs to solve the Max flow problem
void dfs(const std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity
      ,std::vector<std::unordered_set<int>>&residualGraph,std::vector<std::vector<int>>&residualCapacity
      ,std::vector<bool>&inPath,std::vector<std::pair<int,char>>&path
      ,int node,int parent)
{
      if(inPath[node])
      {
           return ;
      }
     
      if(node==graph.size()-1)
      {
           //this is a valid path from the source to the sink
           //add residual edge
           residualGraph[node].insert(parent);
           display(path);
           //add the minCapacity for this path to the capacirty mat
           int min=INT_MAX;
           for(int i=1;i<path.size();i++)
           {
                //if the edge was a normal edge then calculate from normal capacity
                if(path[i].second=='N')
                {
                     min=std::min(min,capacity[path[i-1].first][path[i].first]);
                }
                else
                {    
                     //if the edge used was residual cal from res capacity
                     min=std::min(min,residualCapacity[path[i-1].first][path[i].first]);
                }
           }
           sum+=min;
           //std::cout<<"\n";
           //std::cout<<"\nmin capa = "<<min<<" returning\n\n";

           //update the capacities so that they are affected globally
           for(int i=1;i<path.size();i++)
           {
                if(path[i].second=='N')
                {
                     capacity[path[i-1].first][path[i].first]-=min;
                     residualCapacity[path[i].first][path[i-1].first]+=min;
                }
                else
                {
                     //decrease value in the residual edge
                     residualCapacity[path[i-1].first][path[i].first]-=min;
                     capacity[path[i].first][path[i-1].first]+=min;
                }
                
           }

           return;            
      }
      inPath[node]=true;
      
      for(std::pair<int,int>neigh:graph[node])
      {
           //check if the capacity of the node is zero
           if(neigh.first!=parent&&capacity[node][neigh.first]!=0)
           {
                //add this as a normal edge
                path.push_back(std::make_pair(neigh.first,'N'));

                dfs(graph,capacity,residualGraph,residualCapacity,inPath,path,neigh.first,node);
                
                path.pop_back();
                
                //add a residual edge from neigh to node
                residualGraph[neigh.first].insert(node);
                if(parent!=-1)
                {
                     //if this path has rendered the previous edge 0 capa, then we must
                     //go back as we cannot use this edge anymore
                     //inPath[node]=false;
                     char arrived_via = path.back().second;
                     if(arrived_via == 'N' && capacity[parent][node] == 0) 
                     {
                          break;
                     } 
                     else if (arrived_via == 'R' && residualCapacity[parent][node] == 0) 
                     {
                         break;
                     }
                }
           }     
      }

      //now explore the residual edges
      for(int neigh:residualGraph[node])
      {
           //add a residual edge from neigh to node
           if(neigh!=parent&&residualCapacity[node][neigh]!=0)
           {
                path.push_back(std::make_pair(neigh,'R'));

                dfs(graph,capacity,residualGraph,residualCapacity,inPath,path,neigh,node);

                path.pop_back();

                if (parent != -1) 
                {
                     char arrived_via = path.back().second;
                     if (arrived_via == 'N' && capacity[parent][node] == 0) 
                     {
                          break; 
                     } 
                     else if (arrived_via == 'R' && residualCapacity[parent][node] == 0) 
                     {
                          break; 
                     }
                }
           }

      }

      //mark the current node out of the current path
      inPath[node]=false;

      return ;
}

const inline void displayPath(const std::vector<int>&path)
{
      for(int node:path)
      {
           std::cout<<node<<" ";
      }
}

int flowTrace(const std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&flowRate
      ,std::vector<bool>&visited,std::vector<int>&path,int node, int maxAllowedFlow)
{
      if(node==graph.size()-1)
      {
           //this means we have reached the dest via a valid path
           path.push_back(node);
           displayPath(path);
           std::cout<<" max capacity = "<<maxAllowedFlow<<"\n";
           return maxAllowedFlow;
      }
      visited[node]=true;
      path.push_back(node);

      for(std::pair<int,int>neigh:graph[node])
      {
           if(!visited[neigh.first]&&flowRate[node][neigh.first]>0)
           {
                int allowedFlow=std::min(maxAllowedFlow,flowRate[node][neigh.first]);

                int flowPushed=flowTrace(graph,flowRate,visited,path,neigh.first,allowedFlow);

                if(flowPushed>0)
                {
                     //this means this path is  a valid path
                     flowRate[node][neigh.first]-=flowPushed;
                     path.pop_back();
                     visited[node]=false;
                     return flowPushed;
                }
           }
      }
      path.pop_back();
      visited[node]=false;
      return 0;
}


void findPathFlows(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity)
{
      int n=graph.size();
      
      //build up the original capacity matrix
      std::vector<std::vector<int>>originalCapacity(n,std::vector<int>(n,0));

      for(int i=0;i<n;i++)
      {
           for(std::pair<int,int>node:graph[i])
           {
                originalCapacity[i][node.first]=node.second;
           }
      }

      //create the flow matrix to understand how the flow is propagating
      std::vector<std::vector<int>>flowRateMatrix(n,std::vector<int>(n,0));
      
      for(int i=0;i<n;i++)
      {
           for(int k=0;k<n;k++)
           {
                if(capacity[i][k]!=-1)
                {
                     //this means edge exists between i->k and we can calculate the flow
                     //rate
                     flowRateMatrix[i][k]=originalCapacity[i][k]-capacity[i][k];
                }
           }
      }

      //now we can use this flow rate matrix to determin how much flow to send around paths

      while(true)
      {
           std::vector<bool>visited(graph.size(),false);
           std::vector<int>optimalPaths;
     
           int flow=flowTrace(graph,flowRateMatrix,visited,optimalPaths,0,INT_MAX);
           if(flow==0)
           {
                break;
           }
      }
}


void findMaxFlow(std::vector<std::vector<std::pair<int,int>>>&graph,int n,int m)
{
      std::vector<std::vector<int>>capacity(n,std::vector<int>(n,-1));
     
      for(int i=0;i<n;i++)
      {
           for(std::pair<int,int>node:graph[i])
           {
                capacity[i][node.first]=node.second;
           }
      }
      std::vector<std::pair<int,char>>path;
      path.push_back(std::make_pair(0,'N'));
      std::vector<std::unordered_set<int>>residualGraph(n,std::unordered_set<int>());
      std::vector<std::vector<int>>residualCapacity(n,std::vector<int>(n,0));
      std::vector<bool> inPath(n,false);
      int minCapacity=INT_MAX;
      dfs(graph,capacity,residualGraph,residualCapacity,inPath,path,0,-1);
      std::cout<<sum<<"\n";

    
      
      std::cout<<"printing the paths and their max capacity to achieve the max flow\n";

      findPathFlows(graph,capacity);

      std::cout<<"\n\n";
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
           std::vector<std::vector<std::pair<int,int>>>graph(n,std::vector<std::pair<int,int>>(m));

           for(int i=0;i<m;i++)
           {
                int u,v,cap;
                file>>u>>v>>cap;
                graph[u].push_back(std::make_pair(v,cap));
           }
           sum=0;
           findMaxFlow(graph,n,m);
      }
      
}

int main()
{
     takeInput();
     return 0;
}

/*
std::cout<<"printing capacity\n";
           for(std::vector<int>&row:capacity)
           {
                for(int i:row)
                {
                     std::cout<<i<<" ";
                }
                std::cout<<"\n";
           }

           std::cout<<"printing res capacity\n";
           for(std::vector<int>&row:residualCapacity)
           {
                for(int i:row)
                {
                     std::cout<<i<<" ";
                }
                std::cout<<"\n";
           }*/