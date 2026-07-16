#include<bits/stdc++.h>


void display(std::vector<std::pair<int,char>>&path)
{
      std::cout<<"printint path\n";
      for(const auto&r:path)
      {
           std::cout<<"("<<r.first<<","<<r.second<<") ";
      }
}

int sum=0;

void dfs(std::vector<std::vector<std::pair<int,int>>>&graph,std::vector<std::vector<int>>&capacity
      ,std::vector<std::vector<int>>&residualGraph,std::vector<std::vector<int>>&residualCapacity
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
           residualGraph[node].push_back(parent);
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
           std::cout<<"\n";
           std::cout<<"\nmin capa = "<<min<<" returning\n\n";

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
                residualGraph[neigh.first].push_back(node);
                if(parent!=-1&&capacity[parent][node]==0)
                {
                     //if this path has rendered the previous edge 0 capa, then we must
                     //go back as we cannot use this edge anymore
                     //inPath[node]=false;
                     std::cout<<"node = "<<node<<" parent = "<<parent<<" nor edge= "<<capacity[parent][node]<<"\n";
                     break;
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

                if(parent!=-1&&residualCapacity[parent][node]==0)
                {
                     //if this path has rendered the previous edge 0 capa, then we must
                     //go back as we cannot use this edge anymore
                     std::cout<<"node = "<<node<<" parent = "<<parent<<" res edge= "<<residualCapacity[parent][node]<<"\n";
                     break;
                }
           }

      }

      //mark the current node out of the current path
      inPath[node]=false;

      return ;
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
      std::vector<std::vector<int>>residualGraph(n,std::vector<int>(n));
      std::vector<std::vector<int>>residualCapacity(n,std::vector<int>(n,0));
      std::vector<bool> inPath(n,false);
      int minCapacity=INT_MAX;
      dfs(graph,capacity,residualGraph,residualCapacity,inPath,path,0,-1);
      std::cout<<sum<<"\n";

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
      }
      findMaxFlow(graph,n,m);
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