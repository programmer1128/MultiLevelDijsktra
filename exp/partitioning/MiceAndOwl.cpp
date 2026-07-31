/*
The gopher family, having averted the canine threat, must face a new predator.

The are 
 gophers and 
 gopher holes, each at distinct 
 coordinates. A hawk arrives and if a gopher does not reach a hole in 
 seconds it is vulnerable to being eaten. A hole can save at most one gopher. All the gophers run at the same velocity 
. The gopher family needs an escape strategy that minimizes the number of vulnerable gophers.

Input
The input contains several cases. The first line of each case contains four positive integers less than 100: 
, 
, 
, and 
. The next 
 lines give the coordinates of the gophers; the following 
 lines give the coordinates of the gopher holes. All coordinates are decimal numbers between 0 and 100 with one digit after the decimal point. All distances are in metres; all times are in seconds; all velocities are in metres per second.

Output
Output consists of a single line for each case, giving the number of vulnerable gophers.*/
#include<bits/stdc++.h>


int findMaxFlow(std::vector<std::vector<int>>&graph,std::vector<std::vector<int>>&capacity
     ,std::vector<bool>&visited,int node,int maxFlow)
{
     if(node==graph.size()-1)
     {
         //we have reached the sink via a valid path
         return maxFlow;
     }
     visited[node]=true;

     for(int neigh:graph[node])
     {
         if(!visited[neigh]&&capacity[node][neigh]>0)
         {
             int allowedFlow=std::min(capacity[node][neigh],maxFlow);
             int pushedFlow=findMaxFlow(graph,capacity,visited,neigh,allowedFlow);
             //std::cout<<"hello\n";
             if(pushedFlow>0)
             {
                 //this is a valid augmented path
                 capacity[node][neigh]-=pushedFlow;
                 capacity[neigh][node]+=pushedFlow;
                 return pushedFlow;
             }
         }
     }

     return 0;
}


//this problem is essentially about finding out a combination that alllows max mice to be saved from the
//hawk, following some restrictions about where the mice can go, how many can go. so essentially this is
//a network flow problem where we have to find the max flow in this mice and hole network

void solve(std::vector<std::pair<double,double>>&micePositions,std::vector<std::pair<double,double>>&holePositions
     ,int n,int m,int s,int v)
{
     //first we need to determine which mice can go in which hole
     //so we form a directed graph of the mice and the hole in which they can go
     int maxNode=std::max(n,m);
     double rSq=std::pow((v*s),2);
     std::vector<std::vector<int>>graph(n+m+2);
     std::vector<std::vector<int>>capacity(n+m+2,std::vector<int>(n+m+2,0));
     int sink=n+m+1;
     for (int j = 0; j < m; j++) 
     {
         int holeId = n + 1 + j;
         //mark the edge between hole and sink as 1
         capacity[holeId][sink] = 1; 
         //add the normal edge
         graph[holeId].push_back(sink);
         //add residual edge
         graph[sink].push_back(holeId);
     }

     for(int i=0;i<n;i++)
     {
         //for s seconds the mice can run with v speed in any directions. so the mice can
         //cover the area of circle of radius v*s. the eqn of this circle will be 
         //S:(x-a)^2+(y-b)^2-r^2=0. now if this exp S<=0 for a point then that lies within the 
         //circle boundary and mice can reach. so we add a edge between the mice and the hole 
         double a=micePositions[i].first;
         double b=micePositions[i].second;
         int holeId=0;
         //mark the edge between the dummy source and the mouse node to be 1
         capacity[0][i+1]=1;
         graph[0].push_back(i+1);
         graph[i+1].push_back(0);
         for(std::pair<double,double>&coor:holePositions)
         {
             double x=coor.first;
             double y=coor.second;

             double s=std::pow((x-a),2)+std::pow((y-b),2)-rSq;
             
             if(s<=0)
             {
                 //std::cout<<"hello\n";
                 //that means the mouse can reach this point
                 //add normal edge
                 graph[i+1].push_back(holeId+n+1);
                 //add the residual edge
                 graph[holeId+n+1].push_back(i+1);
                 //mark the normal edge with capacity=1
                 capacity[i+1][holeId+n+1]=1;
             }
             holeId++;
         }
     }
     int maxMiceSaved=0;
     while(true)
     {
         std::vector<bool>visited(n+m+2,false);
         int pushedFlow=findMaxFlow(graph,capacity,visited,0,INT_MAX);

         if(pushedFlow==0)
         {
             //the network has reached maxflow
             break;
         }
         maxMiceSaved+=pushedFlow;
     }
     std::cout<<maxMiceSaved<<"\n";
}

void takeInput()
{
     //std::freopen("input.txt","r",stdin);

     int n,m,s,v;
     std::cin>>n>>m>>s>>v;

     std::vector<std::pair<double,double>>micePositions(n);
     std::vector<std::pair<double,double>>holePositions(m);
     for(int i=0;i<n;i++)
     {
         double x,y;
         std::cin>>x>>y;
         micePositions[i]=std::make_pair(x,y);
     }

     for(int i=0;i<m;i++)
     {
         double x,y;
         std::cin>>x>>y;
         holePositions[i]=std::make_pair(x,y);
     }
     solve(micePositions,holePositions,n,m,s,v);
}

int main()
{
     takeInput();
     return 0;
}