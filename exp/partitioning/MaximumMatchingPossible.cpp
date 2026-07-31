#include<bits/stdc++.h>

void findMax(std::vector<std::vector<int>>&applications,std::vector<bool>&jobs,int appId,int matches,int &maxMatching)
{
     if(appId==applications.size())
     {
         //std::cout<<"max = "<<matches<<"\n";
         maxMatching=std::max(maxMatching,matches);
         return;
     }
     bool found=false;
     for(int job:applications[appId])
     {
         if(!jobs[job])
         {
             found=true;
             //std::cout<<"app id "<<appId<<" job id = "<<job<<"\n";
             //this job has not been taken by anyone till now, so we can match this for
             //the current candidate and increase the match count
             jobs[job]=true;
             findMax(applications,jobs,appId+1,matches+1,maxMatching);
             //after returning from this selection mark this job as not matched for the next
             //combination
             jobs[job]=false;

         }
     }  
     //this accounts for the case when no job is matched
     if(!found)
     {
         findMax(applications, jobs, appId + 1, matches, maxMatching);
     }
}

//the recursive combinatorial problem complexity is too high in factorial and will fail on large test cases
//instead we use a better way to solve the problem.
//this maximum matching, that we have to achieve, every appId has to be connected to a jobId in such a 
//manner that we have the max number of matches, keeping restrictions that 1 appId can connect to only 1 
//jobId. 
//now in network flow what we do is we define a capacity to edges and we determine how much capacity can
//the entire network carry, which is in a way the max flow that arrives on the sink through the network
//the matching problem is the same. we have been given restrictions and we have to follow them and find 
//how much max we can achieve
//now when we look at the matching from a network flow view, we can see that if we allow the edges to have
//wt=1, it satisfies the property that only 1 candidate is selected for  a job. the augmenting paths we will
//find ensures that the candidates are matched to proper jobs of their liking as we build the graph accodingly

//now to simulate the network flow in this problem, we have to add 2 dummy nodes in the graph, which is source
//and the sink. after doing that, we just treat it as a normal network flow graph problem

typedef struct Node
{
     //stores id of node
     int id;
     //stores what the node is job or applicant
     char type;

     Node(int id,char type)
     {
         this->id=id;
         this->type=type;
     }
}Node;


int findMaxFlow(std::unordered_map<Node*,std::unordered_set<Node*>>&network,std::unordered_map<Node*,std::unordered_map<Node*,int>>&capacity
     ,std::unordered_map<Node*,bool>&visited,Node* node,int pushedFlow)
{
     //std::cout<<node->id<<" "<<node->type<<"\n";
     if(node->type=='T')
     {
         //we have reached the sink through a valid path
         //std::cout<<"hello\n";
         //maxFlow=std::max(maxFlow,pushedFlow);
         return 1;
     }
     visited[node]=true;
     for(Node* neigh:network[node])
     {
         //std::cout<<"node = "<<node->id<<" neigh = "<<neigh->id<<" capa = "<<capacity[node][neigh]<<"\n";
         if(!visited[neigh]&&capacity[node][neigh]>0)
         {
             //std::cout<<"hello\n";
             int maxPush=findMaxFlow(network,capacity,visited,neigh,1);

             if(maxPush>0)
             {
                 //update the edges
                 capacity[node][neigh]-=1;
                 capacity[neigh][node]+=1;

                 return maxPush;
             }
         }
     }
     return 0;
}


void findMaxMatching(std::vector<std::vector<int>>&applications,std::vector<bool>&jobs
     ,std::unordered_map<int,Node*>&jobMap)
{
     int maxMatch=0;
     findMax(applications,jobs,0,0,maxMatch);
     std::cout<<maxMatch<<"\n";    

     //network
     std::unordered_map<Node*,std::unordered_set<Node*>>network;

     //capacity matrix
     std::unordered_map<Node*,std::unordered_map<Node*,int>>capacity;
     //dummy nodes
     Node* source= new Node(-1,'S');
     Node* sink= new Node(-1,'T');
     
     for(int i=0;i<applications.size();i++)
     {
         //create the new node for applicant
         Node* newAppNode= new Node(i,'A');
         network[source].insert(newAppNode);
         capacity[source][newAppNode]=1;
         capacity[newAppNode][source]=0;
         for(int jobs:applications[i])
         {
             //normal edge from app to job
             network[newAppNode].insert(jobMap[jobs]);
             //residual edge from job to app
             network[jobMap[jobs]].insert(newAppNode);
             //attach sink to the job nodes
             network[jobMap[jobs]].insert(sink);
             //mark the edge as wt=1
             capacity[newAppNode][jobMap[jobs]]=1;

             capacity[jobMap[jobs]][newAppNode]=0;

             capacity[jobMap[jobs]][sink]=1;

             capacity[sink][jobMap[jobs]]=0;
         }
     }
     int max=0;
     int sum=0;
     while(true)
     {
         std::unordered_map<Node*,bool> visited;
         for(auto key:network)
         {
             visited[key.first]=false;
         }
         int flow=findMaxFlow(network,capacity,visited,source,1);

         if(flow==0)
         {
             break;
         }
         sum+=flow;
     }
     std::cout<<sum<<"\n";
}

void takeInput()
{
     std::ifstream file("input.txt");
     int numApp, numJob, numList;
     file>>numApp>>numJob>>numList;
     
     std::vector<std::vector<int>>applications(numApp,std::vector<int>());
     std::vector<bool> jobMark(numJob,false);
     std::unordered_map<int,Node*> jobMap;
     for(int i=0;i<numList;i++)
     {
         int appId,jobId;
         file>>appId>>jobId;
         applications[appId].push_back(jobId);
         if(jobMap.find(jobId)==jobMap.end())
         {
             Node* newJob= new Node(jobId,'J');
             jobMap[jobId]=newJob;
         }
     }
     for(int i=0;i<numApp;i++)
     {
         std::cout<<i<<" -> ";
         for(int job:applications[i])
         {
             std::cout<<job<<" ";
         }
         std::cout<<"\n";
     }
     findMaxMatching(applications,jobMark,jobMap);
}   

int main()
{
     takeInput();
     return 0;
}