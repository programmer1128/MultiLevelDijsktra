#include<bits/stdc++.h>

//brute force way to solve the problem is to try out all the combibations for
//every pair, while maintaining a hash set of the seen answers

struct PairHash
{
     template<class T1,class T2>
     std::size_t operator() (const std::pair<T1,T2>&p) const
     {
         auto h1=std::hash<T1>{}(p.first);
         auto h2=std::hash<T2>{}(p.second);
         return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
     }   

     std::size_t operator()(const std::pair<int, std::pair<int, int>>& p) const {
        auto h1 = std::hash<int>{}(p.first);
        auto h2 = std::hash<int>{}(p.second.first);
        auto h3 = std::hash<int>{}(p.second.second);
        
        // Combine h1 and h2
        std::size_t hash = h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        // Combine the result with h3
        hash ^= h3 + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        
        return hash;
    }
};

bool insertMap(int val,std::unordered_map<int,std::pair<std::pair<int,int>,char>>&map)
{
     if(map.find(val)==map.end())
     {
         return true;
     }
     return false;
}

typedef struct Pairing
{
     std::pair<int,int> pairs;
     char op;
     int res;

     Pairing(std::pair<int,int>pairs,char op,int res)
     {
         this->pairs=pairs;
         this->op=op;
         this->res=res;
     }
}Pairing;

bool findValidPairing(std::vector<std::pair<int,int>>&pairs,std::unordered_set<int>&set,
     std::vector<Pairing>&pairing,int index)
{
     if(index==pairs.size())
     {
         return true;
     }    
     std::pair<int,int>p=pairs[index];
     int val[]={p.first+p.second,p.first-p.second,p.first*p.second};
     char op[]={'+','-','*'};

     for(int i=0;i<3;i++)
     {
         if(set.find(val[i])==set.end())
         { 
             Pairing newPair(p,op[i],val[i]);
             pairing.push_back(newPair);
             set.insert(val[i]);
             if(findValidPairing(pairs,set,pairing,index+1))
             {
                 return true;
             }
             set.erase(val[i]);
             pairing.pop_back();
         }
     }
     return false;
}

//now recursive brute force explores 3 possibilities for every pair. that will give us TLE
//as it will have 3^n complexity.
//better way to solve this problem is to again think of bipartite matching. we have 2 sets
//pairs and the values they produce. we can form a network of these values and then connect them
//to the dummy source and the sink nodes and then apply max flow to find out how many unique 
//max combinations can we generate. if the max is less than the number of pairs given to us then we can
//be sure that the pairings do not have a possible ans
typedef struct Node
{
     //for the pairing nodes
     std::pair<int,int>p;
     //for the operation result nodes
     int val;
     //for denoting the type of nodes
     char type;

     Node()
     {

     }

     Node(std::pair<int,int>p,int val,char type)
     {
         this->p=p;
         this->val=val;
         this->type=type;
     }

     Node(int val,char type)
     {
         this->val=val;
         this->type=type;
     }
}Node;

void findCombination(std::vector<std::pair<int,int>>&pairs)
{
     std::unordered_set<int> set;
     std::vector<Pairing>pairing;

     if(!findValidPairing(pairs,set,pairing,0))
     {
         std::cout<<"Impossible\n";
         return;
     }
     std::cout<<"Possible\n";

     for(const auto& p:pairing)
     {
         std::cout<<p.pairs.first<<p.op<<p.pairs.second<<"="<<p.res<<"\n";
     }
}


int maxFlow(std::map<Node*,std::vector<std::pair<Node*,char>>>&network
     ,std::unordered_map<Node*,std::unordered_map<Node*,int>>&capacity,std::unordered_map<Node*,bool>&visited
     ,Node* node,int pushedFlow)
{
     if(node->type=='T')
     {
         //we have reached the sink through a valid path
         return pushedFlow;
     }
     visited[node]=true;

     for(std::pair<Node*,char>&neigh:network[node])
     {
         if(!visited[neigh.first]&&capacity[node][neigh.first]>0)
         {
             int allowedFlow=std::min(pushedFlow,capacity[node][neigh.first]);

             int flow=maxFlow(network,capacity,visited,neigh.first,allowedFlow);

             if(flow>0)
             {
                 //decrease the capacity of the normal edge by min allowed flow
                 capacity[node][neigh.first]-=pushedFlow;
                 //add the capacity to the residual edge
                 capacity[neigh.first][node]+=pushedFlow;

                 return flow;
             }
         }
     }
     //for no path to sink
     return 0;
}


void findCombination2(std::vector<std::pair<int,int>>&pairs)
{
     //this is the  network of the pair  nodes and the resultant values that we get
     //from performing operations on them
     std::map<Node*,std::vector<std::pair<Node*,char>>>network;
     std::unordered_map<int,Node*>map;
     std::unordered_map<Node*,std::unordered_map<Node*,int>>capacity;
     char op[]={'+','-','*'};
     //building the network

     Node* source=new Node(-1,'S');
     Node* sink=new Node(-1,'T');
     std::vector<Node*> orderedPairs;
     for(std::pair<int,int>&p:pairs)
     {
         Node* newNode= new Node(p,-1,'P');   
         orderedPairs.push_back(newNode);
         //add the edge between the source and pair nodes
         network[source].push_back(std::make_pair(newNode,'F'));
         //mark the edge with capacity 1
         capacity[source][newNode]=1;
         //mark residual edge with capacity=0;
         capacity[newNode][source]=0;
         int val[]={p.first+p.second,p.first-p.second,p.first*p.second};
         for(int i=0;i<3;i++)
         {
             if(map.find(val[i])==map.end())
             {
                 map[val[i]]=new Node(val[i],'R');
                 //add edge between the res node and the sink
                 network[map[val[i]]].push_back(std::make_pair(sink,'F'));
                 //add the capacity of the edge to be 1
                 capacity[map[val[i]]][sink]=1;
                 //mark the capacity of the residual edge to be zero
                 capacity[sink][map[val[i]]]=0;
             }
             //add edge between the pair node and the res node
             network[newNode].push_back(std::make_pair(map[val[i]],op[i]));
             //mark the original capacity of the node as 1
             capacity[newNode][map[val[i]]]=1;

             network[map[val[i]]].push_back(std::make_pair(newNode,op[i]));
             //mark the capacity of the residual edge as 0
             capacity[map[val[i]]][newNode]=0;
         } 
     }

     int maxFlowInNetwork=0;
     while(true)
     {
         std::unordered_map<Node*,bool>visited;
         
         for(auto key:network)
         {
             visited[key.first]=false;
         }
         int pushedFlow=maxFlow(network,capacity,visited,source,INT_MAX);

         if(pushedFlow==0)
         {
             break;
         }
         maxFlowInNetwork+=pushedFlow;
     }

     if(maxFlowInNetwork!=pairs.size())
     {
         //all the pairs are not satisfied and there is no solution
         std::cout<<maxFlowInNetwork<<"\n";
         std::cout<<"Impossible\n";
         return;
     }

     //std::cout<<"Possible\n";

     for(Node* pNode : orderedPairs)
     {
         for(std::pair<Node*,char>&neigh : network[pNode])
         {
             // Skip the residual connection back to the Super Source (if it exists)
             if(neigh.first->type != 'S' && capacity[pNode][neigh.first] == 0)
             {
                 std::cout << pNode->p.first << neigh.second << pNode->p.second << "=" << neigh.first->val << "\n";
                 break;
             }
         }
     }
}

void takeInput()
{
     std::freopen("input.txt","r",stdin);
     int n;
     std::cin>>n;
     std::vector<std::pair<int,int>>input;
     
     for(int i=0;i<n;i++)
     {
         int u,v;
         std::cin>>u>>v;
         input.push_back(std::make_pair(u,v));
     }
     findCombination(input);
     findCombination2(input);
}

int main()
{
     takeInput();
     return 0;
}

/*
FAILED GREEDY APPROACH

std::unordered_map<int,std::pair<std::pair<int,int>,char>>map;
     std::unordered_map<std::pair<int,std::pair<int,int>>,bool,PairHash> visited;
     int id=0;
     for(std::pair<int,int>&p:pairs)
     {
         int arr[]={p.first+p.second,p.first-p.second,p.first*p.second};
         char op[]={'+','-','*'};
         visited[std::make_pair(id,p)]=false;
         std::pair<int,std::pair<int,int>>uniqPair=std::make_pair(id,p);
         for(int i=0;i<3;i++)
         {
             if(insertMap(arr[i],map))
             {
                 map[arr[i]]=std::make_pair(p,op[i]);
                 if(!visited[uniqPair])
                 {
                     visited[uniqPair]=true;
                 }
             }
         }
         id++;
     }

     //now after we have find all the distinct values possible, we need to see if we have covered 
     //all the pairs or not
     id=0;
     for(std::pair<int,int>&p:pairs)
     {
         std::pair<int,std::pair<int,int>>uniqPair=std::make_pair(id,p);
         if(!visited[uniqPair])
         {
             //this pair is not included so no solution
             std::cout<<"Impossible\n";
             return;
         }
         id++;
     }
     std::cout<<"Possible\n";*/

/*

     std::cout<<"Printing the network\n";
     for(auto nodes:network)
     {
         if(nodes.first->type=='P')
         {
             std::cout<<nodes.first->p.first<<" "<<nodes.first->p.second<<" -> ";
         }
         else
         {
             std::cout<<nodes.first->type<<" -> ";
         }
         for(std::pair<Node*,char>neigh:network[nodes.first])
         {
             if(neigh.first->type=='R')
             {
                 std::cout<<neigh.first->val<<" ";
             }
             else
             {
                 std::cout<<neigh.first->type<<" ";
             }
         }
         std::cout<<"\n";
     }
*/