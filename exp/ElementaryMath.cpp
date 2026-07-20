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