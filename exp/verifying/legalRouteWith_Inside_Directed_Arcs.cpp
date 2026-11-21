#include<bits/stdc++.h>
#include "legalRouteChecker_ByEntireRoute.h"

const double INF=std::numeric_limits<double>::infinity();
int L=2;

using pr=std::pair<uint32_t,uint32_t>;

//this is used in the distTable for the {fromNode,roadId} pair part, since
//using -1 for them can cause the number to rotate and fall back to the max 
//limit of uint32_t. Hence we use optional, marking that the field may or maynot be present
using PredecessorEdge=std::pair<std::optional<std::uint32_t>,std::optional<std::string>>;

//this serves as the key for the distTable 
using DistStateKey=std::pair<uint32_t,std::vector<uint32_t>>;


//this serves as the key for the rule engine map
using RuleStateKey=std::vector<uint32_t>;

//this serves as the state for the priority queue
using HeapState=std::pair<State,double>;


std::size_t vectorHash(const std::vector<uint32_t>& vec,std::size_t initialHash)
{
     for(const auto &id:vec)
     {
         initialHash^= id+ 0x9e3779b9 + (initialHash<<6)+ (initialHash>>2);
     }
     return initialHash;
}

struct DistStateKeyHasher
{
     //const here means that the function cannot change the contents of the 
     //object it is handling, meaning it has read only access only. 
     //no except here means that the function is guaranteed to throw no errors 
     //so the internal compiler can make better optimisations in this code
     std::size_t operator() (const DistStateKey& key) const noexcept
     {
         uint32_t stateKey=key.first;
         std::vector<uint32_t> seq=key.second;
         std::size_t hash=std::hash<uint32_t>{}(stateKey);

         hash=vectorHash(seq,hash);
         return hash;
     }
};


struct RuleStateKeyHasher
{
     std::size_t operator() (const RuleStateKey& key) const noexcept
     {
         std::size_t hash=std::hash<uint32_t>{}(1);
         return vectorHash(key,1);
     }
};


//Rule engine Map for figuring out the state and the cost
//that has to be taken for the current edge
//passing StateHasher struct and the functor for the custom hashing of the state obj
std::unordered_map<RuleStateKey,double,RuleStateKeyHasher> ruleEngineMap;

void buildRuleEngine(const std::vector<Arcs>&arcs,
                 const std::vector<TurnCosts>&turnCosts,
                 const std::vector<ForbiddenTransitions>&forbiddenTransitions)
{

     //for building the rule engine that will have the forbidden states
     //based on any number of arc ids and turn states, we need to track
     //the states the dijkstra is spreading
     //now we do not need to store the entire seq of arc ids for every state
     //we need to store max(1,L-1), where L is the len of the largest
     //forbidden seq.
     //this works, because any seq including the largest one, will now depend
     //on only the L-1 arcId history, not more than that. so we need to keep only
     //that much part, as the hashed key in the map
     for(const auto &tc:turnCosts)
     {
         std::vector<uint32_t> vec{tc.fromArcId,tc.toArcId};
         double cost=tc.transitionCost;
         ruleEngineMap[vec]=cost;
     }
     
     for(const auto &ft:forbiddenTransitions)
     {
         const std::vector<uint32_t>& vec=ft.getForbiddenSeq();
         ruleEngineMap[vec]=INF;
         L=std::max(L,(int)(vec.size()));
     }
}


//the cost engine takes the state, constructs upto L-1 states,
//then checks the rule engine for all the possible rules
//mainly turn costs will involve the currNode and the next node
//but the forbidden rules can take upto L-1 states, so we need 
//to go through the entire L-1 arcId history
double costEngine(State& st,uint32_t toArcId,double originalCost)
{
     std::vector<uint32_t> arcIdHistory;
     std::vector<uint32_t> seq=st.getSeq();
     double cost=0;
     double limit=std::min(st.getSeqLen(),L-1);
     
     //push the next go to arcId
     int index=st.getSeqLen()-1;
     arcIdHistory.push_back(toArcId);
     std::vector<uint32_t> oneIdCheck{toArcId};
     if(ruleEngineMap.find(oneIdCheck)!=ruleEngineMap.end())
     {
         cost=ruleEngineMap[oneIdCheck];
         if(cost==INF)
         {
             return cost;
         }
     }
     for(int i=0;i<limit;i++)
     {
         //push the history arcId now
         arcIdHistory.push_back(seq[index]);

         //if ruleEngineMap has this rule, then we need to check it
         std::vector<uint32_t> key(arcIdHistory.rbegin(),arcIdHistory.rend());
         if(ruleEngineMap.find(key)!=ruleEngineMap.end())
         {
             //if there is a forbidden rule somewhere down the L-1 line
             //then cost becomes INF and the route is automatically forbidden
             //if not, and only turn cost is present, then that is only
             //recorded
             cost=ruleEngineMap[key];
         }
         index--;
     }
     return (originalCost+cost);
}


void buildGraph(const std::vector<Arcs>&arcs, 
         std::vector<std::vector<std::pair<pr,double>>>&graph,
         int n,int arcCount)
{
         
     //iterate over the arcs and form the graph
     graph.resize(n);
     //  std::cout<<"after resize \n";
     for(const auto&arc:arcs)
     {
         uint32_t fromNodeId=arc.fromNode;
         uint32_t toNodeId=arc.toNode;
         double cost=arc.traversalCost;

         graph[fromNodeId].push_back(std::make_pair(std::make_pair(toNodeId,arc.id),cost));
     }
    //  std::cout<<"after building\n";
}


double findMinCostRoute(const std::vector<std::vector<std::pair<pr,double>>>&graph,
         int n, uint32_t queryStart, uint32_t queryEnd)
{
     //distTable for the  dijsktra
     std::map<DistStateKey,double>edgeDistTable;

     //the distTable cannot just store the dist based on curr node, next node
     //since there can be multiple ways btw 2 nodes, we have to save
     //the nodes along with the roads chosen. so the format becomes

     // ((fromNode, roadId), toNode)-> the map saves the shortest dist for this
     //the fromNode= from where we are coming
     //the roadId= via which road we are coming
     //the toNode= to which node we have arrived
     
     //for the start of dijsktra, we take the fromNode and viaRoad 
     //to be std::otpional that signifies no incoming routes for the starting node
     //that is handled by the cost engine
     edgeDistTable[{queryStart,{}}]=0;
     
     //declare the priority queue for the min heap
     //by default priority queue in C++ is a max heap so we need to write a custom
     //lambda function
    
     auto cmp=[&](const HeapState&a,const HeapState&b)->bool
     {
         return a.second>b.second;
     };

     std::priority_queue<HeapState,
         std::vector<HeapState>,
         decltype(cmp)>pq(cmp);
     //push the initial node to the min heap
     State initialState(queryStart);
     pq.push(std::make_pair(initialState,0));
    
     while(!pq.empty())
     {
         auto top=pq.top();
         pq.pop();
         State currState=top.first;
         uint32_t currNode=currState.getCurrNode();
         double currDist=top.second;
         //std::cout<<"curr node = "<<currNode<<"\n";
        
         for(const auto&neigh:graph[currNode])
         {
             std::vector<uint32_t> arcIdHistory=currState.getSeq();
             uint32_t nextNode=neigh.first.first;
             uint32_t toArcId=neigh.first.second;
             double originalCost=neigh.second;
             //std::cout<<"next node= "<<nextNode<<"\n";
             double cost=costEngine(currState,toArcId,originalCost);
             if(cost==INF)
             {
                 //forbidden path
                 continue;
             }
             
             double dist=cost+currDist;
             arcIdHistory.push_back(toArcId);
             if(arcIdHistory.size()>L-1)
             {
                 while(arcIdHistory.size()>L-1)
                 {
                     arcIdHistory.erase(arcIdHistory.begin());
                 }
             }
             DistStateKey key = std::make_pair(nextNode,arcIdHistory);
             
             if(edgeDistTable.find(key)==edgeDistTable.end()
                 ||dist<edgeDistTable[key])
             {
                 State newState(nextNode);
                 newState.appendToPrevStates(arcIdHistory);
                 if(newState.getSeqLen()>L-1)
                 {
                     newState.eraseHistory(L-1);
                 }
                 //newState.pushToPrevStates(toArcId);
                 edgeDistTable[key]=dist;
                 pq.push(std::make_pair(newState,dist));
             }
             //arcIdHistory.pop_back();
         }
         //std::cout<<"\n\n";
     }
     double minDist=INF;
     for(auto &values:edgeDistTable)
     {
         if(values.first.first==queryEnd)
         {
             minDist=std::min(minDist,values.second);
         }
     }
     //node is not reachable
     return minDist;         
}

void process(std::vector<std::vector<std::pair<pr,double>>>&graph,
             const std::vector<Arcs>&arcs,
             const std::vector<TurnCosts>&turnCosts,
             const std::vector<ForbiddenTransitions>&forbiddenTransitions,
             int n,int arcCount,int turnCount,int forbiddenCount,
             std::pair<uint32_t,float>& queryStart,std::pair<uint32_t,float> queryEnd)
{
     //for the forbidden routes and 3 node turn costs, we can make a rule engine
     //for forbidden routes we can give the current route as infinity cost and for
     //the turn transitions we can give the current edge cost + the turn cost

     //build the rule engine
     buildRuleEngine(arcs,turnCosts,forbiddenTransitions);
     std::cout<<"Rule eng success\n";
     //build the graph using arcs
     buildGraph(arcs,graph,n,arcCount);

    
     double minCost=findMinCostRoute(graph,n,queryStart,queryEnd);

     if(minCost==INF)
     {
         std::cout<<"Route is not possible"<<std::endl;
         return;
     }
     std::cout<<"Min cost for route is "<<minCost<<std::endl;
}


std::vector<Arcs> parseArcs(std::ifstream& file,int arc_count)
{   
     std::vector<Arcs> arcs;
     for(int i=0;i<arc_count;i++)
     {
         uint32_t arcId;
         uint32_t fromNodeId;
         uint32_t toNodeId;
         double cost;
         file>>arcId>>fromNodeId>>toNodeId>>cost;
         
         arcs.emplace_back(arcId,fromNodeId,toNodeId,cost);
     }

     return arcs;
}

std::vector<TurnCosts> parseTurnCosts(std::ifstream& file,int count)
{
     std::vector<TurnCosts> turnCosts;
     
     for(int i=0;i<count;i++)
     {
         uint32_t fromArcId;
         uint32_t toArcId;
         double cost;
         file>>fromArcId>>toArcId>>cost;
         
         turnCosts.emplace_back(fromArcId,toArcId,cost);
     }

     return turnCosts;
}

std::vector<ForbiddenTransitions> parseForbiddenTransitions(std::ifstream& file,int count)
{
     std::vector<ForbiddenTransitions> forbiddenTransitions;

     for(int i=0;i<count;i++)
     {
         uint32_t seqLen;
         file>>seqLen;
         ForbiddenTransitions ft;
         for(int k=0;k<seqLen;k++)
         {
             uint32_t arcId;
             file>>arcId;
             ft.addToForbiddenSeq(arcId);
         }
         forbiddenTransitions.emplace_back(ft);
     }
     return forbiddenTransitions;
}

void takeInput()
{
     std::ifstream file("input.txt");
     std::string line;
     int n=0,arcCount=0,forbiddenCount=0,turnCount=0;
     uint32_t queryStart=-1, queryEnd=-1; 
     std::vector<std::vector<std::pair<pr,double>>>graph;
     std::vector<Arcs> arcs; 
     std::vector<ForbiddenTransitions> forbiddenTransitions;
     std::vector<TurnCosts> turnCosts;
     //parse the file
     
     while(file>>line)
     {
         if(line=="NODES")
         {
             //this is the nodes section
             //we get the count and move the file pointer that many places
             int count;
             file >> count; // Reads the number right after "NODES"
             n = count;
             // Properly consume the node details (id and name)
             for (int i = 0; i < count; i++)
             {
                 uint32_t nodeId;
                 std::string nodeName;
                 file >> nodeId >> nodeName;
             }
         }
         else if(line=="ARCS")
         {
             file>>arcCount;
             arcs=parseArcs(file,arcCount);
         }
         else if(line=="TURN_COSTS")
         {
             file>>turnCount;
             turnCosts=parseTurnCosts(file,turnCount);
         }
         else if(line=="FORBIDDEN_TRANSITIONS")
         {
             file>>forbiddenCount;
             forbiddenTransitions=parseForbiddenTransitions(file,forbiddenCount);
         }
         else if(line=="QUERY")
         {
             std::pair<uint32_t,float> q1;
             std::pair<uint32_t,float>q2;
             file>>q1.first>>q1.second;
             file>>q2.first>>q2.second;
         }
     }
     file.close();
     process(graph,arcs,turnCosts,forbiddenTransitions,n,arcCount,
         turnCount,forbiddenCount,q1,q2);    
}



int main()
{
     takeInput();
     return 0;
}