#include<bits/stdc++.h>
#include "legalRouteChecker_ByTurns.h"

const double INF=std::numeric_limits<double>::infinity();

using pr=std::pair<uint32_t,uint32_t>;

//this is used in the distTable for the {fromNode,roadId} pair part, since
//using -1 for them can cause the number to rotate and fall back to the max 
//limit of uint32_t. Hence we use optional, marking that the field may or maynot be present
using PredecessorEdge=std::pair<std::optional<uint32_t>,std::optional<uint32_t>>;

//this serves as the key for the distTable 
using StateKey=std::pair<PredecessorEdge,uint32_t>;

using HeapState=std::pair<std::pair<uint32_t,std::optional<uint32_t>>,double>;

struct StateHasher
{
     //const here means that the function cannot change the contents of the 
     //object it is handling, meaning it has read only access only. 
     //no except here means that the function is guaranteed to throw no errors 
     //so the internal compiler can make better optimisations in this code
     std::size_t operator() (const State& state) const noexcept
     {
         std::size_t h1=std::hash<uint32_t>{}(state.toArcId);
         std::size_t h2=std::hash<uint32_t>{}(state.fromArcId);
         //now for 3 hash values, to ensure they are spread evenly we use the logic
         //of the golden ratio hash bit mixer.
         std::size_t finalHash=h1;

         //we use the fractional part of the golden ratio converted to a 32bit unsigned
         //integer and add it to our hash, as mathematically it is the most irrational
         //number and adding or multiplying it scrambles the bits and allows even spacing
         //in hashes to prevent collisions
         //the golden ratio part is used as (1+sqrt(5))/2 and just the fraction part of it
         //is taken and multiplied with (2^32) to give us the bit mixer number

         //0x9e3779b9
         //the finalHash>>6 and finalHash<<2 is the avalanche effect  the change in even
         //1 bit will cause a cascade to the rest of the 50 percent of the bits
         finalHash^= h2+ 0x9e3779b9 + (finalHash>>6)+(finalHash<<2);

         return static_cast<std::size_t>(finalHash);
     }
};


//Rule engine Map for figuring out the state and the cost
//that has to be taken for the current edge
//passing StateHasher struct and the functor for the custom hashing of the state obj
std::unordered_map<State,double,StateHasher> ruleEngineMap;


void buildRuleEngine(const std::vector<Arcs>&arcs,
                 const std::vector<TurnCosts>&turnCosts,
                 const std::vector<ForbiddenTransitions>&forbiddenTransactions)
{
     for(const auto& turns:turnCosts)
     {
         uint32_t fromArcId=turns.fromArcId;
         uint32_t toArcId=turns.toArcId;
         double cost=turns.transitionCost;
         State state(fromArcId,toArcId);
         
         ruleEngineMap[state]=cost;
     }

     for(const auto& fr:forbiddenTransactions)
     {
         uint32_t fromArcId=fr.fromArc;
         uint32_t toArcId=fr.toArc;
         double cost=INF;

         State state(fromArcId,toArcId);

         ruleEngineMap[state]=cost;
     }
}

//the cost engine takes the current state->(prevNode,currNode,nextNode)
//and the original cost of the edge and then gives the new cost
double costEngine(std::optional<uint32_t> fromArcIdOptional,uint32_t toArcId,double originalCost)
{
     if(!fromArcIdOptional.has_value())
     {
         //this is the condition for the first node at the start of dijsktra
         return originalCost;
     }
     uint32_t fromArcId=*fromArcIdOptional;
     //build the state
     State state(fromArcId,toArcId);   
     
     //if no rule is defined for this state, we return the original cost
     if(ruleEngineMap.find(state)==ruleEngineMap.end())
     {
         //  std::cout<<"original\n";
         //  std::cout<<"prev = "<<fromArcId<<" curr = "<<toArcId<<" next "<<"\n";
         return originalCost;
     }

     //get the cost to be added
     double addedCost=ruleEngineMap[state];

     //oif the cost is infinity then we simply return infinity
     if(addedCost==INF)
     {
         //  std::cout<<"forbidden\n";
         //std::cout<<"prev = "<<prevNode<<" curr = "<<currNode<<" next "<<nextNode<<"\n";
         return INF;
     }
     //std::cout<<"allowed path but turn cost \n";
     //std::cout<<"prev = "<<prevNode<<" curr = "<<currNode<<" next "<<nextNode<<"\n";
     //return the new cost which is the summation of the added and final cost
     return (addedCost+originalCost);
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
         int n, uint32_t queryStart,uint32_t queryEnd)
{
     //distTable for the  dijsktra
     std::map<StateKey,double>edgeDistTable;

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
     edgeDistTable[{{std::nullopt,std::nullopt},queryStart}]=0;
     
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
     pq.push(std::make_pair(std::make_pair(queryStart,std::nullopt),0));
    
     while(!pq.empty())
     {
         auto top=pq.top();
         pq.pop();
         uint32_t currNode=top.first.first;
         std::optional<uint32_t> fromArcId=top.first.second;
         //std::cout<<"curr node = "<<currNode<<"\n";
         double currDist=top.second;
         for(const auto&neigh:graph[currNode])
         {
             uint32_t nextNode=neigh.first.first;
             uint32_t toArcId=neigh.first.second;
             double originalCost=neigh.second;
             //std::cout<<"next node= "<<nextNode<<"\n";
             //std::cout<<"fromArcId "<<fromArcId<<" toArcId "<<toArcId<<"\n\n";
             double cost=costEngine(fromArcId,toArcId,originalCost);
             if(cost==INF)
             {
                 //forbidden path
                 continue;
             }
             double dist=cost+currDist;
             std::pair<pr,uint32_t> p=std::make_pair(std::make_pair(currNode,toArcId),nextNode);
             if(edgeDistTable.find(p)==edgeDistTable.end()||dist<edgeDistTable[p])
             {
                 //update the distTable
                //  std::cout<<"before assigning\n";
                 edgeDistTable[p]=dist;
                //  std::cout<<"assinging in dist table map\n";
                 //push this node to the queue with the curr node as the prev node
                 pq.push(std::make_pair(std::make_pair(nextNode,toArcId),dist));
             }
             
         }
         std::cout<<"\n\n";
     }
     double minDist=INF;
     for(auto &values:edgeDistTable)
     {
         if(values.first.second==queryEnd)
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
             uint32_t queryStart, uint32_t queryEnd)
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
         uint32_t fromArcId;
         uint32_t toArcId;
         file>>fromArcId>>toArcId;

         forbiddenTransitions.emplace_back(fromArcId,toArcId);
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
             file>>queryStart>>queryEnd;
         }
     }
     file.close();
     process(graph,arcs,turnCosts,forbiddenTransitions,n,arcCount,
         turnCount,forbiddenCount,queryStart,queryEnd);    
}



int main()
{
     takeInput();
     return 0;
}