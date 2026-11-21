#pragma once
#include<cstdint>

struct Arcs
{
     uint32_t id;     
     uint32_t fromNode;
     uint32_t toNode;
     double traversalCost;

     Arcs()=default;

     Arcs(uint32_t id,uint32_t fromNode,uint32_t toNode,double cost)
     {
         this->id=id;
         this->fromNode=fromNode;
         this->toNode=toNode;
         this->traversalCost=cost;
     }
};

struct TurnCosts
{
     uint32_t fromArcId;
     uint32_t toArcId;
     double transitionCost;

     TurnCosts(uint32_t fromArcId,uint32_t toArcId, double cost)
     { 
         this->fromArcId=fromArcId;
         this->toArcId=toArcId;
         this->transitionCost=cost;
     }
};

struct ForbiddenTransitions
{
     uint32_t fromArc;
     uint32_t toArc;

     ForbiddenTransitions(uint32_t fromArc,uint32_t toArc)
     {
         this->fromArc=fromArc;
         this->toArc=toArc;
     }
};

struct State
{
     uint32_t fromArcId;
     uint32_t toArcId;
     State(uint32_t fromArcId,uint32_t toArcId)
     {
           this->fromArcId=fromArcId;
           this->toArcId=toArcId;
     }
     bool operator == (const State& other) const
     {
         return other.fromArcId==this->fromArcId&&other.toArcId==this->toArcId;
     }
};
