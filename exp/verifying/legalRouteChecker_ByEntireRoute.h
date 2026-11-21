#pragma once
#include<cstdint>
#include<vector>
#include<optional>
#include <deque>

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
     private:
     std::vector<uint32_t> forbiddenSeq;
     
     public:

     ForbiddenTransitions()=default;

     void addToForbiddenSeq(uint32_t arcId)
     {
         this->forbiddenSeq.push_back(arcId);
     }

     const std::vector<uint32_t>& getForbiddenSeq() const
     {
         return this->forbiddenSeq;
     }
};

struct State
{
     private:
     std::vector<uint32_t> prevStates;
     uint32_t currNode;

     public:
     State(uint32_t currNode)
     {
         this->currNode=currNode;
     }

     void pushToPrevStates(uint32_t arcId)
     {
         this->prevStates.push_back(arcId);
     }

     void appendToPrevStates(std::vector<uint32_t>&vec)
     {
         this->prevStates.reserve(this->prevStates.size()+vec.size());
         this->prevStates.insert(this->prevStates.end(),vec.begin(),vec.end());
     }

     void removeOldArcIdFromPrevStates()
     {
         this->prevStates.erase(this->prevStates.begin());
     }

     std::vector<uint32_t>& getSeq()
     {
         return this->prevStates;
     }

     int getSeqLen()
     {
         return (int)(this->prevStates.size());
     }

     uint32_t getCurrNode()
     {
         return this->currNode;
     }

     void eraseHistory(int L)
     {
         while(this->prevStates.size()>L)
         {
             removeOldArcIdFromPrevStates();
         }
     }
 
     bool operator == (const State& other) const
     {
         if(other.currNode!=this->currNode)
         {
             return false;
         }

         std::vector<uint32_t> pOther=other.prevStates;
         std::vector<uint32_t> curr=this->prevStates;
         if(pOther.size()!=curr.size())
         {
             return false;
         }

         for(int i=0;i<curr.size();i++)
         {
             if(pOther[i]!=curr[i])
             {
                 return false;
             }
         }

         return true;
     }
};
