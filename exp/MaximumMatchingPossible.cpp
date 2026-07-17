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

void findMaxMatching(std::vector<std::vector<int>>&applications,std::vector<bool>&jobs)
{
     int maxMatch=0;
     findMax(applications,jobs,0,0,maxMatch);
     std::cout<<maxMatch<<"\n";    
}

void takeInput()
{
     std::ifstream file("input.txt");
     int numApp, numJob, numList;
     file>>numApp>>numJob>>numList;
     
     std::vector<std::vector<int>>applications(numApp,std::vector<int>());
     std::vector<bool> jobMark(numJob,false);
    
     for(int i=0;i<numList;i++)
     {
         int appId,jobId;
         file>>appId>>jobId;
         applications[appId].push_back(jobId);
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
     findMaxMatching(applications,jobMark);
}   

int main()
{
     takeInput();
     return 0;
}