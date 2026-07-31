#include<bits/stdc++.h>

void build(std::vector<std::pair<double,double>>&tree,std::vector<std::pair<double,double>>&coord
     ,uint32_t index,int l,int h,bool alongXAxis)
{
     //std::cout<<"l = "<<l<<" h = "<<h<<"\n";
     if(l>h||index<0||index>=tree.size())
     {
         return;
     }

     if(l==h)
     {
         tree[index]=coord[l];
         return;
     }

     auto cmpX=[&](std::pair<double,double>&c1,std::pair<double,double>&c2)->bool
     {
         return c1.first<c2.first;
     };

     auto cmpY=[&](std::pair<double,double>&c1,std::pair<double,double>&c2)->bool
     {
         return c1.second<c2.second;
     };

     if(alongXAxis)
     {
         std::sort(coord.begin()+l,coord.begin()+h+1,cmpX);
     }
     else
     {
         std::sort(coord.begin()+l,coord.begin()+h+1,cmpY);
     }

     uint32_t mid=(l+h)/2;

     tree[index]=coord[mid];
     
     //recursively build both sides of the tree
     //storing the tree in array format so we have the left child at 2*index+1 and the right
     //child at 2*index+2
     build(tree,coord,2*index+1,l,mid-1,!alongXAxis);
    
     build(tree,coord,2*index+2,mid+1,h,!alongXAxis);
}


void query(const std::vector<std::pair<double,double>>&tree,const double x,const double y,
     std::pair<double,double>&ans,double &minDist,uint32_t index,bool alongXAxis)
{
     if(index >= tree.size() || tree[index].first == std::numeric_limits<double>::infinity())
     {
         return ;
     }       

     double nodeX=tree[index].first;
     double nodeY=tree[index].second;

     double currDist=std::sqrt(std::pow(x-nodeX,2)+std::pow(y-nodeY,2));

     if(currDist<minDist)
     {
         minDist=currDist;
         ans.first=nodeX;
         ans.second=nodeY;
     }

     double ortho_dist=alongXAxis?x-nodeX:y-nodeY;

     uint32_t probableIndex= ortho_dist<0?2*index+1:2*index+2;
     uint32_t diffSideIndex= ortho_dist<0?2*index+2:2*index+1;

     query(tree,x,y,ans,minDist,probableIndex,!alongXAxis);

     if(std::abs(ortho_dist)<minDist)
     {
         query(tree,x,y,ans,minDist,diffSideIndex,!alongXAxis);
     }
}

void build_kd_tree(std::vector<std::pair<double,double>>&coord,double x,double y,int n)
{
     double INF = std::numeric_limits<double>::infinity();
     std::vector<std::pair<double,double>> tree(4 * n, {INF, INF});
     
     int l = 0;
     int h = coord.size() - 1;
     
     build(tree,coord,0,l,h,true);
     
     std::pair<double,double> ans={-1,-1};
     double min=INF;
     query(tree,x,y,ans,min,0,true);

     std::cout<<ans.first<<" "<<ans.second<<"\n";
}

void takeInput()
{
     std::ifstream file("input.txt");

     int test;
     file>>test;

     while(test--)
     {
         int n;
         double qx,qy;
         file>>qx>>qy;
         file>>n;
         std::vector<std::pair<double,double>>coord;
         for(int i=0;i<n;i++)
         {
             double x,y;
             file>>x>>y;
             coord.push_back(std::make_pair(x,y));
         }
         //std::cout<<"ans\n";
         build_kd_tree(coord,qx,qy,n);
     }
}

int main()
{
     takeInput();
     return 0;
}