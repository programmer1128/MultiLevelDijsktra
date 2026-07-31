#include<bits/stdc++.h>

typedef struct Node
{
      double x;
      double y;

      Node* left;
      Node* right;

      Node(double x,double y)
      {
           this->x=x;
           this->y=y;
           this->left=this->right=nullptr;
      }
}Node;

Node* build(Node* root,std::vector<std::pair<double,double>>&coordinates,int l,int h,bool alongXAxis)
{
      //std::cout<<"l = "<<l<<" h = "<<h<<"\n";
      if(l>h)
      {
           return nullptr;
      }
      if(l==h)
      {
           //this means there is only 1 element
           root= new Node(coordinates[l].first,coordinates[l].second);
           return root;
      }

      auto cmpX=[&](std::pair<double,double>&c1,std::pair<double,double>&c2)->bool
      {
           return c1.first<c2.first;
      };

      auto cmpY=[&](std::pair<double,double>&c1,std::pair<double,double>&c2)->bool
      {
           return c1.second<c2.second;
      };
      //std::cout<<"hello\n";
      if(alongXAxis)
      {
           std::sort(coordinates.begin()+l,coordinates.begin()+h+1,cmpX);
      }
      else
      {
           std::sort(coordinates.begin()+l,coordinates.begin()+h+1,cmpY);
      }
      int mid=(l+h)/2;
      //build up the new sub root using the mid element along the coordinate axis
      //std::cout<<coordinates[mid].first<<"\n";
      root= new Node(coordinates[mid].first,coordinates[mid].second);

      //recursively go left and right inverting the axis
      root->left=build(root->left,coordinates,l,mid-1,!alongXAxis);
      root->right=build(root->right,coordinates,mid+1,h,!alongXAxis);

      return root;
}

void query(Node* root,double x, double y,std::pair<double,double>&ans,double& mindist,bool alongXAxis)
{
      if(root==nullptr)
      {
           return;
      }

      double currDist=std::sqrt(std::pow(x-root->x,2)+std::pow(y-root->y,2));

      if(currDist<mindist)
      {
           mindist=currDist;
           ans.first=root->x;
           ans.second=root->y;
      }

      //check which axis this node was split and accordingly we can check the node we need
      //to move next
      double ortho_dist=alongXAxis?x-root->x:y-root->y;
      //this is the side where the node most probably is 
      Node* probableSide= ortho_dist<0?root->left:root->right;

      //this is the diff side. we need this side because it may happen that the point is just
      //at the boundary of the axis
      Node* diffSide=ortho_dist<0?root->right:root->left;

      query(probableSide,x,y,ans,mindist,!alongXAxis);

      if(std::abs(ortho_dist)<mindist)
      {
           query(diffSide,x,y,ans,mindist,!alongXAxis);
      }
}


void bfs(Node* root)
{
      std::queue<Node*> queue;

      queue.push(root);

      while(!queue.empty())
      {
           int current=queue.size();
           while(current--)
           {
                Node* node=queue.front();
                if(node==nullptr)
                {
                     std::cout<<"error\n";
                     exit(1);
                }
                std::cout<<"["<<node->x<<","<<node->y<<"] ";
                queue.pop();
                if(node->left!=nullptr)
                {
                     queue.push(node->left);
                }
                if(node->right!=nullptr)
                {
                     queue.push(node->right);
                }
           }
           std::cout<<"\n";
      }
}

void build_KD_Tree(std::vector<std::pair<double,double>>&coordinates,double x,double y)
{
      Node* root=build(root,coordinates,0,coordinates.size()-1,true);
      
      //std::cout<<"ans\n";
      //std::cout<<root->x<<" "<<root->y<<"\n";
      std::pair<double,double>ans={-1,-1};

      double max=std::numeric_limits<double>::max();

      query(root,x,y,ans,max,true);

      std::cout<<"nearest point is "<<ans.first<<" "<<ans.second<<"\n";
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
           build_KD_Tree(coord,qx,qy);   
           //std::cout<<"return to input\n"; 
      }
}

int main()
{
      takeInput();
      return 0;    
}