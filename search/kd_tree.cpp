#include<bits/stdc++.h>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

typedef struct KDNode 
{
     double lat;
     double lon;
     uint32_t node_id;
}KDNode;


void build_KD_Tree(std::vector<KDNode>& kd_nodes, int l, int h, bool along_X_Axis)
{
     if(l>=h)
     {
         return;
     }

     int mid= l+(h-l)/2;

     auto cmpX=[&](const KDNode&node1,const KDNode& node2)-> bool
     {
         return node1.lat<node2.lat;
     };

     auto cmpY=[&](const KDNode& node1,const KDNode& node2)->bool
     {
         return node1.lon<node2.lon;
     };

     if(along_X_Axis)
     {
         //we sort acc to the lat
         std::nth_element(kd_nodes.begin()+l,kd_nodes.begin()+mid,kd_nodes.begin()+h+1,cmpX);
     }
     else
     {
         //sort acc to the longitude
         std::nth_element(kd_nodes.begin()+l,kd_nodes.begin()+mid,kd_nodes.begin()+h+1,cmpY);
     }

     build_KD_Tree(kd_nodes,l,mid-1,!along_X_Axis);
     build_KD_Tree(kd_nodes,mid+1,h,!along_X_Axis);
}

void query_kd_tree(const std::vector<KDNode>&kd_nodes,double lat,double lon,int l,int h,
     bool alongXAxis, double& minDist,KDNode& closestNode)
{
     if(l>h)
     {
         return ;
     }    
     int mid=l+(h-l)/2;
     KDNode currNode=kd_nodes[mid];

     double dist=(std::pow(lat-currNode.lat,2)+std::pow(lon-currNode.lon,2));

     if(dist<minDist)
     {
         minDist=dist;
         closestNode=currNode;
     }
     double orthogonal_dist= alongXAxis?lat-currNode.lat:lon-currNode.lon;

     int first_half_l = l, first_half_h = mid - 1;
     int second_half_l = mid + 1, second_half_h = h;

     if (orthogonal_dist > 0) 
     {
         //target position is on righ side
         std::swap(first_half_l, second_half_l);
         std::swap(first_half_h, second_half_h);
     }

     query_kd_tree(kd_nodes,lat,lon,first_half_l,first_half_h,!alongXAxis,minDist,closestNode);
     
     //search the other side if the orthogonal distance squared is less than our best found distance
     if ((orthogonal_dist*orthogonal_dist)<minDist) 
     {
         query_kd_tree(kd_nodes,lat,lon,second_half_l,second_half_h,!alongXAxis,minDist,closestNode);
     }
     
}


void loadGeometryForKDTree(const std::string& filepath, std::vector<KDNode>& kd_nodes) 
{
     std::cout << "Loading geometry.bin for KD-Tree...\n";
     std::ifstream geom_in(filepath, std::ios::binary);
     if(!geom_in) 
     {
         std::cerr << "Failed to open " << filepath << "\n";
         exit(1);
     }
    
     //node count
     uint32_t total_coords = 0;
     geom_in.read(reinterpret_cast<char*>(&total_coords), sizeof(total_coords));
    
     //pre alloc ram for the arrays
     std::vector<Coordinate> raw_coords(total_coords);
     kd_nodes.resize(total_coords);
    
     //read coord struct
     geom_in.read(reinterpret_cast<char*>(raw_coords.data()), total_coords * sizeof(Coordinate));
     geom_in.close();

     //the physical array index as the routing node_id
     for (uint32_t i = 0; i < total_coords; ++i) 
     {
         kd_nodes[i] = {raw_coords[i].lat, raw_coords[i].lon, i};
     }
    
     //free the temp raw buffer
     raw_coords.clear();
     raw_coords.shrink_to_fit();

     std::cout << "Successfully loaded " << total_coords << " points into KD-Tree array.\n";
}



int main()
{
     std::vector<KDNode> kd_nodes;
     std::string file="/home/aritra/MultiLevelDijsktra/parsing/geometry.bin";
     loadGeometryForKDTree(file,kd_nodes);
     build_KD_Tree(kd_nodes,0,kd_nodes.size()-1,true);
     KDNode closestNode;
     double min=std::numeric_limits<double>::max();
     query_kd_tree(kd_nodes,28.6129,77.2295,0,kd_nodes.size()-1,true,min,closestNode);

     std::cout<<"Closest node is "<<closestNode.lat<<" "<<closestNode.lon<<" node id = "<<closestNode.node_id<<"\n";
     return 0;
}