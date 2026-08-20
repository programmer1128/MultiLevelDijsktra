#include<bits/stdc++.h>
#include "kd_tree.h"


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


//new query function. the graph used for this engine is a edge expanded graph. that means we have 
//2 nodes for a bi directional road. first is for a way north to south second for south to nort.
//now here the problem is if the kd tree snaps south node isntead of north user might have to take
//a u turn. so we need a better radius search approach
void query_kd_tree_radius(const std::vector<KDNode>& kd_nodes, double lat, double lon, int l, int h,
     bool alongXAxis, double radius_squared, std::vector<KDNode>& candidates)
{
     if(l > h) return;
     
     int mid = l + (h - l) / 2;
     KDNode currNode = kd_nodes[mid];

     double dist = (std::pow(lat - currNode.lat, 2) + std::pow(lon - currNode.lon, 2));

     // If the node falls within our fixed circle, add it to the candidate pool
     if(dist <= radius_squared)
     {
         candidates.push_back(currNode);
     }
     
     double orthogonal_dist = alongXAxis ? lat - currNode.lat : lon - currNode.lon;

     int first_half_l = l, first_half_h = mid - 1;
     int second_half_l = mid + 1, second_half_h = h;

     if (orthogonal_dist > 0) 
     {
         std::swap(first_half_l, second_half_l);
         std::swap(first_half_h, second_half_h);
     }

     //always search the primary side
     query_kd_tree_radius(kd_nodes, lat, lon, first_half_l, first_half_h, !alongXAxis, radius_squared, candidates);
     
     // Search the other side only if our fixed search radius crosses the splitting plane
     if ((orthogonal_dist * orthogonal_dist) <= radius_squared) 
     {
         query_kd_tree_radius(kd_nodes, lat, lon, second_half_l, second_half_h, !alongXAxis, radius_squared, candidates);
     }
}



uint32_t findClosestNode(const std::vector<KDNode>&kd_nodes,double lat,double lon)
{
     KDNode closestNode;
     double min = std::numeric_limits<double>::max();
     query_kd_tree(kd_nodes,lat, lon, 0, kd_nodes.size()-1, true, min, closestNode);

     //std::cout<<"id = "<<closestNode.node_id<<"\n";
     return closestNode.node_id;
}



uint32_t findBestNodeWithHeading(const std::vector<KDNode>& kd_nodes, double lat, double lon, double user_heading)
{
     std::vector<KDNode> candidates;
     
     // Approx conversion: 1 degree latitude is ~111km. 
     // 20 meters is roughly 0.00018 degrees. 
     // We need radius squared for the distance math: (0.00018)^2 = 3.24e-8
     double max_radius_deg = 0.00018;
     double radius_squared = max_radius_deg * max_radius_deg; 
     
     query_kd_tree_radius(kd_nodes, lat, lon, 0, kd_nodes.size() - 1, true, radius_squared, candidates);

     // Safety net: If no roads are within 20 meters, revert to your original absolute nearest neighbor
     if (candidates.empty()) 
     {
         return findClosestNode(kd_nodes, lat, lon);
     }

     uint32_t best_node_id = candidates[0].node_id;
     double min_cost = std::numeric_limits<double>::max();

     for (const auto& node : candidates) 
     {
         //Pull the start and end coordinates of this specific node_id from your EBG geometry
         // to calculate the physical bearing of the road segment.
         double road_bearing = 0.0; // Placeholder until geometry lookup is connected
         
         //calculate Angular Difference (0 to 180 degrees)
         double angular_diff = 180.0 - std::abs(std::abs(user_heading - road_bearing) - 180.0);
         
         //calculate physical distance
         double dist = std::sqrt(std::pow(lat - node.lat, 2) + std::pow(lon - node.lon, 2));

         // Normalized Cost Function (Alpha = 0.5, Beta = 0.5)
         // We divide distance by max_radius and angular_diff by 180 to scale both between 0.0 and 1.0
         double cost = (0.5 * (dist / max_radius_deg)) + (0.5 * (angular_diff / 180.0));

         if (cost < min_cost) 
         {
             min_cost = cost;
             best_node_id = node.node_id;
         }
     }
     
     return best_node_id;
}

/*
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

void loadEBGForKDTree(const std::string& geom_file, const std::string& meta_file, std::vector<KDNode>& kd_nodes) 
{
    //Load the flat 139M NBG coordinate array
    std::ifstream geom_in(geom_file, std::ios::binary);
    if (!geom_in) {
        std::cerr << "Failed to open " << geom_file << "\n";
        return;
    }
    uint32_t total_coords = 0;
    geom_in.read(reinterpret_cast<char*>(&total_coords), sizeof(total_coords));
    std::vector<Coordinate> global_geom(total_coords);
    geom_in.read(reinterpret_cast<char*>(global_geom.data()), total_coords * sizeof(Coordinate));
    geom_in.close();

    //load the EBG Metadata and construct the mapped KD-Tree array
    std::ifstream meta_in(meta_file, std::ios::binary);
    if (!meta_in) {
        std::cerr << "Failed to open " << meta_file << "\n";
        return;
    }
    
    size_t total_ebg_nodes = 0;
    meta_in.read(reinterpret_cast<char*>(&total_ebg_nodes), sizeof(total_ebg_nodes));
    kd_nodes.reserve(total_ebg_nodes);

    for (size_t i = 0; i < total_ebg_nodes; ++i) 
    {
        uint32_t id;
        uint64_t originalId, start_osm_node_id, end_osm_node_id;
        uint32_t wt, distance_meters, name_id, geometry_offset, geometry_size;
        uint16_t cell_id;
        uint8_t road_type;
        bool is_one_way;
        meta_in.read(reinterpret_cast<char*>(&id), sizeof(id));
        meta_in.read(reinterpret_cast<char*>(&originalId), sizeof(originalId));
        meta_in.read(reinterpret_cast<char*>(&start_osm_node_id), sizeof(start_osm_node_id));
        meta_in.read(reinterpret_cast<char*>(&end_osm_node_id), sizeof(end_osm_node_id));
        meta_in.read(reinterpret_cast<char*>(&wt), sizeof(wt));
        meta_in.read(reinterpret_cast<char*>(&distance_meters), sizeof(distance_meters));
        meta_in.read(reinterpret_cast<char*>(&cell_id), sizeof(cell_id));
        meta_in.read(reinterpret_cast<char*>(&road_type), sizeof(road_type));
        meta_in.read(reinterpret_cast<char*>(&name_id), sizeof(name_id));
        meta_in.read(reinterpret_cast<char*>(&is_one_way), sizeof(is_one_way));
        meta_in.read(reinterpret_cast<char*>(&geometry_offset), sizeof(geometry_offset));
        meta_in.read(reinterpret_cast<char*>(&geometry_size), sizeof(geometry_size));

        // Create the KDNode using the EBG ID and the mapped starting coordinate
        KDNode node;
        node.node_id = id;
        node.lat = global_geom[geometry_offset].lat;
        node.lon = global_geom[geometry_offset].lon;
        kd_nodes.push_back(node);
    }
    meta_in.close();
    std::cout << "Successfully fused and loaded " << kd_nodes.size() << " EBG nodes into the KD-Tree!\n";
}



int main()
{
     std::vector<KDNode> kd_nodes;
     std::string geom_file = "/home/aritra/MultiLevelDijsktra/parsing/geometry.bin";
     std::string meta_file = "/home/aritra/MultiLevelDijsktra/parsing/metadata.bin";
     
     loadEBGForKDTree(geom_file, meta_file, kd_nodes);
     
     build_KD_Tree(kd_nodes, 0, kd_nodes.size()-1, true);
     
     KDNode closestNode;
     
     double min = std::numeric_limits<double>::max();
     query_kd_tree(kd_nodes, 22.480839, 88.305162, 0, kd_nodes.size()-1, true, min, closestNode);
     std::cout << "START: Closest node is " << closestNode.lat << " " << closestNode.lon << " node id = " << closestNode.node_id << "\n";
     
     min = std::numeric_limits<double>::max();
     query_kd_tree(kd_nodes, 22.5646, 88.3512, 0, kd_nodes.size()-1, true, min, closestNode);
     std::cout << "TARGET: Closest node is " << closestNode.lat << " " << closestNode.lon << " node id = " << closestNode.node_id << "\n";
     
     return 0;
}
*/
