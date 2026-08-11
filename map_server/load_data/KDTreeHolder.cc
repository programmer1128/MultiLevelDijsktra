#include "KDTreeHolder.hpp"

uint32_t calculate_distance(double lat1, double lon1, double lat2, double lon2) 
{
     const double R = 6371e3; // Earth radius in meters
     double phi1 = lat1 * M_PI / 180;
     double phi2 = lat2 * M_PI / 180;
     double dphi = (lat2 - lat1) * M_PI / 180;
     double dlambda = (lon2 - lon1) * M_PI / 180;

     double a = std::sin(dphi/2) * std::sin(dphi/2) +
              std::cos(phi1) * std::cos(phi2) *
              std::sin(dlambda/2) * std::sin(dlambda/2);
     double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
     return static_cast<uint32_t>(R * c);
}

void KDTreeHolder::initialize(const GraphDataView& view) 
{
     LOG_INFO<< "Constructing KD-Tree array in RAM\n";
     kd_nodes_array.reserve(view.total_nodes * 2); 

     for (uint32_t i = 0; i < view.total_nodes; ++i) 
     {
         const auto& meta = view.node_metadata[i];
         uint32_t offset = meta.geometry_offset;
         uint32_t size = meta.geometry_size;

         //insert the starting coordinate of the road
         KDNode first_node;
         first_node.node_id = meta.id;
         first_node.lat = view.coordinates[offset].lat;
         first_node.lon = view.coordinates[offset].lon;
         kd_nodes_array.push_back(first_node);

         double last_lat = first_node.lat;
         double last_lon = first_node.lon;

         //Iterate through the remaining coordinates of this specific road
         for (uint32_t j = 1; j < size; ++j) 
         {
             double current_lat = view.coordinates[offset + j].lat;
             double current_lon = view.coordinates[offset + j].lon;

             // Calculate Haversine distance from the last inserted point
             uint32_t dist = calculate_distance(last_lat, last_lon, current_lat, current_lon);

             //if the dist between the 2 nodes is greater than 30m then only we insert
             if (dist >= 30 || j == size - 1) 
             {
                 KDNode node;
                 node.node_id = meta.id; // Multiple KD nodes now point to the same EBG edge
                 node.lat = current_lat;
                 node.lon = current_lon;
                 kd_nodes_array.push_back(node);

                 // Move the anchor forward
                 last_lat = current_lat;
                 last_lon = current_lon;
             }
         }
        
         if (i > 0 && i % 1000000 == 0) 
         {
             LOG_INFO << "Processed " << i << " edges into " << kd_nodes_array.size() << " KD-Tree nodes\n";
         }
     }

     // Optional: Free unused reserved RAM capacity after the loop finishes
     kd_nodes_array.shrink_to_fit();

     if (!kd_nodes_array.empty()) 
     {
         LOG_INFO<< "Starting heavy spatial sort for " << kd_nodes_array.size() << " nodes\n";
         build_KD_Tree(kd_nodes_array, 0, kd_nodes_array.size() - 1, true);
     } 
     LOG_INFO<< "KD-Tree array ready\n";
}