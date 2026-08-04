#include "KDTreeHolder.hpp"
// Make sure you include the file that actually has build_KD_Tree

void KDTreeHolder::initialize(const GraphDataView& view) 
{
     LOG_INFO<< "Constructing KD-Tree array in RAM\n";
     kd_nodes_array.resize(view.total_nodes);

     for (uint32_t i = 0; i < view.total_nodes; ++i) 
     {
         const auto& meta = view.node_metadata[i];
         uint32_t offset = meta.geometry_offset;
 
         kd_nodes_array[i].node_id = meta.id;
         kd_nodes_array[i].lat = view.coordinates[offset].lat;
         kd_nodes_array[i].lon = view.coordinates[offset].lon;
        
         if (i > 0 && i % 1000000 == 0) 
         {
             LOG_INFO<< "Loaded " << i << " nodes into RAM\n";
         }
     }

     if (!kd_nodes_array.empty()) 
     {
         LOG_INFO<< "Starting heavy spatial sort for " << kd_nodes_array.size() << " nodes\n";
         build_KD_Tree(kd_nodes_array, 0, kd_nodes_array.size() - 1, true);
     } 
     LOG_INFO<< "KD-Tree array ready\n";
}