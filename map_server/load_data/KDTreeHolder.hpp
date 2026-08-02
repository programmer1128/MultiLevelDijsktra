#pragma once
#include <vector>
#include <iostream>
#include "GraphDataView.hpp"
#include "/home/aritra/MultiLevelDijsktra/map_server/search/kd_tree.h" // Adjust path to point to your kd_tree.h

class KDTreeHolder
{
private:
    // This is the ONLY place the KD-Tree array lives in RAM
    std::vector<KDNode> kd_nodes_array;

public:
     // Called EXACTLY ONCE by Drogon on server boot
     void buildFromGraphView(const GraphDataView& view) 
     {
         std::cout << "Constructing KD-Tree array in RAM..." << std::endl;
         kd_nodes_array.resize(view.total_nodes);

         for (uint32_t i = 0; i < view.total_nodes; ++i) 
         {
             const auto& meta = view.node_metadata[i];
             uint32_t offset = meta.geometry_offset;
 
             kd_nodes_array[i].node_id = meta.id;
             kd_nodes_array[i].lat = view.coordinates[offset].lat;
             kd_nodes_array[i].lon = view.coordinates[offset].lon;
         }

         std::cout << "Building KD-Tree spatial index...\n";
         if (!kd_nodes_array.empty()) 
         { 
             // Pass the array to your external kd_tree.cpp logic
             build_KD_Tree(kd_nodes_array, 0, kd_nodes_array.size() - 1, true);
         }
         std::cout << "KD-Tree successfully constructed and isolated in RAM!\n";
     }

     const std::vector<KDNode>& getArray() const 
     {
         return kd_nodes_array;
     }
};