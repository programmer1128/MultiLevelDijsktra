#pragma once
#include<cstdint>
#include<vector>
#include "load_data/GraphDataView.hpp"
#include<drogon/drogon.h>

struct EBG_Metadata 
{
     bool is_reverse_geometry;
     uint32_t geometry_offset;
     uint32_t geometry_size;
};


struct RoutingData 
{
     uint32_t total_nodes = 0;
    
     // Partitioning / Cell Mapping
     std::vector<uint16_t> node_cell_mapping;

     // Forward Base Graph (For Forward Search)
     std::vector<uint32_t> routing_head;
     std::vector<RoutingEdge> routing_edges;
     
     // Reverse Base Graph (For Backward Search)
     std::vector<uint32_t> reverse_routing_head;
     std::vector<RoutingEdge> reverse_routing_edges;
    
     // Forward Overlay Graph (For Forward Search)
     std::vector<uint32_t> overlay_head;
     std::vector<ShortcutEdge> overlay_edges;

     // Reverse Overlay Graph (For Backward Search)
     std::vector<uint32_t> reverse_overlay_head;
     std::vector<ShortcutEdge> reverse_overlay_edges;

     std::vector<Coordinate> coordinates;

     std::vector<EBG_Metadata> node_metadata;
};


void multi_level_dijsktra(const GraphDataView& data,uint32_t source,uint32_t target,std::vector<uint32_t>&path);

std::vector<uint32_t> localized_shortcut_unpack(uint32_t start_node, uint32_t end_node, const GraphDataView& data);

std::vector<uint32_t> unpack_abstract_path(const std::vector<uint32_t>& abstract_path, const GraphDataView& data);

Json::Value getPathJson(uint32_t startNodeId,uint32_t destNodeId,const GraphDataView&data);
