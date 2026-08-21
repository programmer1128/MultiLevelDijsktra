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


struct RouteCoordinate 
{
      double lon;
      double lat;
}; 

struct CorridorNodeDTO 
{
      double lat;
      double lon;
      double bearing;
};

struct CorridorEdgeDTO 
{
      uint32_t from;
      uint32_t to;
};

// The Master DTO
struct NavigationPayload 
{
      // Route Data
      std::vector<RouteCoordinate> route_geometry;
      uint32_t total_duration_seconds = 0;
      uint32_t total_distance_meters = 0;
      std::string duration_string;
      std::string distance_string;
    
      // Viterbi Corridor Data
      std::vector<uint32_t> route_path_ids;
      std::unordered_map<uint32_t, CorridorNodeDTO> corridor_nodes;
      std::vector<CorridorEdgeDTO> corridor_edges;
};



void multi_level_dijsktra(const GraphDataView& data,uint32_t source,uint32_t target,std::vector<uint32_t>&path);

std::vector<uint32_t> localized_shortcut_unpack(uint32_t start_node, uint32_t end_node, const GraphDataView& data);

std::vector<uint32_t> unpack_abstract_path(const std::vector<uint32_t>& abstract_path, const GraphDataView& data);

Json::Value getPathJson(uint32_t startNodeId,uint32_t destNodeId,const GraphDataView&data);

NavigationPayload extract_navigation_payload(const std::vector<uint32_t>& detailed_path, 
      const GraphDataView& graph_view);
