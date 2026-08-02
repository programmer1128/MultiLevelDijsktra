#pragma once
#include <cstdint>
#include "/home/aritra/MultiLevelDijsktra/map_server/include/GraphStructures.hpp"


#pragma pack(push, 1)
struct PackedMetadata 
{
     uint32_t id;
     uint64_t originalId;
     uint64_t start_osm_node_id;
     uint64_t end_osm_node_id;
     uint32_t wt;
     uint32_t distance_meters;
     uint16_t cell_id;
     uint8_t  road_type;
     uint32_t name_id;
     bool     is_one_way;
     bool     is_reverse_geometry;
     uint32_t geometry_offset;
     uint32_t geometry_size;
};

#pragma pack(pop)

//The Zero-Copy Data View
struct GraphDataView 
{
     uint32_t total_nodes = 0;

     // Base Graph (Forward & Reverse)
     const uint32_t* routing_head = nullptr;
     const RoutingEdge* routing_edges = nullptr;
    
     const uint32_t* reverse_routing_head = nullptr;
     const RoutingEdge* reverse_routing_edges = nullptr;

     // Overlay Graph (Forward & Reverse)
     const uint32_t* overlay_head = nullptr;
     const ShortcutEdge* overlay_edges = nullptr;

     const uint32_t* reverse_overlay_head = nullptr;
     const ShortcutEdge* reverse_overlay_edges = nullptr;

     // Global UI Data
     const uint16_t* node_cell_mapping = nullptr;
     const Coordinate* coordinates = nullptr;
     const PackedMetadata* node_metadata = nullptr;
};

