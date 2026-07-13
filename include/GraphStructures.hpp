#pragma once
#include <vector>
#include <cstdint>

struct Coordinate 
{
    double lat;
    double lon;
};

typedef struct ExpandedNode 
{
     uint32_t id;

     // OSM Tracking (Must be 64-bit to prevent overflow)
     uint64_t originalId;
    
     // CRITICAL: Required by the parser to connect the roads into a graph
     uint64_t start_osm_node_id;
     uint64_t end_osm_node_id;

     // Weight of this road (Dynamically updatable via your GNN)
     uint32_t wt;

     // Physical length of this road segment
     uint32_t distance_meters;

     // Array of points for MapLibre to draw curved roads
     std::vector<Coordinate> geometry;

     // Metadata
     uint8_t road_type;
     uint32_t name_id;

     bool is_one_way;
} ExpandedNode;

typedef struct ExpandedEdge 
{
     // Points to the ExpandedNode::id you are turning onto
     int destNode;

     // Time penalty for making this specific turn
     uint32_t turnWeight;

     // Geometric angle for navigation instructions (e.g., "Sharp Right")
     int16_t turnAngle;

     // Bitfield flag to heavily penalize or ban U-turns
     bool is_u_turn : 1;
} ExpandedEdge;

std::vector<ExpandedNode> expandedNodes;
std::vector<std::vector<ExpandedEdge>> expandedGraph;