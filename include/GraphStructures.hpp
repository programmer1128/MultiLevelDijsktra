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


//this is used for the CSR format of graph storage to convert the graph before we feed it
//to dinic algorithm for the partition
typedef struct FlowEdge
{
      uint32_t to;
      int32_t cap;
      int32_t flow;
      uint32_t rev_idx; // Index of the residual edge in the 'edges' array     
}FlowEdge;

std::vector<uint32_t> head;
std::vector<FlowEdge> edges;


typedef struct ProjectedNode
{
     uint32_t id;
     double projectedValue;
}ProjectedNode;


//this struct is used in the Dinic partitioning of the graph for trying out all the 12 combinations
//for 10,20,30% for every 4 projection axes and then storing the best set of partitions
struct CutResult 
{
      double score;
      int cut_size;
      std::vector<uint32_t> partition_A;
      std::vector<uint32_t> partition_B;
};

//used in the Dinic partitioner for the graph when the split is done
struct LocalGraph 
{
      uint32_t num_nodes;
      std::vector<uint32_t> head;
      std::vector<FlowEdge> edges;
      uint32_t local_source;
      uint32_t local_sink;
};


//used in making the overlay graph which is the shortcut graph consisting of shortcuts between the
//boundary nodes in a cell
struct RoutingEdge 
{
      uint32_t to;
      uint32_t weight; 
};
// This is the physical distance or travel time (e.g., in milliseconds or meters)