#pragma once
#include <vector>
#include <cstdint>

enum ProjectionAxis 
{
     AXIS_0_DEG = 0,
     AXIS_90_DEG = 1,
     AXIS_45_DEG = 2,
     AXIS_135_DEG = 3
};

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

     // --- NEW: UI Data-Oriented Decoupling ---
     // Replaces std::vector<Coordinate> geometry.
     // Points directly to the memory-mappable "geometry.bin" flat array.
     uint32_t geometry_offset;
     uint32_t geometry_size;

     // --- NEW: Partition Assignment ---
     // Inherited from the NBG intersection node partition cut.
     uint16_t cell_id;

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




/**
 * @brief Represents a physical intersection or junction in the real world.
 * 
 * In a production engine, this graph is tiny compared to the Edge-Expanded version.
 * This structure tracks the true physical location, which is critical for the 
 * Inertial Flow / Dinic projection axes.
 */
struct NodeBasedNode 
{
       // Sequential internal 32-bit ID (0, 1, 2...) for fast array indexing
       uint32_t id;

       // The original 64-bit OSM node ID, essential for mapping turn restrictions
       uint64_t osm_node_id;

       // Physical location needed for distance calculations and partitioning projections
       Coordinate coordinate;
      
       // Tracks the assigned cell after Dinic finishes partitioning the NBG.
       // 0 means unassigned. Max 65,535 cells (perfect for multi-level mapping).
       uint16_t cell_id; 
};

/**
 * @brief Represents a physical road segment connecting two intersections.
 * 
 * Unlike the EBG (where edges are turns), edges here are actual streets.
 * This struct is optimized to retain all UI metadata through compaction
 * without holding heavy vector objects.
 */
struct NodeBasedEdge 
{
       // Points to the internal index of the NodeBasedNode this road leads to
       uint32_t target_node_id;

       // --- Metrics ---
       uint32_t weight;          // Travel time weight (dynamic/GNN updatable)
       uint32_t distance_meters; // Physical structural length

       // --- Metadata Reference ---
       uint64_t osm_way_id;      // The original OSM street ID (needed for turn restrictions)
       uint32_t name_id;         // Offset pointer to string dictionary
       uint8_t road_type;         // Functional classification (Motorway, Primary, etc.)

       // --- UI Geometry References ---
       // Instead of std::vector<Coordinate>, we track offsets in a global array.
       // This allows Degree-2 compaction to merge paths without shifting heap memory.
       uint32_t geometry_offset; 
       uint32_t geometry_size;   

       // --- Topography Flags ---
       bool is_oneway;           // Tracks flow direction constraints
};

// --- In-Memory Construction Containers ---
// During Phase 1 & 2, we use an adjacency list because we need to delete 
// nodes and append edges dynamically during compaction. 
// We will flatten this to CSR *only* when passing it to Dinic.
std::vector<NodeBasedNode> nbgNodes;
std::vector<std::vector<NodeBasedEdge>> nbgGraph;

std::vector<Coordinate> global_geometry;
// We will need a map to quickly find the NBG Node index using the OSM Node ID
std::unordered_map<uint64_t, uint32_t> osm_to_nbg_index;


//Structure to group a cell's boundary nodes for localized Dijkstra sweeps
struct CellBoundaries 
{
     std::vector<uint32_t> entry_nodes;
     std::vector<uint32_t> exit_nodes;
};
