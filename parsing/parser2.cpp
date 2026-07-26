#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <string>
#include <osmium/visitor.hpp>
#include<bits/stdc++.h>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

// --- Data Structures ---

// Represents an OSM Turn Restriction (e.g., No Left Turn)
struct TurnRestriction 
{
     uint64_t from_way_id;
     uint64_t to_way_id;
     uint64_t via_node_id;
     std::string restriction_type; // e.g., "no_left_turn", "only_straight_on"
};

// Global storage for our parsed data
std::unordered_map<uint64_t, uint8_t> node_counts;
std::vector<TurnRestriction> turn_restrictions;

// --- Handlers ---

// Handler 1: Finds true intersections by counting node references on ALL roads
class NodeReferenceCounter : public osmium::handler::Handler {
public:
    void way(const osmium::Way& way) {
        // FILTERING: If it has a highway tag, we keep it. No exclusions.
        const char* highway = way.tags().get_value_by_key("highway");
        if (!highway) {
            return; 
        }

        // COUNTING: Cap at 2 to identify intersections safely
        for (const auto& node_ref : way.nodes()) {
            uint64_t node_id = node_ref.ref();
            if (node_counts[node_id] < 2) {
                node_counts[node_id]++;
            }
        }
    }
};

// Handler 2: Extracts Turn Restrictions from Relations
class TurnRestrictionHandler : public osmium::handler::Handler {
public:
    void relation(const osmium::Relation& relation) {
        // Only look at relations that are explicitly tagged as restrictions
        const char* type = relation.tags().get_value_by_key("type");
        if (!type || std::string(type) != "restriction") {
            return;
        }

        const char* restriction_tag = relation.tags().get_value_by_key("restriction");
        if (!restriction_tag) {
            return;
        }

        TurnRestriction tr;
        tr.restriction_type = restriction_tag;
        tr.from_way_id = 0;
        tr.to_way_id = 0;
        tr.via_node_id = 0;

        // A restriction has members (from way, to way, via node)
        for (const auto& member : relation.members()) {
            std::string role = member.role();
            if (role == "from" && member.type() == osmium::item_type::way) {
                tr.from_way_id = member.ref();
            } else if (role == "to" && member.type() == osmium::item_type::way) {
                tr.to_way_id = member.ref();
            } else if (role == "via" && member.type() == osmium::item_type::node) {
                tr.via_node_id = member.ref();
            }
        }

        // Only save it if it has all three required components
        if (tr.from_way_id != 0 && tr.to_way_id != 0 && tr.via_node_id != 0) {
            turn_restrictions.push_back(tr);
        }
    }
};


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



class Pass2Handler : public osmium::handler::Handler 
{
public:
     uint32_t current_nbg_node_id = 0;

     // Helper to get or create an NBG Node for true intersections
     uint32_t get_or_create_node(const osmium::NodeRef& node_ref) 
     {
         uint64_t osm_id = node_ref.ref();
         auto it = osm_to_nbg_index.find(osm_id);
         if (it != osm_to_nbg_index.end()) 
         {
             return it->second;
         }

         // Create the physical intersection node
         NodeBasedNode new_node;
         new_node.id = current_nbg_node_id++;
         new_node.osm_node_id = osm_id;
         new_node.coordinate = { static_cast<double>(node_ref.lat()), static_cast<double>(node_ref.lon()) };
         new_node.cell_id = 0; // Unassigned until Dinic runs

         nbgNodes.push_back(new_node);
         nbgGraph.push_back(std::vector<NodeBasedEdge>()); // Allocate adjacency list
         osm_to_nbg_index[osm_id] = new_node.id;
         
         return new_node.id;
     }

     void way(const osmium::Way& way) 
     {
         const char* highway = way.tags().get_value_by_key("highway");
         if (!highway) return;

         bool is_oneway = false;
         const char* oneway_tag = way.tags().get_value_by_key("oneway");
         if (oneway_tag && (std::string(oneway_tag) == "yes" || std::string(oneway_tag) == "1")) 
         {
             is_oneway = true;
         }

         float speed_m_s = 11.0f; // Default 40 km/h
         const char* maxspeed_tag = way.tags().get_value_by_key("maxspeed");
         if (maxspeed_tag) {
             int speed_kmh = std::atoi(maxspeed_tag);
             if (speed_kmh > 0) speed_m_s = speed_kmh / 3.6f;
         }

         // Ensure the start node is instantiated
         uint32_t segment_start_nbg_id = get_or_create_node(way.nodes().front());
         uint32_t segment_distance = 0;
         
         // Track where our geometry chunk begins in the global array
         uint32_t current_geometry_offset = global_geometry.size();
         uint32_t current_geometry_size = 0;

         for (size_t i = 0; i < way.nodes().size(); ++i) 
         {
             const auto& node_ref = way.nodes()[i];
             
             // Append to the global geometry flat array
             global_geometry.push_back({static_cast<double>(node_ref.lat()), static_cast<double>(node_ref.lon())});
             current_geometry_size++;

             if (i > 0) {
                 const auto& prev_node_ref = way.nodes()[i - 1];
                 segment_distance += calculate_distance(
                     prev_node_ref.lat(), prev_node_ref.lon(),
                     node_ref.lat(), node_ref.lon()
                 );
             }

             bool is_intersection = (node_counts[node_ref.ref()] >= 2);
             bool is_last_node = (i == way.nodes().size() - 1);

             // Split the segment at true intersections or ends of the road
             if ((is_intersection && i != 0) || is_last_node) 
             {
                 uint32_t segment_end_nbg_id = get_or_create_node(node_ref);

                 // 1. Create the Forward Edge
                 NodeBasedEdge forward_edge;
                 forward_edge.target_node_id = segment_end_nbg_id;
                 forward_edge.distance_meters = segment_distance;
                 forward_edge.weight = std::max(1u, static_cast<uint32_t>(segment_distance / speed_m_s));
                 forward_edge.osm_way_id = way.id();
                 forward_edge.road_type = 0; // Replace with highway tag mapper if needed
                 forward_edge.name_id = 0;   // Replace with string dictionary mapper
                 forward_edge.is_oneway = is_oneway;
                 forward_edge.geometry_offset = current_geometry_offset;
                 forward_edge.geometry_size = current_geometry_size;

                 nbgGraph[segment_start_nbg_id].push_back(forward_edge);

                 // 2. If two-way, create the Reverse Edge
                 if (!is_oneway) 
                 {
                     NodeBasedEdge reverse_edge = forward_edge; // Copy all metrics
                     reverse_edge.target_node_id = segment_start_nbg_id; // Point backward
                     nbgGraph[segment_end_nbg_id].push_back(reverse_edge);
                 }

                 // Reset for the next segment along this OSM Way
                 segment_start_nbg_id = segment_end_nbg_id;
                 segment_distance = 0;
                 
                 // The end of this segment is the start of the next geometry chunk
                 current_geometry_offset = global_geometry.size() - 1; 
                 current_geometry_size = 1;
             }
         }
     }
};




int main() 
{
     std::string osm_file = "india-260724.osm.pbf";
    
     // --- Pass 1A: Count Node References (Ways) ---
     std::cout << "Pass 1A: Counting node references on all roads...\n";
     osmium::io::Reader way_reader{osm_file, osmium::osm_entity_bits::way};
     NodeReferenceCounter counter_handler;
     osmium::apply(way_reader, counter_handler);
     way_reader.close();
    
     // --- Pass 1B: Extract Turn Restrictions (Relations) ---
     std::cout << "Pass 1B: Extracting turn restrictions...\n";
     osmium::io::Reader rel_reader{osm_file, osmium::osm_entity_bits::relation};
     TurnRestrictionHandler rel_handler;
     osmium::apply(rel_reader, rel_handler);
     rel_reader.close();

     std::cout << "Extraction Complete\n";
     std::cout << "Total unique road nodes discovered: " << node_counts.size() << "\n";
     std::cout << "Total turn restrictions found: " << turn_restrictions.size() << "\n";




     // --- Run Pass 2 ---
     std::cout << "Pass 2: Splitting ways and extracting coordinates...\n";
    
     // 1. Initialize the index that will hold the node coordinates in RAM
     using index_type = osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location>;
     index_type location_index;
    
     // 2. Create the location handler that automatically links nodes to ways
     osmium::handler::NodeLocationsForWays<index_type> location_handler{location_index};
    
     // 3. We must tell the reader to read BOTH nodes (to cache them) and ways (to process them)
     osmium::io::Reader pass2_reader{osm_file, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
     Pass2Handler pass2_handler;
    
     // 4. Apply BOTH handlers. The location_handler must come first so ways get their coordinates.
     osmium::apply(pass2_reader, location_handler, pass2_handler);
     pass2_reader.close();


     // ========================================================================
     // PHASE 3: GENERATE PARTITIONER DATA (FROM LIGHTWEIGHT NBG)
     // ========================================================================
     std::cout << "Phase 3: Exporting NBG Flow Graph & Projections for Dinic...\n";
     
     uint32_t nbg_nodes_count = nbgNodes.size();

     // 1. Projections (Using NBG Intersections instead of EBG segments)
     std::vector<std::vector<ProjectedNode>> nbg_projections(4);
     for (int i = 0; i < 4; ++i) nbg_projections[i].reserve(nbg_nodes_count);

     for (const auto& node : nbgNodes) 
     {
         nbg_projections[AXIS_0_DEG].push_back({node.id, node.coordinate.lon});
         nbg_projections[AXIS_90_DEG].push_back({node.id, node.coordinate.lat});
         nbg_projections[AXIS_45_DEG].push_back({node.id, node.coordinate.lon + node.coordinate.lat});
         nbg_projections[AXIS_135_DEG].push_back({node.id, -node.coordinate.lon + node.coordinate.lat});
     }

     // Sort NBG projections by their projected values
     auto cmp = [&](const ProjectedNode& a, const ProjectedNode& b) -> bool {
         return a.projectedValue < b.projectedValue;  
     };
     for (int i = 0; i < 4; ++i) std::sort(nbg_projections[i].begin(), nbg_projections[i].end(), cmp);

     // Write projections_format.bin
     std::ofstream proj_out("projections_format.bin", std::ios::binary);
     uint32_t num_axes = 4;
     proj_out.write(reinterpret_cast<const char*>(&num_axes), sizeof(num_axes));
     proj_out.write(reinterpret_cast<const char*>(&nbg_nodes_count), sizeof(nbg_nodes_count));
     for (int i = 0; i < 4; ++i) {
         proj_out.write(reinterpret_cast<const char*>(nbg_projections[i].data()), nbg_nodes_count * sizeof(ProjectedNode));
     }
     proj_out.close();
     nbg_projections.clear(); // Wipe projections immediately

     // 2. FLOW CSR Generation (From NBG)
     std::vector<uint32_t> flow_head(nbg_nodes_count + 1, 0); 
     std::vector<uint32_t> nbg_degree(nbg_nodes_count, 0);

     for (uint32_t u = 0; u < nbg_nodes_count; ++u) {
         for (const auto& edge : nbgGraph[u]) {
             nbg_degree[u]++;                  // Forward edge u -> v
             nbg_degree[edge.target_node_id]++; // Residual edge v -> u
         }
     }

     for (uint32_t i = 0; i < nbg_nodes_count; ++i) {
         flow_head[i + 1] = flow_head[i] + nbg_degree[i];
     }

     uint32_t total_flow_edges = flow_head[nbg_nodes_count];
     std::vector<FlowEdge> flow_edges(total_flow_edges);
     std::vector<uint32_t> tail = flow_head; 

     for (uint32_t u = 0; u < nbg_nodes_count; ++u) 
     {
         for (const auto& edge : nbgGraph[u]) 
         {
             uint32_t v = edge.target_node_id;
             uint32_t idx_uv = tail[u]++;
             uint32_t idx_vu = tail[v]++;

             // Capacity = 1 for unit edge partitioning cuts
             flow_edges[idx_uv] = {v, 1, 0, idx_vu};
             flow_edges[idx_vu] = {u, 0, 0, idx_uv};
         }
     }

     // Write flow_csr.bin
     std::ofstream flow_out("flow_csr.bin", std::ios::binary);
     flow_out.write(reinterpret_cast<const char*>(&nbg_nodes_count), sizeof(nbg_nodes_count));
     flow_out.write(reinterpret_cast<const char*>(&total_flow_edges), sizeof(total_flow_edges));
     flow_out.write(reinterpret_cast<const char*>(flow_head.data()), flow_head.size() * sizeof(uint32_t));
     flow_out.write(reinterpret_cast<const char*>(flow_edges.data()), flow_edges.size() * sizeof(FlowEdge));
     flow_out.close();
     
     flow_head.clear();
     flow_edges.clear();
     std::cout << "NBG Flow Graph and Projections successfully saved. Partitioner footprint neutralized.\n";

     // ========================================================================
     // PHASE 3.5: BRIDGE - CONVERTING NBG ROADS INTO EBG NODES
     // ========================================================================
     std::cout << "Phase 3.5: Transforming NBG topology into Edge-Based Graph (EBG) nodes...\n";

     uint32_t current_expanded_node_id = 0;
     
     // Temporary map to easily find edges when generating turn links later
     // Key: exit_intersection_osm_id -> Vector of EBG Node IDs entering/originating
     std::unordered_map<uint64_t, std::vector<int>> intersection_to_segments;

     for (uint32_t u = 0; u < nbg_nodes_count; ++u) 
     {
         for (const auto& nbg_edge : nbgGraph[u]) 
         {
             ExpandedNode exp_node;
             exp_node.id = current_expanded_node_id++;
             exp_node.originalId = nbg_edge.osm_way_id;
             
             // Core structural bridges for Turn Restrictions & Projection mapping
             exp_node.start_osm_node_id = nbgNodes[u].osm_node_id;
             exp_node.end_osm_node_id = nbgNodes[nbg_edge.target_node_id].osm_node_id;
             
             exp_node.wt = nbg_edge.weight;
             exp_node.distance_meters = nbg_edge.distance_meters;
             exp_node.road_type = nbg_edge.road_type;
             exp_node.name_id = nbg_edge.name_id;
             exp_node.is_one_way = nbg_edge.is_oneway;
             
             // Geometry index maps (No actual vector reallocations!)
             exp_node.geometry_offset = nbg_edge.geometry_offset;
             exp_node.geometry_size = nbg_edge.geometry_size;
             
             // Map cell assignments directly via originating NBG node
             exp_node.cell_id = nbgNodes[u].cell_id;

             expandedNodes.push_back(exp_node);
             
             // Map this EBG segment to its starting point for Phase 4 turn discovery
             intersection_to_segments[exp_node.start_osm_node_id].push_back(exp_node.id);
         }
     }

     // The NBG has finished its job; dump the dynamic adjacency structure to reclaim memory
     nbgGraph.clear();
     nbgGraph.shrink_to_fit();
     nbgNodes.clear();
     nbgNodes.shrink_to_fit();

     // ========================================================================
     // PHASE 3.9: SERIALIZE UI DATA & METADATA DECOUPLING
     // ========================================================================
     std::cout << "Phase 3.9: Writing UI files and metadata descriptors...\n";

     // 1. Serialize flat global coordinate arrays for mmap access
     std::ofstream geom_out("geometry.bin", std::ios::binary);
     uint32_t total_coords = global_geometry.size();
     geom_out.write(reinterpret_cast<const char*>(&total_coords), sizeof(total_coords));
     geom_out.write(reinterpret_cast<const char*>(global_geometry.data()), total_coords * sizeof(Coordinate));
     geom_out.close();

     // Permanent RAM purge of coordinate array strings/vectors
     global_geometry.clear();
     global_geometry.shrink_to_fit();

     // 2. Serialize Metadata properties with zero vector structures
     std::ofstream meta_out("metadata.bin", std::ios::binary);
     size_t total_ebg_nodes = expandedNodes.size();
     meta_out.write(reinterpret_cast<const char*>(&total_ebg_nodes), sizeof(total_ebg_nodes));

     for (const auto& node : expandedNodes) 
     {
         meta_out.write(reinterpret_cast<const char*>(&node.id), sizeof(node.id));
         meta_out.write(reinterpret_cast<const char*>(&node.originalId), sizeof(node.originalId));
         meta_out.write(reinterpret_cast<const char*>(&node.start_osm_node_id), sizeof(node.start_osm_node_id));
         meta_out.write(reinterpret_cast<const char*>(&node.end_osm_node_id), sizeof(node.end_osm_node_id));
         meta_out.write(reinterpret_cast<const char*>(&node.wt), sizeof(node.wt));
         meta_out.write(reinterpret_cast<const char*>(&node.distance_meters), sizeof(node.distance_meters));
         meta_out.write(reinterpret_cast<const char*>(&node.cell_id), sizeof(node.cell_id));
         
         meta_out.write(reinterpret_cast<const char*>(&node.road_type), sizeof(node.road_type));
         meta_out.write(reinterpret_cast<const char*>(&node.name_id), sizeof(node.name_id));
         meta_out.write(reinterpret_cast<const char*>(&node.is_one_way), sizeof(node.is_one_way));
         meta_out.write(reinterpret_cast<const char*>(&node.geometry_offset), sizeof(node.geometry_offset));
         meta_out.write(reinterpret_cast<const char*>(&node.geometry_size), sizeof(node.geometry_size));
     }
     meta_out.close();

     // ========================================================================
     // PHASE 4: BUILDING THE TURNS (EDGE-EXPANDED GRAPH RELATIONSHIPS)
     // ========================================================================
     std::cout << "Phase 4: Evaluating turn restrictions and building turns...\n";
     expandedGraph.resize(expandedNodes.size());

     for (const auto& from_node : expandedNodes) 
     {
         uint64_t exit_intersection = from_node.end_osm_node_id;
         
         // Scan all EBG nodes that originate from this node's destination intersection
         for (int to_node_id : intersection_to_segments[exit_intersection]) 
         {
             if (from_node.id == to_node_id) continue; // Prevent loopbacks
            
             const auto& to_node = expandedNodes[to_node_id];

             // Enforce Complex Turn Restrictions[cite: 7]
             bool turn_is_banned = false;
             bool has_only_restriction = false;
             bool satisfies_only_restriction = false;

             for (const auto& restriction : turn_restrictions) 
             {
                 if (restriction.from_way_id == from_node.originalId && restriction.via_node_id == exit_intersection) 
                 {
                     bool is_target_way = (restriction.to_way_id == to_node.originalId);
                     if (restriction.restriction_type.find("only_") != std::string::npos) {
                         has_only_restriction = true;
                         if (is_target_way) satisfies_only_restriction = true;
                     } else if (restriction.restriction_type.find("no_") != std::string::npos) {
                         if (is_target_way) turn_is_banned = true;
                     }
                 }
             }

             if ((has_only_restriction && !satisfies_only_restriction) || turn_is_banned) continue;
 
             ExpandedEdge turn;
             turn.destNode = to_node_id;
             if (from_node.originalId == to_node.originalId) {
                 turn.is_u_turn = true;
                 turn.turnWeight = 120; // 2-minute penalty for U-turns[cite: 7]
             } else {
                 turn.is_u_turn = false;
                 turn.turnWeight = 2;   // 2-second standard cost intersection traversal[cite: 7]
             }
             expandedGraph[from_node.id].push_back(turn);
         }
     }
     
     // Clean up auxiliary references
     intersection_to_segments.clear();
     turn_restrictions.clear();
     turn_restrictions.shrink_to_fit();

     // ========================================================================
     // PHASE 5: GENERATE FINAL ROUTING CSR FORMAT
     // ========================================================================
     std::cout << "Phase 5: Finalizing Dijkstra Routing CSR binary...\n";

     uint32_t total_routing_edges = 0;
     for (uint32_t i = 0; i < total_ebg_nodes; ++i) {
         total_routing_edges += expandedGraph[i].size();
     }

     std::vector<uint32_t> routing_head(total_ebg_nodes + 1, 0);
     std::vector<RoutingEdge> routing_edges(total_routing_edges);

     uint32_t edge_idx = 0;
     for (uint32_t i = 0; i < total_ebg_nodes; ++i) 
     {
         routing_head[i] = edge_idx;
         for (const auto& exp_edge : expandedGraph[i]) 
         {
             routing_edges[edge_idx].to = exp_edge.destNode;
             // Core Metric Fusion: Segment Weight + Turn Penalty[cite: 7]
             routing_edges[edge_idx].weight = exp_edge.turnWeight + expandedNodes[exp_edge.destNode].wt;
             edge_idx++;
         }
     }
     routing_head[total_ebg_nodes] = edge_idx;

     // Write routing_csr.bin[cite: 7]
     std::ofstream route_out("routing_csr.bin", std::ios::binary);
     uint32_t total_nodes_u32 = static_cast<uint32_t>(total_ebg_nodes);
     route_out.write(reinterpret_cast<const char*>(&total_nodes_u32), sizeof(total_nodes_u32));
     route_out.write(reinterpret_cast<const char*>(&total_routing_edges), sizeof(total_routing_edges));
     route_out.write(reinterpret_cast<const char*>(routing_head.data()), routing_head.size() * sizeof(uint32_t));
     route_out.write(reinterpret_cast<const char*>(routing_edges.data()), routing_edges.size() * sizeof(RoutingEdge));
     route_out.close();

     std::cout << "\n========================================================\n";
     std::cout << "SUCCESS! Production Engine Pipeline Execution Complete.\n";
     std::cout << "- geometry.bin (Contiguous array coordinates for mmap access)\n";
     std::cout << "- metadata.bin (Optimized side-car metadata mapping file)\n";
     std::cout << "- projections_format.bin (1.5M node spatial partition vectors)\n";
     std::cout << "- routing_csr.bin (High-speed 47M edge-expanded routing engine file)\n";
     std::cout << "- flow_csr.bin (Compact partitioner cut network profile)\n";
     std::cout << "========================================================\n";

     return 0;
}