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
     uint32_t current_node_id = 0;

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

         std::vector<Coordinate> current_geometry;
         uint64_t segment_start_osm_id = way.nodes().front().ref();
         uint32_t segment_distance = 0;

         for (size_t i = 0; i < way.nodes().size(); ++i) 
         {
             const auto& node_ref = way.nodes()[i];
            
             Coordinate coord = 
             { 
                 static_cast<double>(node_ref.lat()), 
                 static_cast<double>(node_ref.lon()) 
             }; 
            
             current_geometry.push_back(coord);

             // Calculate distance if this is not the first node in the segment
             if (i > 0) 
             {
                 const auto& prev_node_ref = way.nodes()[i - 1];
                
                 segment_distance += calculate_distance(
                     prev_node_ref.lat(), prev_node_ref.lon(),
                     node_ref.lat(),      node_ref.lon()
                 );
             }
             // Split the way if this node is an intersection (count >= 2) AND it's not the very first node of the way,
             // OR if it's the absolute last node of the way.
             bool is_intersection = (node_counts[node_ref.ref()] >= 2);
             bool is_last_node = (i == way.nodes().size() - 1);

             if ((is_intersection && i != 0) || is_last_node) 
             {
                
                 float speed_m_s = 11.0f;

                 // 2. Check if OSM provides a specific speed limit for this road
                 const char* maxspeed_tag = way.tags().get_value_by_key("maxspeed");
                 if (maxspeed_tag) 
                 {
                     // std::atoi is safe here. If the tag is a weird string like "none", 
                     // it simply returns 0 rather than crashing the program.
                      int speed_kmh = std::atoi(maxspeed_tag);
                    
                     // Ensure it's a valid, positive speed to avoid division by zero
                     if (speed_kmh > 0) 
                     {
                         speed_m_s = speed_kmh / 3.6f; // Convert km/h to m/s
                     } 
                 }

                 ExpandedNode forward_node;
                 forward_node.id = current_node_id++;
                 forward_node.originalId = way.id();
                 forward_node.start_osm_node_id = segment_start_osm_id;
                 forward_node.end_osm_node_id = node_ref.ref();
                 forward_node.distance_meters = segment_distance;
                 forward_node.wt = std::max(1u, static_cast<uint32_t>(segment_distance / speed_m_s));
                 forward_node.is_one_way = true; // Structurally, this node is now one-way
                 forward_node.geometry = current_geometry;
                 expandedNodes.push_back(forward_node);

                 //If it is a two-way street, generate the rev segment
                 if (!is_oneway) 
                 {
                     ExpandedNode reverse_node;
                     reverse_node.id = current_node_id++;
                     reverse_node.originalId = way.id();
                    
                     //swapping the start and end intersections
                     reverse_node.start_osm_node_id = node_ref.ref();
                     reverse_node.end_osm_node_id = segment_start_osm_id;
                     
                     reverse_node.distance_meters = segment_distance;
                     reverse_node.wt = forward_node.wt;
                     reverse_node.is_one_way = true; 
                      
                     //rev the geometry so the map draws it correctly
                     reverse_node.geometry = current_geometry;
                     std::reverse(reverse_node.geometry.begin(), reverse_node.geometry.end());
                    
                     expandedNodes.push_back(reverse_node);
                 }
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


     location_index.clear();
     node_counts.clear();

     // --- Build the Edge-Expanded Graph ---
     std::cout << "Building turns (ExpandedEdges)...\n";
     expandedGraph.resize(expandedNodes.size());

     // A lookup map to quickly find which ExpandedNodes start at a specific OSM intersection
     std::unordered_map<uint64_t, std::vector<int>> intersection_to_segments;
     for (const auto& node : expandedNodes) 
     {
         intersection_to_segments[node.start_osm_node_id].push_back(node.id);
     }



     for (const auto& from_node : expandedNodes) 
     {
         uint64_t exit_intersection = from_node.end_osm_node_id;
        
         // Look at all roads that start at the intersection where from_node ends
         for (int to_node_id : intersection_to_segments[exit_intersection]) 
         {
             if (from_node.id == to_node_id) continue; // Prevent instant U-turns into the exact same segment
            
             const auto& to_node = expandedNodes[to_node_id];

             // --- ENFORCE TURN RESTRICTIONS (The "Baking" Step) ---
             bool turn_is_banned = false;
             bool has_only_restriction = false;
             bool satisfies_only_restriction = false;

             for (const auto& restriction : turn_restrictions) 
             {
                 if (restriction.from_way_id == from_node.originalId && restriction.via_node_id == exit_intersection) 
                 {
                     bool is_target_way = (restriction.to_way_id == to_node.originalId);
                     
                     if (restriction.restriction_type.find("only_") != std::string::npos) 
                     {
                         has_only_restriction = true;
                         if (is_target_way) satisfies_only_restriction = true;
                     } 
                     else if (restriction.restriction_type.find("no_") != std::string::npos) 
                     {
                         if (is_target_way) turn_is_banned = true;
                     }
                 }
             }

             // If there is an "only_" restriction, and this turn isn't it, ban it.
             if (has_only_restriction && !satisfies_only_restriction) 
             {
                 turn_is_banned = true;
             }

             if (turn_is_banned) 
             {
                 continue;
             }
             // -----------------------------------------------------
 
             // The turn is perfectly legal. Create the edge.
             ExpandedEdge turn;
             turn.destNode = to_node_id;

             // --- U-TURN DETECTION ---
             if (from_node.originalId == to_node.originalId) 
             {
                 turn.is_u_turn = true;
                 turn.turnWeight = 120; // 120-second penalty (2 minutes) to strongly discourage U-turns
             }  
             else 
             {
                 turn.is_u_turn = false;
                 turn.turnWeight = 2;   // Standard 2-second structural penalty for normal turns
             }

             expandedGraph[from_node.id].push_back(turn);
         }
     }






     intersection_to_segments.clear();


     std::cout << "Saving Graph to binary file...\n";
     std::ofstream out_file("engine_graph.bin", std::ios::binary);

     //write total nodes
     size_t total_nodes = expandedNodes.size();
     out_file.write(reinterpret_cast<const char*>(&total_nodes), sizeof(total_nodes));

     for (const auto& node : expandedNodes) 
     {
         //all the meta data of the nodes and edges
         out_file.write(reinterpret_cast<const char*>(&node.id), sizeof(node.id));
         out_file.write(reinterpret_cast<const char*>(&node.originalId), sizeof(node.originalId));
         out_file.write(reinterpret_cast<const char*>(&node.start_osm_node_id), sizeof(node.start_osm_node_id));
         out_file.write(reinterpret_cast<const char*>(&node.end_osm_node_id), sizeof(node.end_osm_node_id));
         out_file.write(reinterpret_cast<const char*>(&node.wt), sizeof(node.wt));
         out_file.write(reinterpret_cast<const char*>(&node.distance_meters), sizeof(node.distance_meters));
         out_file.write(reinterpret_cast<const char*>(&node.road_type), sizeof(node.road_type));
         out_file.write(reinterpret_cast<const char*>(&node.name_id), sizeof(node.name_id));
         out_file.write(reinterpret_cast<const char*>(&node.is_one_way), sizeof(node.is_one_way));
        
         // Write geometry array size, then the raw geometry bytes
         size_t geom_size = node.geometry.size();
         out_file.write(reinterpret_cast<const char*>(&geom_size), sizeof(geom_size));
         out_file.write(reinterpret_cast<const char*>(node.geometry.data()), geom_size * sizeof(Coordinate));

         // Write edges for this node (This writes the whole struct block safely)
         size_t edge_count = expandedGraph[node.id].size();
         out_file.write(reinterpret_cast<const char*>(&edge_count), sizeof(edge_count));
         out_file.write(reinterpret_cast<const char*>(expandedGraph[node.id].data()), edge_count * sizeof(ExpandedEdge));
     }

     out_file.close();
     std::cout << "Offline Preprocessing Complete. Graph saved to engine_graph.bin\n";
     
     return 0;
}