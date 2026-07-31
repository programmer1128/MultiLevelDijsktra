#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

// Must perfectly match the structs used during the writing phase
struct Coordinate {
    double lat;
    double lon;
};

typedef struct ExpandedEdge {
    int destNode;
    uint32_t turnWeight;
    int16_t turnAngle;
    bool is_u_turn:1;
} ExpandedEdge;

int main() {
    std::string filename = "/home/aritra/MultiLevelDijsktra/parsing/engine_graph.bin";
    std::ifstream in_file(filename, std::ios::binary);

    if (!in_file) {
        std::cerr << "Error: Could not open " << filename << " for reading.\n";
        return 1;
    }

    std::cout << "--- Graph Integrity Verification ---\n";

    // 1. Read Total Nodes
    size_t total_nodes = 0;
    in_file.read(reinterpret_cast<char*>(&total_nodes), sizeof(total_nodes));
    std::cout << "Total Nodes Declared: " << total_nodes << "\n\n";

    uint64_t total_edges = 0;
    uint64_t total_geometry_points = 0;
    size_t max_edges_on_node = 0;

     // 2. Stream through the file sequentially
     for (size_t i = 0; i < total_nodes; ++i) 
     {
         uint32_t node_id;
         uint32_t node_wt;
         size_t geom_size;
         size_t edge_count;

         // Read Node Metadata
         in_file.read(reinterpret_cast<char*>(&node_id), sizeof(node_id));
         in_file.read(reinterpret_cast<char*>(&node_wt), sizeof(node_wt));

         // Read Geometry
         in_file.read(reinterpret_cast<char*>(&geom_size), sizeof(geom_size));
         std::vector<Coordinate> geometry(geom_size);
         in_file.read(reinterpret_cast<char*>(geometry.data()), geom_size * sizeof(Coordinate));

         // Read Edges
         in_file.read(reinterpret_cast<char*>(&edge_count), sizeof(edge_count));
         std::vector<ExpandedEdge> edges(edge_count);
         in_file.read(reinterpret_cast<char*>(edges.data()), edge_count * sizeof(ExpandedEdge));

         // Analytics tracking
         total_edges += edge_count;
         total_geometry_points += geom_size;
         if (edge_count > max_edges_on_node) 
         {
             max_edges_on_node = edge_count;
         }

         // Deep Inspection of the first 5 nodes to visually verify data isn't corrupted
         if (i < 5) 
         {
             std::cout << "[Node " << node_id << "]\n";
             std::cout << "  Traversal Weight: " << node_wt << " seconds\n";
             std::cout << "  Geometry Points : " << geom_size << "\n";
             if (geom_size > 0) 
             {
                 std::cout << "    Start Coord   : " << geometry.front().lat << ", " << geometry.front().lon << "\n";
                 std::cout << "    End Coord     : " << geometry.back().lat << ", " << geometry.back().lon << "\n";
             }
             std::cout << "  Valid Turns     : " << edge_count << "\n";
             for (size_t e = 0; e < edge_count; ++e) {
                 std::cout << "    -> to Node " << edges[e].destNode << " (Penalty: " << edges[e].turnWeight << "s)\n";
             }
             std::cout << "--------------------------------------\n";
         }
     }

     // 3. Ensure we reached the actual end of the file
     in_file.peek();
     bool is_eof = in_file.eof();
     in_file.close();

     // 4. Print Global Statistics
     std::cout << "\n--- Full Graph Summary ---\n";
     std::cout << "Total Edges (Turns)       : " << total_edges << "\n";
     std::cout << "Total Coordinate Points   : " << total_geometry_points << "\n";
     std::cout << "Max Branches on one road  : " << max_edges_on_node << "\n";
     std::cout << "File parsed completely?   : " << (is_eof ? "YES (Perfect Alignment)" : "NO (Data leftover/Corruption!)") << "\n";

     return 0;
}