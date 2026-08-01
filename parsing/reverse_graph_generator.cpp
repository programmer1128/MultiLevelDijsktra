#include <iostream>
#include <vector>
#include <fstream>
#include <stdexcept>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp" // Ensure this has RoutingEdge and ShortcutEdge

template <typename EdgeType>
void generate_reverse_csr_split(const std::string& head_in_file, const std::string& edges_in_file, 
                                const std::string& head_out_file, const std::string& edges_out_file) 
{
    std::cout << "Reading " << head_in_file << " and " << edges_in_file << "...\n";
    
    // 1. Read Head Array
    std::ifstream head_in(head_in_file, std::ios::binary);
    if (!head_in) throw std::runtime_error("Cannot open: " + head_in_file);
    uint32_t total_nodes = 0, total_edges = 0;
    head_in.read(reinterpret_cast<char*>(&total_nodes), sizeof(uint32_t));
    head_in.read(reinterpret_cast<char*>(&total_edges), sizeof(uint32_t));
    std::vector<uint32_t> forward_head(total_nodes + 1);
    head_in.read(reinterpret_cast<char*>(forward_head.data()), forward_head.size() * sizeof(uint32_t));
    head_in.close();

    // 2. Read Edges Array
    std::ifstream edges_in(edges_in_file, std::ios::binary);
    if (!edges_in) throw std::runtime_error("Cannot open: " + edges_in_file);
    std::vector<EdgeType> forward_edges(total_edges);
    edges_in.read(reinterpret_cast<char*>(forward_edges.data()), forward_edges.size() * sizeof(EdgeType));
    edges_in.close();

    std::cout << "Reversing graph in memory...\n";
    
    std::vector<uint32_t> in_degree(total_nodes, 0);
    for (uint32_t i = 0; i < total_edges; ++i) {
        in_degree[forward_edges[i].to]++;
    }

    std::vector<uint32_t> reverse_head(total_nodes + 1, 0);
    for (uint32_t i = 0; i < total_nodes; ++i) {
        reverse_head[i + 1] = reverse_head[i] + in_degree[i];
    }

    std::vector<EdgeType> reverse_edges(total_edges);
    std::vector<uint32_t> tail = reverse_head; 

    for (uint32_t u = 0; u < total_nodes; ++u) {
        for (uint32_t i = forward_head[u]; i < forward_head[u + 1]; ++i) {
            uint32_t v = forward_edges[i].to;
            uint32_t weight = forward_edges[i].weight;
            
            uint32_t insert_idx = tail[v]++;
            reverse_edges[insert_idx].to = u;         
            reverse_edges[insert_idx].weight = weight; 
        }
    }

    std::cout << "Writing reversed outputs...\n";
    
    // Write Reversed Head
    std::ofstream head_out(head_out_file, std::ios::binary);
    head_out.write(reinterpret_cast<const char*>(&total_nodes), sizeof(uint32_t));
    head_out.write(reinterpret_cast<const char*>(&total_edges), sizeof(uint32_t));
    head_out.write(reinterpret_cast<const char*>(reverse_head.data()), reverse_head.size() * sizeof(uint32_t));
    head_out.close();

    // Write Reversed Edges
    std::ofstream edges_out(edges_out_file, std::ios::binary);
    edges_out.write(reinterpret_cast<const char*>(reverse_edges.data()), reverse_edges.size() * sizeof(EdgeType));
    edges_out.close();

    std::cout << "Successfully created reverse overlay files!\n\n";
}

int main() {
    try {
        generate_reverse_csr_split<ShortcutEdge>(
            "overlay_head.bin", "overlay_edges.bin", 
            "reverse_overlay_head.bin", "reverse_overlay_edges.bin"
        );
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }
    return 0;
}