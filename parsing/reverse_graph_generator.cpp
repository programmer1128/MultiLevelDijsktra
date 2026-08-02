#include <iostream>
#include <vector>
#include <fstream>
#include <stdexcept>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

// -------------------------------------------------------------------------
// 1. Reverses the Combined Base Graph (routing_csr.bin)
// -------------------------------------------------------------------------
void generate_reverse_base_graph(const std::string& input_file, const std::string& output_file) 
{
    std::cout << "Reading Base Graph: " << input_file << "...\n";
    
    std::ifstream in(input_file, std::ios::binary);
    if (!in) throw std::runtime_error("Cannot open: " + input_file);
    
    uint32_t total_nodes = 0, total_edges = 0;
    in.read(reinterpret_cast<char*>(&total_nodes), sizeof(uint32_t));
    in.read(reinterpret_cast<char*>(&total_edges), sizeof(uint32_t));
    
    std::vector<uint32_t> forward_head(total_nodes + 1);
    std::vector<RoutingEdge> forward_edges(total_edges);
    
    in.read(reinterpret_cast<char*>(forward_head.data()), forward_head.size() * sizeof(uint32_t));
    in.read(reinterpret_cast<char*>(forward_edges.data()), forward_edges.size() * sizeof(RoutingEdge));
    in.close();

    std::cout << "Reversing Base Graph in memory...\n";
    
    std::vector<uint32_t> in_degree(total_nodes, 0);
    for (uint32_t i = 0; i < total_edges; ++i) {
        in_degree[forward_edges[i].to]++;
    }

    std::vector<uint32_t> reverse_head(total_nodes + 1, 0);
    for (uint32_t i = 0; i < total_nodes; ++i) {
        reverse_head[i + 1] = reverse_head[i] + in_degree[i];
    }

    std::vector<RoutingEdge> reverse_edges(total_edges);
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

    std::cout << "Writing Reversed Base Graph: " << output_file << "...\n";
    
    std::ofstream out(output_file, std::ios::binary);
    out.write(reinterpret_cast<const char*>(&total_nodes), sizeof(uint32_t));
    out.write(reinterpret_cast<const char*>(&total_edges), sizeof(uint32_t));
    out.write(reinterpret_cast<const char*>(reverse_head.data()), reverse_head.size() * sizeof(uint32_t));
    out.write(reinterpret_cast<const char*>(reverse_edges.data()), reverse_edges.size() * sizeof(RoutingEdge));
    out.close();
}

// -------------------------------------------------------------------------
// 2. Reverses the Split Overlay Graph
// -------------------------------------------------------------------------
void generate_reverse_overlay_graph(const std::string& head_in_file, const std::string& edges_in_file, 
                                    const std::string& head_out_file, const std::string& edges_out_file) 
{
    std::cout << "Reading Overlay Graph...\n";
    
    std::ifstream head_in(head_in_file, std::ios::binary);
    if (!head_in) throw std::runtime_error("Cannot open: " + head_in_file);
    uint32_t total_nodes = 0, total_edges = 0;
    head_in.read(reinterpret_cast<char*>(&total_nodes), sizeof(uint32_t));
    head_in.read(reinterpret_cast<char*>(&total_edges), sizeof(uint32_t));
    std::vector<uint32_t> forward_head(total_nodes + 1);
    head_in.read(reinterpret_cast<char*>(forward_head.data()), forward_head.size() * sizeof(uint32_t));
    head_in.close();

    std::ifstream edges_in(edges_in_file, std::ios::binary);
    if (!edges_in) throw std::runtime_error("Cannot open: " + edges_in_file);
    std::vector<ShortcutEdge> forward_edges(total_edges);
    edges_in.read(reinterpret_cast<char*>(forward_edges.data()), forward_edges.size() * sizeof(ShortcutEdge));
    edges_in.close();

    std::cout << "Reversing Overlay Graph in memory...\n";
    
    std::vector<uint32_t> in_degree(total_nodes, 0);
    for (uint32_t i = 0; i < total_edges; ++i) {
        in_degree[forward_edges[i].to]++;
    }

    std::vector<uint32_t> reverse_head(total_nodes + 1, 0);
    for (uint32_t i = 0; i < total_nodes; ++i) {
        reverse_head[i + 1] = reverse_head[i] + in_degree[i];
    }

    std::vector<ShortcutEdge> reverse_edges(total_edges);
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

    std::cout << "Writing Reversed Overlay Graph...\n";
    
    std::ofstream head_out(head_out_file, std::ios::binary);
    head_out.write(reinterpret_cast<const char*>(&total_nodes), sizeof(uint32_t));
    head_out.write(reinterpret_cast<const char*>(&total_edges), sizeof(uint32_t));
    head_out.write(reinterpret_cast<const char*>(reverse_head.data()), reverse_head.size() * sizeof(uint32_t));
    head_out.close();

    std::ofstream edges_out(edges_out_file, std::ios::binary);
    edges_out.write(reinterpret_cast<const char*>(reverse_edges.data()), reverse_edges.size() * sizeof(ShortcutEdge));
    edges_out.close();
}

int main() 
{
    std::string base = "/home/aritra/MultiLevelDijsktra/parsing/";
    try 
    {
        // 1. Process Base Graph
        generate_reverse_base_graph(base + "routing_csr.bin", base + "reverse_routing_csr.bin");
        
        // 2. Process Overlay Graph
        generate_reverse_overlay_graph(
            base + "overlay_head.bin", base + "overlay_edges.bin", 
            base + "reverse_overlay_head.bin", base + "reverse_overlay_edges.bin"
        );
        
        std::cout << "\nSUCCESS: All reverse graphs completely synced and written to disk!\n";
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "Error: " << e.what() << "\n";
    }
    return 0;
}