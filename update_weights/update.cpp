#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

struct RoutingEdge {
    uint32_t to;
    uint32_t weight;
};

int main() {
    // 1. Open metadata.bin to extract the structural OSM Way IDs
    std::ifstream meta_in("/home/aritra/MultiLevelDijsktra/parsing/metadata.bin", std::ios::binary);
    if (!meta_in) {
        std::cerr << "CRITICAL ERROR: Could not open metadata.bin\n";
        return 1;
    }

    size_t total_ebg_nodes = 0;
    meta_in.read(reinterpret_cast<char*>(&total_ebg_nodes), sizeof(total_ebg_nodes));
    std::vector<uint64_t> original_way_ids(total_ebg_nodes);
    
    // Read the metadata fields sequentially exactly as they were written to avoid struct padding
    for (size_t i = 0; i < total_ebg_nodes; ++i) {
        uint32_t id, wt, dist, cell, name, offset, size;
        uint64_t orig, start, end;
        uint8_t r_type;
        bool o_way;

        meta_in.read(reinterpret_cast<char*>(&id), 4);
        meta_in.read(reinterpret_cast<char*>(&orig), 8); // This is the OSM Way ID
        meta_in.read(reinterpret_cast<char*>(&start), 8);
        meta_in.read(reinterpret_cast<char*>(&end), 8);
        meta_in.read(reinterpret_cast<char*>(&wt), 4);
        meta_in.read(reinterpret_cast<char*>(&dist), 4);
        meta_in.read(reinterpret_cast<char*>(&cell), 4);
        meta_in.read(reinterpret_cast<char*>(&r_type), 1);
        meta_in.read(reinterpret_cast<char*>(&name), 4);
        meta_in.read(reinterpret_cast<char*>(&o_way), 1);
        meta_in.read(reinterpret_cast<char*>(&offset), 4);
        meta_in.read(reinterpret_cast<char*>(&size), 4);

        original_way_ids[i] = orig;
    }
    meta_in.close();
    std::cout << "Successfully parsed " << total_ebg_nodes << " nodes from metadata.\n";

    // 2. Open routing_csr.bin in read/write mode
    std::fstream route_file("/home/aritra/MultiLevelDijsktra/parsing/routing_csr.bin", std::ios::in | std::ios::out | std::ios::binary);
    if (!route_file) {
        std::cerr << "CRITICAL ERROR: Could not open routing_csr.bin\n";
        return 1;
    }

    uint32_t total_nodes_u32 = 0;
    uint32_t total_routing_edges = 0;
    route_file.read(reinterpret_cast<char*>(&total_nodes_u32), sizeof(total_nodes_u32));
    route_file.read(reinterpret_cast<char*>(&total_routing_edges), sizeof(total_routing_edges));

    if (total_nodes_u32 != total_ebg_nodes) {
        std::cerr << "CRITICAL ERROR: Graph size mismatch between metadata and CSR file!\n";
        return 1;
    }

    std::vector<uint32_t> routing_head(total_nodes_u32 + 1);
    route_file.read(reinterpret_cast<char*>(routing_head.data()), routing_head.size() * sizeof(uint32_t));

    std::vector<RoutingEdge> routing_edges(total_routing_edges);
    route_file.read(reinterpret_cast<char*>(routing_edges.data()), routing_edges.size() * sizeof(RoutingEdge));

    // 3. Scan the CSR network structure in RAM and patch the weights
    uint32_t patched_count = 0;
    for (uint32_t u = 0; u < total_nodes_u32; ++u) {
        uint32_t start_idx = routing_head[u];
        uint32_t end_idx = routing_head[u + 1];

        for (uint32_t idx = start_idx; idx < end_idx; ++idx) {
            uint32_t v = routing_edges[idx].to;

            // If the source segment and target segment belong to the same OSM Way, it's a U-Turn
            if (original_way_ids[u] == original_way_ids[v]) {
                // Deduct the 120-second penalty, add the 5-second penalty (-115 total)
                routing_edges[idx].weight = routing_edges[idx].weight - 120 + 5;
                patched_count++;
            }
        }
    }

    // 4. Seek past headers and routing_head to write updated edges back to disk
    std::streamoff edges_offset = sizeof(uint32_t) + sizeof(uint32_t) + (routing_head.size() * sizeof(uint32_t));
    route_file.seekp(edges_offset, std::ios::beg);
    route_file.write(reinterpret_cast<const char*>(routing_edges.data()), routing_edges.size() * sizeof(RoutingEdge));
    route_file.close();

    std::cout << "SUCCESS: Patched " << patched_count << " U-turn edges down to 5 seconds directly in routing_csr.bin!\n";
    return 0;
}