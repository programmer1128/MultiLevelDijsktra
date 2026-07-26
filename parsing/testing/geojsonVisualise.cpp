#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include <cstdint>

// Replicate your basic structs
struct FlowEdge {
    uint32_t to;
    uint32_t cap;
    uint32_t flow;
    uint32_t rev_idx;
};

struct ProjectedNode {
    uint32_t id;
    double projectedValue;
};

struct Coordinate {
    double lat;
    double lon;
};

int main() 
{
    std::string base_path = "/home/aritra/MultiLevelDijsktra/parsing/";
    
    // 1. Load the Cell Assignments
    std::cout << "Loading partition_cells.bin...\n";
    std::ifstream cell_in(base_path + "node_cell_mapping.bin", std::ios::binary);
    if (!cell_in) { std::cerr << "Missing partition_cells.bin\n"; return 1; }
    
    // Assuming you wrote the size first, then the flat array of uint16_t
    uint32_t num_nodes = 0;
    cell_in.read(reinterpret_cast<char*>(&num_nodes), sizeof(num_nodes));
    std::vector<uint16_t> cell_ids(num_nodes);
    cell_in.read(reinterpret_cast<char*>(cell_ids.data()), num_nodes * sizeof(uint16_t));
    cell_in.close();

    // 2. Load the Flow Graph (Topology)
    std::cout << "Loading flow_csr.bin...\n";
    std::ifstream flow_in(base_path + "flow_csr.bin", std::ios::binary);
    uint32_t csr_nodes = 0, total_edges = 0;
    flow_in.read(reinterpret_cast<char*>(&csr_nodes), sizeof(csr_nodes));
    flow_in.read(reinterpret_cast<char*>(&total_edges), sizeof(total_edges));
    
    std::vector<uint32_t> head(csr_nodes + 1);
    std::vector<FlowEdge> edges(total_edges);
    flow_in.read(reinterpret_cast<char*>(head.data()), head.size() * sizeof(uint32_t));
    flow_in.read(reinterpret_cast<char*>(edges.data()), edges.size() * sizeof(FlowEdge));
    flow_in.close();

    // 3. Reconstruct Coordinates from Projections
    std::cout << "Reconstructing coordinates from projections_format.bin...\n";
    std::ifstream proj_in(base_path + "projections_format.bin", std::ios::binary);
    uint32_t num_axes = 0, num_proj_nodes = 0;
    proj_in.read(reinterpret_cast<char*>(&num_axes), sizeof(num_axes));
    proj_in.read(reinterpret_cast<char*>(&num_proj_nodes), sizeof(num_proj_nodes));

    std::vector<Coordinate> coords(num_nodes);
    std::vector<ProjectedNode> temp_proj(num_proj_nodes);

    // Read Axis 0 (Longitude)
    proj_in.read(reinterpret_cast<char*>(temp_proj.data()), num_proj_nodes * sizeof(ProjectedNode));
    for (const auto& pn : temp_proj) coords[pn.id].lon = pn.projectedValue;

    // Read Axis 90 (Latitude)
    proj_in.read(reinterpret_cast<char*>(temp_proj.data()), num_proj_nodes * sizeof(ProjectedNode));
    for (const auto& pn : temp_proj) coords[pn.id].lat = pn.projectedValue;
    proj_in.close();

    // 4. Generate the GeoJSON
    std::cout << "Exporting boundary cuts to GeoJSON...\n";
    std::ofstream geo_out("boundary_cuts.geojson");
    geo_out << "{ \"type\": \"FeatureCollection\", \"features\": [\n";
    bool first = true;

    for (uint32_t u = 0; u < num_nodes; ++u) 
    {
        for (uint32_t e = head[u]; e < head[u+1]; ++e) 
        {
            uint32_t v = edges[e].to;

            // Is this a cut edge?
            if (cell_ids[u] != cell_ids[v]) 
            {
                // Print only u < v to prevent duplicate lines for the same physical road
                if (u < v) 
                {
                    if (!first) geo_out << ",\n";
                    
                    geo_out << "  { \"type\": \"Feature\", \"properties\": { \"from_cell\": " << cell_ids[u] 
                            << ", \"to_cell\": " << cell_ids[v] << " }, \"geometry\": { \"type\": \"LineString\", \"coordinates\": [ ";
                    
                    geo_out << "[" << std::fixed << std::setprecision(6) << coords[u].lon << ", " << coords[u].lat << "], ";
                    geo_out << "[" << std::fixed << std::setprecision(6) << coords[v].lon << ", " << coords[v].lat << "] ";
                    
                    geo_out << "] } }";
                    first = false;
                }
            }
        }
    }
    
    geo_out << "\n] }\n";
    geo_out.close();
    
    std::cout << "GeoJSON export complete! Open boundary_cuts.geojson to view.\n";
    return 0;
}