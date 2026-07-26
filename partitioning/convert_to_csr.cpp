#include<bits/stdc++.h>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

// --- Streamlined Data Structures ---
// We only store the projection values and a lightweight adjacency list


enum ProjectionAxis 
{
     AXIS_0_DEG = 0,
     AXIS_90_DEG = 1,
     AXIS_45_DEG = 2,
     AXIS_135_DEG = 3
};

// --- Updated Core Logic ---

void processAndSaveProjections(std::vector<std::vector<ProjectedNode>>& sorted_projections, 
                               const std::string& filename) 
{
     auto cmp = [&](const ProjectedNode& a, const ProjectedNode& b) -> bool {
         return a.projectedValue < b.projectedValue;  
     };

     std::cout << "Sorting Projections...\n";
     for (int i = 0; i < 4; ++i) {
         std::sort(sorted_projections[i].begin(), sorted_projections[i].end(), cmp);
     }

     std::cout << "Saving Projections to " << filename << "...\n";
     std::ofstream out_file(filename, std::ios::binary);
     if (!out_file) return;

     uint32_t num_axes = sorted_projections.size(); 
     uint32_t num_nodes = sorted_projections[0].size();

     out_file.write(reinterpret_cast<const char*>(&num_axes), sizeof(num_axes));
     out_file.write(reinterpret_cast<const char*>(&num_nodes), sizeof(num_nodes));

     for (uint32_t i = 0; i < num_axes; ++i) {
         out_file.write(reinterpret_cast<const char*>(sorted_projections[i].data()), 
                        num_nodes * sizeof(ProjectedNode));
     }
     out_file.close();
}


void buildAndSaveCSR(const std::vector<std::vector<uint32_t>>& lightGraph, 
                     const std::string& filename) 
{
     uint32_t num_nodes = lightGraph.size();
     std::vector<uint32_t> head(num_nodes + 1, 0); 
     std::vector<uint32_t> degree(num_nodes, 0);

     std::cout << "Pass 1: Calculating degrees for CSR allocation...\n";
     for (uint32_t u = 0; u < num_nodes; ++u) {
         for (uint32_t v : lightGraph[u]) {
             degree[u]++; // Forward edge u -> v
             degree[v]++; // Residual edge v -> u
         }
     }

     for (uint32_t i = 0; i < num_nodes; ++i) {
         head[i + 1] = head[i] + degree[i];
     }

     uint32_t total_flow_edges = head[num_nodes];
     std::vector<FlowEdge> edges(total_flow_edges);
     std::vector<uint32_t> tail = head; 

     std::cout << "Pass 2: Populating residual edges...\n";
     for (uint32_t u = 0; u < num_nodes; ++u) {
         for (uint32_t v : lightGraph[u]) {
             uint32_t idx_uv = tail[u]++;
             uint32_t idx_vu = tail[v]++;

             edges[idx_uv] = {v, 1, 0, idx_vu};
             edges[idx_vu] = {u, 0, 0, idx_uv};
         }
     }

     std::cout << "Saving CSR to " << filename << "...\n";
     std::ofstream out_file(filename, std::ios::binary);
     out_file.write(reinterpret_cast<const char*>(&num_nodes), sizeof(num_nodes));
     out_file.write(reinterpret_cast<const char*>(&total_flow_edges), sizeof(total_flow_edges));
     out_file.write(reinterpret_cast<const char*>(head.data()), head.size() * sizeof(uint32_t));
     out_file.write(reinterpret_cast<const char*>(edges.data()), edges.size() * sizeof(FlowEdge));
     out_file.close();
}


void streamAndConvertGraph(const std::string& input_file, 
                           const std::string& csr_file, 
                           const std::string& proj_file) 
{
     std::ifstream in_file(input_file, std::ios::binary);
     if (!in_file) {
         std::cerr << "Failed to open " << input_file << "\n";
         return;
     }

     size_t total_nodes = 0;
     in_file.read(reinterpret_cast<char*>(&total_nodes), sizeof(total_nodes));

     // 1. Setup the lightweight arrays
     std::vector<std::vector<uint32_t>> lightGraph(total_nodes);
     std::vector<std::vector<ProjectedNode>> sorted_projections(4);
     for (int i = 0; i < 4; ++i) {
         sorted_projections[i].reserve(total_nodes);
     }

     std::cout << "Streaming " << total_nodes << " nodes from SSD (Bypassing heavy metadata)...\n";

     std::vector<ExpandedEdge> temp_edges;

     for(size_t i = 0; i < total_nodes; ++i) 
     {
         uint32_t node_id;
         in_file.read(reinterpret_cast<char*>(&node_id), sizeof(node_id));
         
         // 2. SKIP NEW METADATA (originalId(8) + start_osm(8) + end_osm(8) + wt(4) + dist(4) + type(1) + name(4) + oneway(1) = 38 bytes)
         in_file.seekg(38, std::ios::cur);

         // 3. READ GEOMETRY SIZE
         size_t geom_size = 0;
         in_file.read(reinterpret_cast<char*>(&geom_size), sizeof(geom_size));
        
         // 4. GRAB FIRST COORD & SKIP THE REST
         Coordinate first_coord = {0.0, 0.0};
         if (geom_size > 0) {
             in_file.read(reinterpret_cast<char*>(&first_coord), sizeof(Coordinate));
             if (geom_size > 1) {
                 in_file.seekg((geom_size - 1) * sizeof(Coordinate), std::ios::cur);
             }
         }

         // Calculate Projections immediately
         sorted_projections[AXIS_0_DEG].push_back({node_id, first_coord.lon});
         sorted_projections[AXIS_90_DEG].push_back({node_id, first_coord.lat});
         sorted_projections[AXIS_45_DEG].push_back({node_id, first_coord.lon + first_coord.lat});
         sorted_projections[AXIS_135_DEG].push_back({node_id, -first_coord.lon + first_coord.lat});

         // 5. READ EDGES
         size_t edge_count = 0;
         in_file.read(reinterpret_cast<char*>(&edge_count), sizeof(edge_count));
        
         temp_edges.resize(edge_count);
         in_file.read(reinterpret_cast<char*>(temp_edges.data()), edge_count * sizeof(ExpandedEdge));
         
         // Extract only the destination node for the CSR builder
         lightGraph[node_id].reserve(edge_count);
         for (const auto& exp_edge : temp_edges) {
             lightGraph[node_id].push_back(exp_edge.destNode);
         }
     }
     in_file.close();

     // Execute saving pipelines
     processAndSaveProjections(sorted_projections, proj_file);
     buildAndSaveCSR(lightGraph, csr_file);
}

int main()
{ 
     std::string base_path = "/home/aritra/MultiLevelDijsktra/parsing/";
     
     streamAndConvertGraph(base_path + "engine_graph.bin", 
                           base_path + "csr_format.bin", 
                           base_path + "projections_format.bin");

     std::cout << "\nConversion Complete. Ready for Dinic Partitioner.\n";
     return 0;    
}