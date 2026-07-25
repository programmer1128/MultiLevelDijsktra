#include<bits/stdc++.h>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"



enum ProjectionAxis 
{
     AXIS_0_DEG = 0,
     AXIS_90_DEG = 1,
     AXIS_45_DEG = 2,
     AXIS_135_DEG = 3
};

void generateAndSortProjections(std::vector<ExpandedNode>&nodes,
         std::vector<std::vector<ProjectedNode>>& sorted_projections)
{
     size_t num_nodes = nodes.size();
    
     //initialize 4 projection arrays
     sorted_projections.resize(4);
     for (int i = 0; i < 4; ++i) 
     {
         sorted_projections[i].reserve(num_nodes);
     }

     std::cout<<"Computing centroids\n";

     for(const auto&node:nodes)
     {
         double sum_lat = 0.0;
         double sum_lon = 0.0;
         size_t geom_size = node.geometry.size();

         // Safety check to prevent division by zero
         if (geom_size == 0) continue; 

         for (const auto& coord : node.geometry) 
         {
             sum_lat += coord.lat;
             sum_lon += coord.lon;
         }

         double centroid_lat = sum_lat / geom_size;
         double centroid_lon = sum_lon / geom_size;

         //apply Optimized Projections
         sorted_projections[AXIS_0_DEG].push_back({node.id, centroid_lon});
         sorted_projections[AXIS_90_DEG].push_back({node.id, centroid_lat});
         sorted_projections[AXIS_45_DEG].push_back({node.id, centroid_lon + centroid_lat});
         sorted_projections[AXIS_135_DEG].push_back({node.id, -centroid_lon + centroid_lat});
     }

     auto cmp=[&](const ProjectedNode&a,const ProjectedNode&b)->bool
     {
         return a.projectedValue<b.projectedValue;  
     };

     for (int i = 0; i < 4; ++i) 
     {
         std::sort(sorted_projections[i].begin(), sorted_projections[i].end(), cmp);
     }
     std::cout<<"Projection sorting done\n";
}

void saveProjectionsToBinary(const std::vector<std::vector<ProjectedNode>>& sorted_projections, 
                             const std::string& filename) 
{
     std::ofstream out_file(filename, std::ios::binary);
     if (!out_file) 
     {
         std::cerr << "Error: Failed to open " << filename << " for writing.\n";
         return;
     }

     uint32_t num_axes = sorted_projections.size(); // Will be 4
     uint32_t num_nodes = sorted_projections[0].size();

     //number of axes and number of nodes per axis
     out_file.write(reinterpret_cast<const char*>(&num_axes), sizeof(num_axes));
     out_file.write(reinterpret_cast<const char*>(&num_nodes), sizeof(num_nodes));

     // Write the raw bytes for each sorted array
     for (uint32_t i = 0; i < num_axes; ++i) 
     {
         out_file.write(reinterpret_cast<const char*>(sorted_projections[i].data()), 
                       num_nodes * sizeof(ProjectedNode));
     }

     out_file.close();
     std::cout << "Projections successfully saved to " << filename << ".\n";
}


void loadGraphFromBinary(const std::string& filename, 
                         std::vector<ExpandedNode>& nodes, 
                         std::vector<std::vector<ExpandedEdge>>& graph) 
{
     std::ifstream in_file(filename, std::ios::binary);
     if (!in_file) 
     {
         std::cerr << "Error: Failed to open " << filename << " for reading.\n";
         return;
     }

     //Read total nodes
     size_t total_nodes = 0;
     in_file.read(reinterpret_cast<char*>(&total_nodes), sizeof(total_nodes));

     //Pre-allocate the vectors to avoid reallocation overhead
     nodes.resize(total_nodes);
     graph.resize(total_nodes);

     for(size_t i = 0; i < total_nodes; ++i) 
     {
         ExpandedNode node;
        
         //Read node metadata
         in_file.read(reinterpret_cast<char*>(&node.id), sizeof(node.id));
         in_file.read(reinterpret_cast<char*>(&node.wt), sizeof(node.wt));
        
         //Read geometry array
         size_t geom_size = 0;
         in_file.read(reinterpret_cast<char*>(&geom_size), sizeof(geom_size));
        
         node.geometry.resize(geom_size);
         in_file.read(reinterpret_cast<char*>(node.geometry.data()), geom_size * sizeof(Coordinate));

         // Place the loaded node into the array
         nodes[node.id] = node; 

         //Read edges for this node
         size_t edge_count = 0;
         in_file.read(reinterpret_cast<char*>(&edge_count), sizeof(edge_count));
        
         graph[node.id].resize(edge_count);
         in_file.read(reinterpret_cast<char*>(graph[node.id].data()), edge_count * sizeof(ExpandedEdge));
     }

     in_file.close();
     std::cout << "Successfully loaded " << total_nodes << " nodes into memory.\n";
}

void buildAndSaveCSR(const std::vector<std::vector<ExpandedEdge>>& expandedGraph, 
                     const std::string& filename) 
{
     uint32_t num_nodes = expandedGraph.size();
    
     // We add +1 to head array to easily calculate the degree of the last node
     std::vector<uint32_t> head(num_nodes + 1, 0); 
     std::vector<uint32_t> degree(num_nodes, 0);

     std::cout << "Pass 1: Calculating degrees for exact memory allocation...\n";
     for (uint32_t u = 0; u < num_nodes; ++u) 
     {
         for (const auto& edge : expandedGraph[u]) 
         {
             uint32_t v = edge.destNode;
             degree[u]++; // Forward edge u -> v
             degree[v]++; // Residual edge v -> u
         }
     }

     // Prefix sum to build the head pointers
     for (uint32_t i = 0; i < num_nodes; ++i) 
     {
         head[i + 1] = head[i] + degree[i];
     }

     uint32_t total_flow_edges = head[num_nodes];
     std::vector<FlowEdge> edges(total_flow_edges);
    
     // Tail array to keep track of insertion points
     std::vector<uint32_t> tail = head; 

     std::cout << "Pass 2: Populating and cross-linking residual edges...\n";
     for (uint32_t u = 0; u < num_nodes; ++u) 
     {
         for (const auto& edge : expandedGraph[u]) 
         {
             uint32_t v = edge.destNode;

             uint32_t idx_uv = tail[u]++;
             uint32_t idx_vu = tail[v]++;

             // Forward Edge (Capacity = 1)
             edges[idx_uv] = {v, 1, 0, idx_vu};
            
             // Residual Edge (Capacity = 0)
             edges[idx_vu] = {u, 0, 0, idx_uv};
         }
     }

     std::cout << "Saving CSR Flow Network to binary\n";
     std::ofstream out_file(filename, std::ios::binary);
    
     out_file.write(reinterpret_cast<const char*>(&num_nodes), sizeof(num_nodes));
     out_file.write(reinterpret_cast<const char*>(&total_flow_edges), sizeof(total_flow_edges));
    
     out_file.write(reinterpret_cast<const char*>(head.data()), head.size() * sizeof(uint32_t));
     out_file.write(reinterpret_cast<const char*>(edges.data()), edges.size() * sizeof(FlowEdge));
    
     out_file.close();
     std::cout << "CSR successfully saved. (" << total_flow_edges << " total flow edges)\n";
}

int main()
{ 
     std::string base_path = "/home/aritra/MultiLevelDijsktra/parsing/";
     loadGraphFromBinary(base_path + "engine_graph.bin", expandedNodes, expandedGraph);

     //convert expanded edge graph to CSR format
     buildAndSaveCSR(expandedGraph, base_path + "csr_format.bin");

     //Inertial Flow projections
     std::vector<std::vector<ProjectedNode>> sorted_projections;
     generateAndSortProjections(expandedNodes, sorted_projections);
     saveProjectionsToBinary(sorted_projections, base_path + "projections_format.bin");

     std::cout << "\nOffline Preprocessing Pipeline Complete.\n";
     return 0;    
}