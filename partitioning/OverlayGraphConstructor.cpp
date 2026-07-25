#include<bits/stdc++.h>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

uint32_t num_cells=9200;

void build_Overlay_Graph()
{
     
}


void convert_To_Routing_CSR(const std::vector<ExpandedNode>& nodes, 
                            const std::vector<std::vector<ExpandedEdge>>& graph, 
                            std::vector<uint32_t>& head, 
                            std::vector<RoutingEdge>& edges) 
{
     uint32_t total_nodes = graph.size();
    
     // Pass 1: Count total edges to allocate memory exactly once
     uint32_t total_edges = 0;
     for (uint32_t i = 0; i < total_nodes; ++i) 
     {
         total_edges += graph[i].size();
     }

     // Pre-allocate the exact RAM required
     head.resize(total_nodes + 1, 0);
     edges.resize(total_edges); 

     // Pass 2: Flatten the 2D adjacency list into the 1D CSR arrays
     uint32_t edge_idx = 0;
     for (uint32_t i = 0; i < total_nodes; ++i) 
     {
         head[i] = edge_idx;
         for (const auto& exp_edge : graph[i]) 
         {
             // 1. Map the destination property correctly
             edges[edge_idx].to = exp_edge.destNode;
             
             // 2. FUSE the turn penalty and the physical road traversal weight
             edges[edge_idx].weight = exp_edge.turnWeight + nodes[exp_edge.destNode].wt; 
             
             edge_idx++;
         }
     }
     // Cap the head array for the final node
     head[total_nodes] = edge_idx; 

     std::cout << "Successfully built Global Routing CSR with " << total_edges << " edges.\n";
}



void loadNodeCellMapping(std::string&fileName,uint32_t&totalNodes,std::vector<uint32_t>&node_cell_mapping)
{
     std::ifstream file(fileName,std::ios::binary);

     if(!file)
     {
         std::cerr<<"Cannot be opened\n";
         exit(1);
     }
     //read the total number of nodes
     file.read(reinterpret_cast<char*>(&totalNodes),sizeof(uint32_t));
     
     //resize the vector to allocate the ram before reading
     node_cell_mapping.resize(totalNodes);

     //read the node cell mapping vector
     file.read(reinterpret_cast<char*>(node_cell_mapping.data()),node_cell_mapping.size()*sizeof(uint32_t));

     std::cout<<"Node cell mapping array loaded\n";
}


void load_Original_Graph_From_Binary(const std::string& filename, 
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

int main()
{
     
     return 0;  
}