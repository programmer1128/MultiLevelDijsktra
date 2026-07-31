#include<bits/stdc++.h>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"
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