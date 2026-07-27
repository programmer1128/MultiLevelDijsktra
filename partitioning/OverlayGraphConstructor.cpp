#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <limits>
#include <chrono>
#include <string>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

const uint32_t INF = std::numeric_limits<uint32_t>::max();


std::vector<CellBoundaries> identifyBoundaries(uint32_t total_ebg_nodes, uint32_t total_cells,
                                               const std::vector<uint16_t>& cell_ids,
                                               const std::vector<uint32_t>& routing_head,
                                               const std::vector<RoutingEdge>& routing_edges) 
{
     std::cout << "Identifying cell boundary nodes\n";
     std::vector<CellBoundaries> cell_boundaries(total_cells);
    
     //the original graph network that we have is a expanded road network. so the roads are
     //nodes and the edges are dir to a diff road.so we need to treat the nodes as the roads which 
     //are coming in and out of a cell and then apply dijsktra on the localised graph of that cell
     std::vector<bool> is_entry_node(total_ebg_nodes, false);
     std::vector<bool> is_exit_node(total_ebg_nodes, false);

     for (uint32_t u = 0; u < total_ebg_nodes; ++u) 
     {
         uint16_t u_cell = cell_ids[u];
         for (uint32_t e=routing_head[u];e<routing_head[u+1];++e) 
         {
             uint32_t v = routing_edges[e].to;
             uint16_t v_cell = cell_ids[v];

             if (u_cell!=v_cell) 
             {
                 // u leaves its cell -> Exit Node
                 if(!is_exit_node[u]) 
                 {
                     is_exit_node[u] = true;
                     cell_boundaries[u_cell].exit_nodes.push_back(u);
                 }
                 //v enters a new cell -> Entry Node
                 if (!is_entry_node[v])
                 {
                     is_entry_node[v] = true;
                     cell_boundaries[v_cell].entry_nodes.push_back(v);
                 }
             }
         }
     }

     uint64_t total_boundaries = 0;
     for (const auto& cb : cell_boundaries) 
     {
         total_boundaries += cb.entry_nodes.size() + cb.exit_nodes.size();
     }
     std::cout << "Boundary discovery complete. Found " << total_boundaries << " global boundary points.\n";
    
     return cell_boundaries;
}

void dijkstra_Overlay_Graph(const std::vector<uint32_t>& routing_head, 
     const std::vector<RoutingEdge>& routing_edges
     ,const std::vector<uint16_t>&cell_ids, const std::vector<uint32_t>&exit_nodes,
     std::vector<ShortcutEdge>& global_overlay_adj_bucket
     ,uint32_t&source,uint16_t&curr_cell_id)
{
     auto cmp=[&](std::pair<uint32_t,uint32_t>&node1,std::pair<int,int>&node2)->bool
     {
         return node1.second>node2.second;
     };

     //declare the min heap for the dijkstra
     std::priority_queue<std::pair<uint32_t,uint32_t>,std::vector<std::pair<uint32_t,uint32_t>>,decltype(cmp)>queue(cmp);

     std::unordered_map<uint32_t, uint32_t> distanceTable;
     std::unordered_map<uint32_t, uint32_t> parent;
     distanceTable[source]=0;
     parent[source]=source;

     //offer up the source node
     queue.push({source,0});

     while(!queue.empty())
     {
         std::pair<int,int>node=queue.top();
         uint32_t u=node.first;
         uint32_t currDist=node.second;
         queue.pop();

         if(currDist>distanceTable[u])
         {
             continue;
         }
         //now the edges for the node will be between the heads[u] and heads[u+1]
         //index of the routing edges array
         uint32_t start=routing_head[u];
         uint32_t end=routing_head[u+1];

         for(uint32_t index=start;index<end;index++)
         {
             uint32_t v=routing_edges[index].to;
             if(cell_ids[v]!=curr_cell_id)
             {
                 continue;
             }
             uint32_t wt=routing_edges[index].weight;

             uint32_t dist=wt+distanceTable[u];

             if(distanceTable.find(v)==distanceTable.end()||dist<distanceTable[v])
             {
                 //update the dist table
                 distanceTable[v]=dist;
                 //mark the path
                 parent[v]=u;
                 //push the node into queue.
                 queue.push(std::make_pair(v,dist));
             }
         }
     }

        
}



//localised dijkstra to find the overlay graph using shortcuts in the same cell
void find_Overlay_Graphs(std::vector<CellBoundaries>&cellBoundaries,std::vector<uint32_t>& routing_head
     ,std::vector<RoutingEdge>& routing_edges,const std::vector<uint16_t>&cell_ids
     )
{
     //for the cell boundaries vector, the index is the cell id and every index contains
     //the cell with the incoming and the outgoing expanded nodes and for every one of them
     //we have to apply the localised dijkstra and find the overlay graphs
     for(uint16_t cell_id=0;cell_id<cellBoundaries.size();cell_id++)
     {
         for(uint32_t u:cellBoundaries[cell_id].entry_nodes)
         {
             dijkstra_Overlay_Graph(routing_head,routing_edges,cell_ids,cellBoundaries[cell_id].exit_nodes
             ,global_overlay_adj_bucket,u,cell_id);
         }
     }
}



//load the CSR graph of the real graph 
void loadRoutingCSR(const std::string& filepath, uint32_t& total_ebg_nodes, 
                    std::vector<uint32_t>& routing_head, std::vector<RoutingEdge>& routing_edges) 
{
     std::cout << "Loading routing_csr.bin\n";
     std::ifstream route_in(filepath, std::ios::binary);
     if (!route_in) 
     {
         std::cerr << "Failed to open " << filepath << "\n";
         exit(1);
     }
    
     uint32_t total_routing_edges=0;
     route_in.read(reinterpret_cast<char*>(&total_ebg_nodes), sizeof(total_ebg_nodes));
     route_in.read(reinterpret_cast<char*>(&total_routing_edges), sizeof(total_routing_edges));

     routing_head.resize(total_ebg_nodes + 1);
     routing_edges.resize(total_routing_edges);
    
     route_in.read(reinterpret_cast<char*>(routing_head.data()), routing_head.size() * sizeof(uint32_t));
     route_in.read(reinterpret_cast<char*>(routing_edges.data()), routing_edges.size() * sizeof(RoutingEdge));
     route_in.close();
    
     std::cout << "Loaded " << total_ebg_nodes << " nodes, " << total_routing_edges << " edges.\n";
}


//loading the assigned cell ids of the nodes 
uint32_t loadCellIDs(const std::string& filepath, uint32_t total_ebg_nodes, std::vector<uint16_t>& cell_ids) 
{
     std::cout << "Loading cell assignments from metadata.bin...\n";
     std::ifstream meta_in(filepath, std::ios::binary);
     if (!meta_in) 
     {
         std::cerr << "Failed to open " << filepath << "\n";
         exit(1);
     }
    
     size_t check_nodes = 0;
     meta_in.read(reinterpret_cast<char*>(&check_nodes), sizeof(check_nodes));
    
     cell_ids.resize(total_ebg_nodes);
     uint32_t max_cell_id = 0;

     // Stream through metadata to extract just the cell_id array
     for (size_t i = 0; i < total_ebg_nodes; ++i) 
     {
         uint32_t id; uint64_t orig, s_osm, e_osm; uint32_t wt, dist;
         uint8_t r_type; uint32_t n_id, g_off, g_sz; bool o_way;
         
         //read in stack temporary variables just to move the file pointer ahead
         meta_in.read(reinterpret_cast<char*>(&id), sizeof(id));
         meta_in.read(reinterpret_cast<char*>(&orig), sizeof(orig));
         meta_in.read(reinterpret_cast<char*>(&s_osm), sizeof(s_osm));
         meta_in.read(reinterpret_cast<char*>(&e_osm), sizeof(e_osm));
         meta_in.read(reinterpret_cast<char*>(&wt), sizeof(wt));
         meta_in.read(reinterpret_cast<char*>(&dist), sizeof(dist));
        
         //read the req cell id from the file into the cell id array which we will use
         //the index is the node_id itself and for every node_id we will have the cell id
         //as cell_id[node_id]
         meta_in.read(reinterpret_cast<char*>(&cell_ids[i]), sizeof(cell_ids[i]));
         if (cell_ids[i] > max_cell_id) 
         {
             max_cell_id = cell_ids[i];
         }

         meta_in.read(reinterpret_cast<char*>(&r_type), sizeof(r_type));
         meta_in.read(reinterpret_cast<char*>(&n_id), sizeof(n_id));
         meta_in.read(reinterpret_cast<char*>(&o_way), sizeof(o_way));
         meta_in.read(reinterpret_cast<char*>(&g_off), sizeof(g_off));
         meta_in.read(reinterpret_cast<char*>(&g_sz), sizeof(g_sz));
     }
     meta_in.close();
    
     uint32_t total_cells = max_cell_id + 1;
     std::cout << "Total cells recognized: " << total_cells << "\n";
     return total_cells;
}


int main() 
{
     std::string base_path = "/home/aritra/MultiLevelDijsktra/parsing/";

     //data Structures
     uint32_t total_ebg_nodes = 0;
     std::vector<uint32_t> routing_head;
     std::vector<RoutingEdge> routing_edges;
     std::vector<uint16_t> cell_ids;

     //load Data
     loadRoutingCSR(base_path + "routing_csr.bin", total_ebg_nodes, routing_head, routing_edges);
     uint32_t total_cells = loadCellIDs(base_path + "metadata.bin", total_ebg_nodes, cell_ids);

     //process Topology
     std::vector<CellBoundaries> cell_boundaries=identifyBoundaries(total_ebg_nodes, total_cells, 
                 cell_ids, routing_head, routing_edges);
    
     std::vector<std::vector<ShortcutEdge>> overlay_adj(total_ebg_nodes);

     // overlay CSR Storage (Populated after Dijkstra finishes)
     std::vector<uint32_t> overlay_head(total_ebg_nodes + 1, 0);
     std::vector<ShortcutEdge> overlay_edges;


     return 0;
}