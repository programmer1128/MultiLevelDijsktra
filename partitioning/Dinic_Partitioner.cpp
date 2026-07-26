#include<bits/stdc++.h>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"
#include <chrono>



// Global tracking for the final partition results
std::vector<uint32_t> node_cell_mapping;
uint32_t global_cell_id = 0;

// Logic for building a local graph
LocalGraph buildInducedSubgraph(
    const std::vector<uint32_t>& cell_nodes,
    const std::vector<uint32_t>& global_head,
    const std::vector<FlowEdge>& global_edges,
    const std::vector<uint32_t>& local_source_targets,
    const std::vector<bool>& local_is_sink,
    const std::vector<int>& global_to_local            
) 
{
     uint32_t k = cell_nodes.size();
     uint32_t local_source = k;
     uint32_t local_sink = k + 1;
 
     LocalGraph lg;
     lg.num_nodes = k + 2;
     lg.local_source = local_source;
     lg.local_sink = local_sink;

     std::vector<std::vector<FlowEdge>> local_adj(lg.num_nodes);
     const int INF_CAP = INT_MAX / 2; //Infinite capacity for virtual edges

     // copying internal cell edges of the real graph that is physically present in the road network
     //graph
     for (uint32_t i = 0; i < k; ++i) 
     {
         uint32_t u_global = cell_nodes[i];
         uint32_t u_local = i;

         for (uint32_t e = global_head[u_global]; e < global_head[u_global + 1]; ++e) 
         {
             uint32_t v_global = global_edges[e].to;
             int v_local = global_to_local[v_global];

             if (v_local != -1) 
             {
                 local_adj[u_local].push_back({(uint32_t)v_local, global_edges[e].cap, 0, (uint32_t)local_adj[v_local].size()});
                 local_adj[v_local].push_back({u_local, 0, 0, (uint32_t)(local_adj[u_local].size() - 1)});
             }
         }
     }

     //connecting virtual source to selected real source nodes. inertial flow selects x% of the 
     //nodes on extreme left side as the source nodes. these source nodes have to be connected to
     //the virtual source node with infinite capacity to partition the graph
     for (uint32_t src_local : local_source_targets)
     {
         local_adj[local_source].push_back({src_local, INF_CAP, 0, (uint32_t)local_adj[src_local].size()});
         local_adj[src_local].push_back({local_source, 0, 0, (uint32_t)(local_adj[local_source].size() - 1)});
     }

     //connect selected local sinks to virtual Sink for infinite capacity
     for (uint32_t i = 0; i < k; ++i) 
     {
         if (local_is_sink[i]) // i is perfectly matched to the local ID
         {
             local_adj[i].push_back({local_sink, INF_CAP, 0, (uint32_t)local_adj[local_sink].size()});
             local_adj[local_sink].push_back({i, 0, 0, (uint32_t)(local_adj[i].size() - 1)});
         }
     }

     //Flatten local_adj into a clean local CSR format
     lg.head.resize(lg.num_nodes + 1, 0);
     uint32_t edge_count = 0;
     for (uint32_t i = 0; i < lg.num_nodes; ++i) 
     {
         lg.head[i] = edge_count;
         edge_count += local_adj[i].size();
     }
     lg.head[lg.num_nodes] = edge_count;

     lg.edges.reserve(edge_count);
     for (uint32_t i = 0; i < lg.num_nodes; ++i) 
     {
         for (const auto& edge : local_adj[i]) 
         {
             lg.edges.push_back(edge);
         }
     }

    return lg;
}


int maxFlow(uint32_t u, int pushedFlow, uint32_t S_NODE, uint32_t T_NODE, 
            const std::vector<uint32_t>& source_targets, const std::vector<bool>& is_sink,
            const std::vector<uint32_t>& head, std::vector<FlowEdge>& edges, 
            std::vector<int>& levels, std::vector<uint32_t>& deadEnds)
{
     if (u == T_NODE || pushedFlow == 0) return pushedFlow;
     
     if(u == S_NODE) 
     {
         for(uint32_t& i = deadEnds[S_NODE]; i < head[S_NODE + 1]; i++) // Use CSR directly for S_NODE
         {
             uint32_t v = edges[i].to;
             int residual_cap = edges[i].cap - edges[i].flow;
             if (levels[v] == levels[u] + 1 && residual_cap > 0) 
             {
                 int flow = maxFlow(v, std::min(pushedFlow, residual_cap), S_NODE, T_NODE, source_targets, is_sink, head, edges, levels, deadEnds);
                 if(flow > 0) 
                 {
                     edges[i].flow += flow;
                     edges[edges[i].rev_idx].flow -= flow;
                     return flow;
                 }
             }
         }
         return 0;
     }

     for (uint32_t& i = deadEnds[u]; i < head[u + 1]; i++) 
     {
         uint32_t v = edges[i].to;
         int residual_cap = edges[i].cap - edges[i].flow;

         if(levels[v] == levels[u] + 1 && residual_cap > 0) 
         {
             int flow = maxFlow(v, std::min(pushedFlow, residual_cap), S_NODE, T_NODE, source_targets, is_sink, head, edges, levels, deadEnds);
             if(flow > 0) 
             {
                 edges[i].flow += flow;
                 edges[edges[i].rev_idx].flow -= flow; 
                 return flow;
             }
         }
     }
     return 0;
}

//level graph used for the dinic flow. the graph takes all the edges with the capacity of
//more > 0 and which connect a level to it's next level. which is why we use the BFS to find
//out edges level by level
bool buildLevelGraph(uint32_t num_nodes, const std::vector<uint32_t>& head, 
                     const std::vector<FlowEdge>& edges, std::vector<int>& levels,
                     uint32_t S_NODE, uint32_t T_NODE) 
{
     //initialisation of the levels
     std::fill(levels.begin(), levels.end(), -1);
     std::queue<uint32_t> queue;
     
     levels[S_NODE] = 0;
     queue.push(S_NODE);
    
     while (!queue.empty()) 
     {
         uint32_t u = queue.front();
         queue.pop();

         if (u == T_NODE) 
         {
             continue;
         }
         for (uint32_t i = head[u]; i < head[u + 1]; i++) 
         {
             uint32_t v = edges[i].to;
             if (edges[i].cap - edges[i].flow > 0 && levels[v] == -1) 
             {
                 levels[v] = levels[u] + 1;
                 queue.push(v);
             }
         }
     }
     return levels[T_NODE] != -1;
}

//after the dinic flow is done we extract the edges where the nodes are not reachable from
//the sources that where selected for the inertial flow
std::vector<bool> extractPartitionA(uint32_t num_nodes, 
                                    const std::vector<uint32_t>& head, 
                                    const std::vector<FlowEdge>& edges,
                                    const std::vector<uint32_t>& local_source_targets) 
{
     std::vector<bool> reachable(num_nodes, false);
     std::queue<uint32_t> queue;

     for(uint32_t start_node : local_source_targets) 
     {
         if (!reachable[start_node]) 
         {
             reachable[start_node] = true;
             queue.push(start_node);
         }
     }
     //BFS to explore the nodes level by level and accordingly exlcude the nodes not
     //reachable
     while (!queue.empty()) 
     {
         uint32_t u = queue.front();
         queue.pop();
         
         for (uint32_t i = head[u]; i < head[u + 1]; i++) 
         {
             uint32_t v = edges[i].to;
             if (edges[i].cap - edges[i].flow > 0 && !reachable[v]) 
             {
                 reachable[v] = true;
                 queue.push(v);
             }
         }
     }
     return reachable;
}


int runDinic(uint32_t num_nodes, const std::vector<uint32_t>& head, 
             std::vector<FlowEdge>& edges, uint32_t S_NODE, uint32_t T_NODE,
             const std::vector<bool>& is_sink, const std::vector<uint32_t>& source_targets)
{
     int sum = 0;
     while(true)
     {
         std::vector<int> levels(num_nodes, -1);
         std::vector<uint32_t> deadEnds = head; //sized to num_nodes + 1 via local CSR

         if(!buildLevelGraph(num_nodes, head, edges, levels, S_NODE, T_NODE)) break;

         while(true)
         {
             int flow = maxFlow(S_NODE, INT_MAX, S_NODE, T_NODE, source_targets, is_sink, head, edges, levels, deadEnds);
             if(flow == 0) break;
             sum += flow;
         }
     }
     return sum;
}


CutResult findBestPartition(const std::vector<uint32_t>& current_cell_nodes,
                            const std::vector<std::vector<ProjectedNode>>& projections,
                            const std::vector<uint32_t>& head, 
                            const std::vector<FlowEdge>& global_edges, 
                            const std::vector<std::vector<double>>& p_value_lookup,
                            std::vector<int>& global_to_local) 
{
     std::vector<double> margins = {0.1, 0.2, 0.3};
     CutResult best_result;
     best_result.score = std::numeric_limits<double>::max();
     
     uint32_t k_nodes = current_cell_nodes.size();

     auto start_time = std::chrono::high_resolution_clock::now();
     for (int axis = 0; axis < 4; ++axis) 
     {
         std::vector<uint32_t> sorted_cell_nodes = current_cell_nodes;
        
         std::sort(sorted_cell_nodes.begin(), sorted_cell_nodes.end(), 
             [&](uint32_t a, uint32_t b) 
             {
                 return p_value_lookup[axis][a] < p_value_lookup[axis][b];
             });

         for (double alpha : margins) 
         {             
             for (uint32_t i = 0; i < k_nodes; ++i) 
             {
                 global_to_local[current_cell_nodes[i]] = i;
             }

             std::vector<uint32_t> local_source_targets;
             std::vector<bool> local_is_sink(k_nodes + 2, false); 

             int k_margin = static_cast<int>(k_nodes * alpha);

             for (int i = 0; i < k_margin; ++i) 
             {
                 int local_id = global_to_local[sorted_cell_nodes[i]];
                 if (local_id != -1) local_source_targets.push_back(local_id);
             }
             for (int i = k_nodes - k_margin; i < k_nodes; ++i) 
             {
                 int local_id = global_to_local[sorted_cell_nodes[i]];
                 if (local_id != -1) local_is_sink[local_id] = true;
             }

             LocalGraph lg = buildInducedSubgraph(current_cell_nodes, head, global_edges, 
                                                  local_source_targets, local_is_sink, global_to_local);

             int cut_size = runDinic(lg.num_nodes, lg.head, lg.edges, lg.local_source, lg.local_sink, local_is_sink, local_source_targets);
             
             std::cout << "Cut Size: " << cut_size << " edges severed.\n";
             std::vector<bool> in_partition_A = extractPartitionA(lg.num_nodes, lg.head, lg.edges, local_source_targets);

             std::vector<uint32_t> part_A, part_B;
             for (uint32_t i = 0; i < k_nodes; ++i) 
             {
                 uint32_t global_node = current_cell_nodes[i];
                 if (in_partition_A[i]) part_A.push_back(global_node);
                 else part_B.push_back(global_node);
             }

             for (uint32_t i = 0; i < k_nodes; ++i) 
             {
                 global_to_local[current_cell_nodes[i]] = -1;
             }

             int balance = std::min(part_A.size(), part_B.size());
             if (balance == 0) continue; 

             double current_score = static_cast<double>(cut_size) / balance;

             if (current_score < best_result.score) 
             {
                 best_result.score = current_score;
                 best_result.cut_size = cut_size;
                 best_result.partition_A = part_A;
                 best_result.partition_B = part_B;
             }
         }
     }
     auto end_time = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
     std::cout << "  -> Cell bisection complete in " << duration << " ms.\n";
     return best_result;
}

//in standard graph the dinic is applied only on the main graph. but here we have a huge road network
//graph. every dinic will result in a partition.so we need to partition those individual graphs too and
//finally we will have cells of nodes within 5000
void recursiveBisection(const std::vector<uint32_t>& cell_nodes, int level,
                        const std::vector<std::vector<ProjectedNode>>& projections,
                        const std::vector<uint32_t>& head, 
                        const std::vector<FlowEdge>& edges,
                        const std::vector<std::vector<double>>& p_value_lookup,
                        std::vector<int>& global_to_local) 
{
     const int MAX_CELL_SIZE = 5000;

     if (cell_nodes.size() <= MAX_CELL_SIZE) 
     {
         for (uint32_t node : cell_nodes) 
         {
             node_cell_mapping[node] = global_cell_id;
         }
         global_cell_id++;
         return;
     }

     std::cout << "[Level " << level << "] Bisecting cell of size " << cell_nodes.size() << "\n";
 
     CutResult best_cut = findBestPartition(cell_nodes, projections, head, edges, p_value_lookup, global_to_local);

     recursiveBisection(best_cut.partition_A, level + 1, projections, head, edges, p_value_lookup, global_to_local);
     recursiveBisection(best_cut.partition_B, level + 1, projections, head, edges, p_value_lookup, global_to_local);
}



void savePartitionMapping(const std::string& filename, uint32_t total_nodes) 
{
     std::ofstream out_file(filename, std::ios::binary);
     out_file.write(reinterpret_cast<const char*>(&total_nodes), sizeof(total_nodes));
     out_file.write(reinterpret_cast<const char*>(node_cell_mapping.data()), 
                   total_nodes * sizeof(uint32_t));
     out_file.close();
    
     std::cout << "\nPartitioning Complete!\n";
     std::cout << "Total unique cells created: " << global_cell_id << "\n";
     std::cout << "Mapping saved to: " << filename << "\n";
}



void loadCSRBinary(const std::string& filename, uint32_t& num_nodes, 
                   std::vector<uint32_t>& head, std::vector<FlowEdge>& edges) 
{
     std::ifstream in_file(filename, std::ios::binary);
     if (!in_file) 
     {
         std::cerr << "Failed to open " << filename << "\n";
         exit(1);
     }

     uint32_t total_edges = 0;
     in_file.read(reinterpret_cast<char*>(&num_nodes), sizeof(num_nodes));
     in_file.read(reinterpret_cast<char*>(&total_edges), sizeof(total_edges));

     head.resize(num_nodes + 1);
     edges.resize(total_edges);

     in_file.read(reinterpret_cast<char*>(head.data()), head.size() * sizeof(uint32_t));
     in_file.read(reinterpret_cast<char*>(edges.data()), edges.size() * sizeof(FlowEdge));
    
     in_file.close();
     std::cout << "Loaded CSR: " << num_nodes << " nodes, " << total_edges << " edges.\n";
}

void loadProjectionsBinary(const std::string& filename, 
                           std::vector<std::vector<ProjectedNode>>& sorted_projections) 
{
     std::ifstream in_file(filename, std::ios::binary);
     if (!in_file) 
     {
         std::cerr << "Failed to open " << filename << "\n";
         exit(1);
     }

     uint32_t num_axes, num_nodes;
     in_file.read(reinterpret_cast<char*>(&num_axes), sizeof(num_axes));
     in_file.read(reinterpret_cast<char*>(&num_nodes), sizeof(num_nodes));

     sorted_projections.resize(num_axes);
     for (uint32_t i = 0; i < num_axes; ++i) 
     {
         sorted_projections[i].resize(num_nodes);
         in_file.read(reinterpret_cast<char*>(sorted_projections[i].data()), 
                     num_nodes * sizeof(ProjectedNode));
     }
    
     in_file.close();
     std::cout << "Loaded Projections for " << num_nodes << " nodes across " << num_axes << " axes.\n";
}

int main() 
{
     std::string base_path = "/home/aritra/MultiLevelDijsktra/parsing/";
    
     uint32_t num_nodes = 0;
     std::vector<uint32_t> head;
     std::vector<FlowEdge> edges;
     std::vector<std::vector<ProjectedNode>> sorted_projections;

     loadCSRBinary(base_path + "flow_csr.bin", num_nodes, head, edges);
     loadProjectionsBinary(base_path + "projections_format.bin", sorted_projections);
    
     node_cell_mapping.resize(num_nodes, 0);

     std::cout << "Building O(1) projection lookup table...\n";
     std::vector<std::vector<double>> p_value_lookup(4, std::vector<double>(num_nodes));
     for (int axis = 0; axis < 4; ++axis) 
     {
         for (const auto& proj_node : sorted_projections[axis]) 
         {
             p_value_lookup[axis][proj_node.id] = proj_node.projectedValue;
         }
     } 

     std::vector<uint32_t> initial_cell(num_nodes);
     for (uint32_t i = 0; i < num_nodes; ++i) 
     {
         initial_cell[i] = i;
     }

     std::vector<int> global_to_local(num_nodes, -1);

     std::cout << "\nStarting Localized Inertial Flow Recursive Bisection\n";
     // Note: total_nodes is no longer passed globally!
     recursiveBisection(initial_cell, 0, sorted_projections, head, edges, p_value_lookup, global_to_local);

     savePartitionMapping(base_path + "node_cell_mapping.bin", num_nodes);

     return 0;
}
/*
void savePartitionMapping(const std::string& filename, uint32_t total_nodes) 
{
     std::ofstream out_file(filename, std::ios::binary);
     out_file.write(reinterpret_cast<const char*>(&total_nodes), sizeof(total_nodes));
     out_file.write(reinterpret_cast<const char*>(node_cell_mapping.data()), 
                   total_nodes * sizeof(uint32_t));
     out_file.close();
    
     std::cout << "\nPartitioning Complete!\n";
     std::cout << "Total unique cells created: " << global_cell_id << "\n";
     std::cout << "Mapping saved to: " << filename << "\n";
}



void loadCSRBinary(const std::string& filename, uint32_t& num_nodes, 
                   std::vector<uint32_t>& head, std::vector<FlowEdge>& edges) 
{
     std::ifstream in_file(filename, std::ios::binary);
     if (!in_file) 
     {
         std::cerr << "Failed to open " << filename << "\n";
         exit(1);
     }

     uint32_t total_edges = 0;
     in_file.read(reinterpret_cast<char*>(&num_nodes), sizeof(num_nodes));
     in_file.read(reinterpret_cast<char*>(&total_edges), sizeof(total_edges));

     head.resize(num_nodes + 1);
     edges.resize(total_edges);

     in_file.read(reinterpret_cast<char*>(head.data()), head.size() * sizeof(uint32_t));
     in_file.read(reinterpret_cast<char*>(edges.data()), edges.size() * sizeof(FlowEdge));
    
     in_file.close();
     std::cout << "Loaded CSR: " << num_nodes << " nodes, " << total_edges << " edges.\n";
}

void loadProjectionsBinary(const std::string& filename, 
                           std::vector<std::vector<ProjectedNode>>& sorted_projections) 
{
     std::ifstream in_file(filename, std::ios::binary);
     if (!in_file) 
     {
         std::cerr << "Failed to open " << filename << "\n";
         exit(1);
     }

     uint32_t num_axes, num_nodes;
     in_file.read(reinterpret_cast<char*>(&num_axes), sizeof(num_axes));
     in_file.read(reinterpret_cast<char*>(&num_nodes), sizeof(num_nodes));

     sorted_projections.resize(num_axes);
     for (uint32_t i = 0; i < num_axes; ++i) 
     {
         sorted_projections[i].resize(num_nodes);
         in_file.read(reinterpret_cast<char*>(sorted_projections[i].data()), 
                     num_nodes * sizeof(ProjectedNode));
     }
    
     in_file.close();
     std::cout << "Loaded Projections for " << num_nodes << " nodes across " << num_axes << " axes.\n";
}
*/