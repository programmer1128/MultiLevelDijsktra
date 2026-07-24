#include<bits/stdc++.h>
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

std::vector<uint32_t> source_targets;
std::vector<bool> is_sink;

//these are the virtual source nodes and the virtual sink nodes that will be attached to the
//entire graph of india to apply the max flow/min cut partition logic of inertial flow and 
//dinic algorithm
uint32_t S_NODE; //total_nodes
uint32_t T_NODE; //total_nodes + 1

// Global tracking for the final partition results
std::vector<uint32_t> node_cell_mapping;
uint32_t global_cell_id = 0;



//finds the maxflow through the graph with the edge wts as 1. this allows that if the capacity
//is becoming zero, a node is reachable from the source but the next node is not then we can be
//sure that edge will be partitioned and that edge will act as the boundary edge
int maxFlow(uint32_t u, int pushedFlow, uint32_t total_nodes, 
            const std::vector<uint32_t>& head, std::vector<FlowEdge>& edges, 
            std::vector<int>& levels, std::vector<uint32_t>& deadEnds) 
{
     if (u==T_NODE||pushedFlow==0) 
     {
         return pushedFlow;
     }
     //the S_NODE is a dynamic source node that is not in our graph so we cannot run a normal
     //dfs on it. instead we have to figure out the select nodes as per our projections and then
     //from this virtual source node we have to apply the DFS to reach those nodes
     if(u==S_NODE) 
     {
         for(uint32_t&i=deadEnds[S_NODE];i<source_targets.size();i++) 
         {
             uint32_t v = source_targets[i];
             if (levels[v]==levels[u]+1) 
             {
                 int flow=maxFlow(v, pushedFlow, total_nodes, head, edges, levels, deadEnds);
                 if(flow>0) 
                 {
                     return flow;
                 }
             }
         }
         return 0;
     }
     //if the node is a real node of the road network graph then it is connected to the entire graph
     //and we can apply dfs on it
     for (uint32_t&i=deadEnds[u];i<head[u+1];i++) 
     {
         uint32_t v=edges[i].to;
         int residual_cap=edges[i].cap-edges[i].flow;

         if(levels[v]==levels[u]+1&&residual_cap>0) 
         {
             int allowedFlow = std::min(pushedFlow, residual_cap);
             int flow = maxFlow(v, allowedFlow, total_nodes, head, edges, levels, deadEnds);
             if(flow>0) 
             {
                 //in normal problems we update the capacity matrix here before the backtracking phase
                 //but since the road network graph contains more than 24M*24M nodes it will cause out
                 //of bound error and hence we need to update the flow in the struct itself
                 edges[i].flow+=flow;
                 edges[edges[i].rev_idx].flow-=flow; // Update reverse edge instantly
                 return flow;
             }
         }
     }

     if(is_sink[u]) 
     {
         //since Virtual Sink has infinite capacity we just check if it isthe next level
         if (levels[T_NODE]==levels[u]+1) 
         {
             int flow = maxFlow(T_NODE, pushedFlow, total_nodes, head, edges, levels, deadEnds);
             if (flow > 0) 
             {
                 return flow;
             }
         }
     }
     return 0;
}

bool buildLevelGraph(uint32_t total_nodes, const std::vector<uint32_t>& head, 
                     const std::vector<FlowEdge>& edges, std::vector<int>& levels) 
{
     // Reset levels to -1
     std::fill(levels.begin(), levels.end(), -1);
    
     std::queue<uint32_t> queue;
     //we are tracing the levels via the node ids
     levels[S_NODE] = 0;
     queue.push(S_NODE);
    
     while (!queue.empty()) 
     {
         uint32_t u=queue.front();
         queue.pop();
        
         if(u==S_NODE) 
         {
             for(uint32_t v:source_targets) 
             {
                 //this node has not beeen seen prev. so we upadte level to level+1 of parent
                 if(levels[v]==-1) 
                 {
                     levels[v]=levels[u]+1;
                     queue.push(v);
                 }
             }
             continue;
         }

         if (u == T_NODE) 
         {
             continue; 
         }

         //this section is for the real road network expanded graph in CSR format
         for (uint32_t i=head[u];i<head[u+1];i++) 
         {
             uint32_t v = edges[i].to;
             //only traverse if residual capacity>0
             //in normal DSA or CP problems we use a capacity matrix but here we cannot use that 
             //as the road network contains more than 24million nodes so 24million*24million nodes
             //will cause out of memory error 
             if (edges[i].cap-edges[i].flow>0&&levels[v]==-1) 
             {
                 levels[v] = levels[u] + 1;
                 queue.push(v);
             }
         }
         // If this node is pinned to the sink, test if we can reach T_NODE
         if(is_sink[u]&&levels[T_NODE]==-1) 
         {
             levels[T_NODE]=levels[u]+1;
             queue.push(T_NODE);
         }
     }
     // If we reached the sink, return true
     return levels[T_NODE] != -1;
}

std::vector<bool> extractPartitionA(uint32_t total_nodes, 
                                    const std::vector<uint32_t>& head, 
                                    const std::vector<FlowEdge>& edges) 
{
     // A contiguous block of memory is significantly faster than an unordered_map which we have to
     //change from the normal DSA problem code, as hashing and pointer storage increase memory and
     //time complexity. since the graph is in CSR format we can use contiguous arrays for reachable
     //node
     std::vector<bool> reachable(total_nodes, false);
     std::queue<uint32_t> queue;

     //seed the BFS with the pinned source nodes these are the first x% of the nodes from the projection
     //axes sorting that are treated as the real source nodes connected to prior max flow of the 
     //virtual source nodes
     for(uint32_t start_node:source_targets) 
     {
         if (!reachable[start_node]) 
         {
             reachable[start_node]=true;
             queue.push(start_node);
         }
     }

     //Standard BFS through the CSR graph using residual capacities
     while (!queue.empty()) 
     {
         uint32_t u = queue.front();
         queue.pop();
         // Iterate through all outgoing edges from u using the CSR head array
         for (uint32_t i=head[u];i<head[u+1];i++) 
         {
             uint32_t v = edges[i].to;
             // Only traverse edges that have residual capacity remaining
             if (edges[i].cap - edges[i].flow > 0 && !reachable[v]) 
             {
                 reachable[v] = true;
                 queue.push(v);
             }
         }
     }
     // Return the boolean array representing Partition A
     return reachable;
}


int runDinic(uint32_t total_nodes,const std::vector<uint32_t>& head, 
                            std::vector<FlowEdge>& edges)
{
     //in dinic algorithm we need to figure out the level graph first. in the level graph, we have
     //to take original edges only and those edges should have capacity more than 0. for such a 
     //level graph, we continue to find the augmented paths. when we can no longer find augmented paths
     //for a level graph, we stop. then we again make a level graph based on the updated capacities
     //we keep doing this till we can no longer reach the node sink
     int sum = 0;
     while(true)
     {
         // FIX 1: Size the levels array for physical nodes + 2 virtual nodes
         std::vector<int> levels(total_nodes + 2, -1);
        
         //Copy the 'head' array so deadEnds starts at the correct CSR index
         std::vector<uint32_t> deadEnds = head;
         // Ensure it is large enough for the virtual nodes (initialized to 0 for S_NODE)
         deadEnds.resize(total_nodes + 2, 0); 

         deadEnds[total_nodes]=0;     // S_NODE must start at 0
         deadEnds[total_nodes+1]=0;

         if(!buildLevelGraph(total_nodes, head, edges, levels))
         {
             break;
         }

         while(true)
         {
            //Start maxFlow from S_NODE (which is total_nodes)
             int flow = maxFlow(total_nodes, INT_MAX, total_nodes, head, edges, levels, deadEnds);
             if(flow == 0)
             {
                 break;
             }
             sum += flow;
         }
     }
     //Return the cut size to the evaluator
     return sum;
}



CutResult findBestPartition(const std::vector<uint32_t>& current_cell_nodes,
                            const std::vector<std::vector<ProjectedNode>>& projections,
                            uint32_t total_nodes, 
                            const std::vector<uint32_t>& head, 
                            std::vector<FlowEdge>& edges,
                            const std::vector<std::vector<double>>& p_value_lookup) 
{
     std::vector<double> margins = {0.1, 0.2, 0.3};
     CutResult best_result;
     best_result.score = std::numeric_limits<double>::max();

     //iterate through the 4 axes
     for (int axis = 0; axis < 4; ++axis) 
     {
         std::cout << "  [Axis " << axis << "] Sorting 24M nodes...\n";

         std::vector<uint32_t> sorted_cell_nodes = current_cell_nodes;
        
         // Sort based on physical geometry using the O(1) lookup table
         std::sort(sorted_cell_nodes.begin(), sorted_cell_nodes.end(), 
             [&](uint32_t a, uint32_t b) 
             {
                 return p_value_lookup[axis][a] < p_value_lookup[axis][b];
             });

         // 2. Inner Loop: Iterate through the margins
         for (double alpha : margins) 
         {
             std::cout << "    -> Testing margin " << (alpha * 100) << "%... " << std::flush;
             source_targets.clear();
             std::fill(is_sink.begin(), is_sink.end(), false);
            
             for (auto& edge : edges) 
             {
                 edge.flow = 0;
             }

             int k = static_cast<int>(sorted_cell_nodes.size() * alpha);

             // Pin the extremes of the current subgraph
             for (int i = 0; i < k; ++i) 
             {
                 source_targets.push_back(sorted_cell_nodes[i]);
             }
             for (int i = sorted_cell_nodes.size() - k; i < sorted_cell_nodes.size(); ++i) 
             {
                 is_sink[sorted_cell_nodes[i]] = true;
             }

             // Execute Dinic's Algorithm 
             // (Assuming runDinic and extractPartitionA are implemented as discussed)
             int cut_size = runDinic(total_nodes, head, edges);
             std::cout << "Cut Size: " << cut_size << " edges severed.\n";
             std::vector<bool> in_partition_A = extractPartitionA(total_nodes, head, edges);

             std::vector<uint32_t> part_A, part_B;
             for (uint32_t node : current_cell_nodes) 
             {
                 if (in_partition_A[node]) 
                 {
                     part_A.push_back(node);
                 }
                 else
                 {
                     part_B.push_back(node);
                 } 
             }

             int balance = std::min(part_A.size(), part_B.size());
             if (balance == 0)
             {
                 continue;
             } 

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
     return best_result;
}


void recursiveBisection(const std::vector<uint32_t>& cell_nodes, int level,
                        const std::vector<std::vector<ProjectedNode>>& projections,
                        uint32_t total_nodes, const std::vector<uint32_t>& head, 
                        std::vector<FlowEdge>& edges,
                        const std::vector<std::vector<double>>& p_value_lookup) 
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
 
     // Pass all arguments matching your updated signature
     CutResult best_cut = findBestPartition(cell_nodes, projections, total_nodes, head, edges, p_value_lookup);

     // Recurse on the two new sub-cells, passing the full signature down
     recursiveBisection(best_cut.partition_A, level + 1, projections, total_nodes, head, edges, p_value_lookup);
     recursiveBisection(best_cut.partition_B, level + 1, projections, total_nodes, head, edges, p_value_lookup);
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

     //Load the data
     loadCSRBinary(base_path + "csr_format.bin", num_nodes, head, edges);
     loadProjectionsBinary(base_path + "projections_format.bin", sorted_projections);

     //Initialize dynamic structures for Dinic's
     S_NODE = num_nodes; 
     T_NODE = num_nodes + 1;
     is_sink.resize(num_nodes + 2, false);
    
     // Resize the global mapping array
     node_cell_mapping.resize(num_nodes, 0);

     //Build the O(1) Projection Lookup Table
     std::cout << "Building O(1) projection lookup table...\n";
     std::vector<std::vector<double>> p_value_lookup(4, std::vector<double>(num_nodes));
     for (int axis = 0; axis < 4; ++axis) 
     {
         for (const auto& proj_node : sorted_projections[axis]) 
         {
             p_value_lookup[axis][proj_node.id] = proj_node.projectedValue;
         }
     } 

     //Set up the initial "master cell" containing all nodes
     std::vector<uint32_t> initial_cell(num_nodes);
     for (uint32_t i = 0; i < num_nodes; ++i) 
     {
         initial_cell[i] = i;
     }

     //Trigger the engine
     std::cout << "\n--- Starting Inertial Flow Recursive Bisection ---\n";
     recursiveBisection(initial_cell, 0,sorted_projections,num_nodes,head,edges,p_value_lookup);

     //Save the final topology
     savePartitionMapping(base_path + "node_cell_mapping.bin", num_nodes);

    return 0;
}