#include<bits/stdc++.h>
#include <json/json.h>
#include "plugins/GraphDataPlugin.hpp"
#include "multi_level_dijkstra.h"
#include<chrono>

//alias for the std pair to be used in the priority queue
using SearchState = std::pair<uint32_t,uint32_t>;

const uint32_t INF = std::numeric_limits<uint32_t>::max();

//core navigation engine handling the multilevel dijkstra of the partitioned edge expanded graph
void multi_level_dijsktra(const GraphDataView& data,uint32_t source,uint32_t target,std::vector<uint32_t>&path)
{
     if (source >= data.total_nodes || target >= data.total_nodes) 
     {
         LOG_ERROR<< "[ERROR] KD-Tree returned an out-of-bounds node ID!\n";
         LOG_ERROR<< "Source ID: " << source << " | Target ID: " << target<<"\n";
         return; 
     }
     //distance tables for the forward and the backward search
     std::vector<uint32_t> dist_f(data.total_nodes, INF);
     std::vector<uint32_t> dist_b(data.total_nodes, INF);

     //parent arrays for the path tracing
     std::vector<uint32_t> parent_f(data.total_nodes, INF);
     std::vector<uint32_t> parent_b(data.total_nodes, INF);

     // std::greater naturally sorts std::pair by the .first element (weight) ascendingly 
     std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> pq_f;
     std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> pq_b;

     
     uint32_t best_meeting_node = INF;
     uint32_t shortest_path_weight = INF;

     dist_f[source] = 0;
     pq_f.push({0, source}); // 0 is weight (.first), source is ID (.second)

     dist_b[target] = 0;
     pq_b.push({0, target});

     uint32_t start_cell_id= data.node_metadata[source].cell_id;
     uint32_t target_cell_id= data.node_metadata[target].cell_id;
     //std::cout<<"dijsktra started\n";
     while(!pq_f.empty()&&!pq_b.empty())
     {
         if(pq_f.top().first + pq_b.top().first >= shortest_path_weight)
         {
             break;
         }

         //now for the forward node and the backward node we will use the routing head and routing
         //edges CSR format.
         //the edges of the node lie in the routing edges array from routing_head[u] to < routing_head[u+1]
         //indices so we traverse them and add them to the pq. now if the search is in a state inbetween
         //we have to use the shortcuts.this is when the search has left the target cell and the search cell
         //which denotes that we are in the middle. for those roads, we can use the shorcut edges we have
         //so we only check the local edges when the node popped from the pq are inside the target cell or
         //the source cell


         if(pq_f.top().first <= pq_b.top().first)
         {
             SearchState forward = pq_f.top();
             pq_f.pop();

             uint32_t forwardDist = forward.first;
             uint32_t forwardNode = forward.second;
             uint32_t forwardNodeCell = data.node_metadata[forwardNode].cell_id;

             // Stale entry check (Early out)
             if(forwardDist > dist_f[forwardNode]) 
             { 
                 continue;
             }

             if (dist_b[forwardNode] != INF) 
             {
                 uint32_t total_weight = forwardDist + dist_b[forwardNode];
                 if (total_weight < shortest_path_weight) 
                 {
                     shortest_path_weight = total_weight;
                     best_meeting_node = forwardNode;
                 }
             }

             bool isLocalForward=forwardNodeCell==start_cell_id||forwardNodeCell==target_cell_id;

             //apply the local search if the popped node is in source or target cell
             for(uint32_t u=data.routing_head[forwardNode]; u<data.routing_head[forwardNode+1]; u++)
            {
                uint32_t toNode = data.routing_edges[u].to;
                uint32_t toNodeCell = data.node_metadata[toNode].cell_id;

                //check if the road is inside the local cell or not
                bool isLocal = (forwardNodeCell == start_cell_id || forwardNodeCell == target_cell_id);
                
                //check if it is a boundary cell
                bool isCrossCell = (forwardNodeCell != toNodeCell);

                // Only allow the search to take this physical road if it satisfies one of the rules.
                if(isLocal || isCrossCell)
                {
                    uint32_t wt = data.routing_edges[u].weight;
                    uint32_t currDist = wt + dist_f[forwardNode];

                    if(currDist < dist_f[toNode])
                    {
                        dist_f[toNode] = currDist;
                        parent_f[toNode] = forwardNode;
                        pq_f.push({currDist, toNode});   
                    }
                }
            }

             //overlay edges are used always for the nodes because it might be so we get a direct edge from
             //the boundary node to the target
             for(uint32_t u=data.overlay_head[forwardNode];u<data.overlay_head[forwardNode+1];u++)
             {
                 uint32_t toNode=data.overlay_edges[u].to;
                 uint32_t wt=data.overlay_edges[u].weight;
                 uint32_t currDist=wt+dist_f[forwardNode];

                 if(currDist<dist_f[toNode])
                 {
                     dist_f[toNode]=currDist;
                     parent_f[toNode]=forwardNode;
                     pq_f.push({currDist,toNode});   
                 }
             }
         }
         else
         {
             SearchState backward=pq_b.top();
             pq_b.pop();
             uint32_t backwardDist=backward.first;

             uint32_t backwardNode=backward.second;
             uint32_t backwardNodeCell=data.node_metadata[backwardNode].cell_id;

             if(backwardDist > dist_b[backwardNode]) 
             {
                 continue;
             }
             // Check if backward search collided with forward search
             if (dist_f[backwardNode] != INF) 
             {
                 uint32_t total_weight = backwardDist + dist_f[backwardNode];
                 if (total_weight < shortest_path_weight) 
                 {
                     shortest_path_weight = total_weight;
                     best_meeting_node = backwardNode;
                 }
             }
             //same process is repeated for the backward search for the local nodes and the shortcut edges
             for(uint32_t u=data.reverse_routing_head[backwardNode]; u<data.reverse_routing_head[backwardNode+1]; u++)
             {
                 uint32_t toNode = data.reverse_routing_edges[u].to;
                 uint32_t toNodeCell = data.node_metadata[toNode].cell_id;
 
                 bool isLocal = (backwardNodeCell == start_cell_id || backwardNodeCell == target_cell_id);
                 bool isCrossCell = (backwardNodeCell != toNodeCell);
 
                 if(isLocal || isCrossCell)
                 {
                     uint32_t wt = data.reverse_routing_edges[u].weight;
                     uint32_t currDist = wt + dist_b[backwardNode];
 
                     if(currDist < dist_b[toNode])
                    {
                         dist_b[toNode] = currDist;
                         parent_b[toNode] = backwardNode;
                         pq_b.push({currDist, toNode});   
                     }
                 }
             }
             //overlay edges are used always for the nodes because it might be so we get a direct edge from
             //the boundary node to the target
             for(uint32_t u=data.reverse_overlay_head[backwardNode];u<data.reverse_overlay_head[backwardNode+1];u++)
             {
                 uint32_t toNode=data.reverse_overlay_edges[u].to;
                 uint32_t wt=data.reverse_overlay_edges[u].weight;
                 uint32_t currDist=wt+dist_b[backwardNode];

                 if(currDist<dist_b[toNode])
                 {
                     dist_b[toNode]=currDist;
                     parent_b[toNode]=backwardNode;
                     pq_b.push({currDist,toNode});   
                 }
             }    
         }
     }

     //std::cout<<"dijsktra completed\n";
     if (best_meeting_node == INF) 
     {
         LOG_INFO<< "No path found between source and target.\n";
         return;
     }

     //reconstruct Forward of path
     std::vector<uint32_t> forward_half;
     uint32_t curr_f = best_meeting_node;
     while (curr_f != INF && curr_f != source) 
     {
         forward_half.push_back(curr_f);
         curr_f = parent_f[curr_f];
     }
     forward_half.push_back(source);
    
     //reversing path as we constructed the path by backtracking
     std::reverse(forward_half.begin(), forward_half.end());

     // Add to the final path
     path.insert(path.end(), forward_half.begin(), forward_half.end());

     //Reconstruct Backward Half (Meeting Node to Target)
     uint32_t curr_b = parent_b[best_meeting_node];
     while (curr_b != INF) 
     {
         path.push_back(curr_b);
         curr_b = parent_b[curr_b];
     }

     //std::cout << "Shortest Path Found! Weight: " << shortest_path_weight << " seconds.\n";
     //std::cout << "Total Nodes in Path: " << path.size() << "\n";
}



// Micro-Dijkstra to recover the base nodes of a single shortcut inside a cell
std::vector<uint32_t> localized_shortcut_unpack(uint32_t start_node, uint32_t end_node, const GraphDataView& data) 
{
     uint16_t cell_id = data.node_metadata[start_node].cell_id;
    
     std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> pq;
     std::unordered_map<uint32_t, uint32_t> dist;
     std::unordered_map<uint32_t, uint32_t> parent;
    
     pq.push({0, start_node});
     dist[start_node] = 0;
    
     while (!pq.empty()) 
     {
         SearchState current = pq.top();
         pq.pop();
        
         uint32_t currDist = current.first;
         uint32_t currNode = current.second;
        
         if (currNode == end_node) break; // Reached the end of the shortcut
         if (currDist > dist[currNode]) continue;
        
         // Traverse only base routing edges
         for (uint32_t u = data.routing_head[currNode]; u < data.routing_head[currNode+1]; ++u) 
         {
             uint32_t nextNode = data.routing_edges[u].to;
            
             // STRICT CELL BOUNDARY PRUNING: Only explore roads inside this exact cell
            // if (data.node_cell_mapping[nextNode] != cell_id) continue;
             
             uint32_t nextDist = currDist + data.routing_edges[u].weight;
            
             if (dist.find(nextNode) == dist.end() || nextDist < dist[nextNode]) 
             {
                 dist[nextNode] = nextDist;
                 parent[nextNode] = currNode;
                 pq.push({nextDist, nextNode});
             }
         }
     }
    
     // Trace back the local path safely
     std::vector<uint32_t> sub_path;
     uint32_t curr = end_node;
    
     while (curr != start_node) 
     {
         // SAFETY CHECK: Prevent the unordered_map infinite loop trap
         auto it = parent.find(curr);
         if (it == parent.end()) 
         {
             std::cout << "[WARNING] Unpack missing inner roads for shortcut " << start_node << " -> " << end_node << ". Falling back to straight line.\n";
             sub_path.clear();
             sub_path.push_back(end_node);
             break;
         }
        
         sub_path.push_back(curr);
         curr = it->second; // Safely assign the parent
     }
    
     sub_path.push_back(start_node);
     std::reverse(sub_path.begin(), sub_path.end());
    
     return sub_path;
}

// Converts a mixed abstract path into a purely physical base edge path
std::vector<uint32_t> unpack_abstract_path(const std::vector<uint32_t>& abstract_path, const GraphDataView& data) 
{
    if (abstract_path.empty()) return {};
    
    std::vector<uint32_t> detailed_path;
    detailed_path.push_back(abstract_path.front());
    
    for (size_t i = 0; i < abstract_path.size() - 1; ++i) 
    {
        uint32_t u = abstract_path[i];
        uint32_t v = abstract_path[i+1];
        
        // 1. Check if (u -> v) is already a direct base edge
        bool is_base_edge = false;
        for (uint32_t e = data.routing_head[u]; e < data.routing_head[u+1]; ++e) 
        {
            if (data.routing_edges[e].to == v) 
            {
                is_base_edge = true;
                break;
            }
        }
        
        if (is_base_edge) 
        {
            // It was a physical road hop. Add the destination node directly.
            detailed_path.push_back(v);
        } 
        else 
        {
            // It was a shortcut! Unpack it to get the missing physical roads.
            std::vector<uint32_t> sub_path = localized_shortcut_unpack(u, v, data);
            
            // Append the sub-path (skipping the first node 'u' to avoid duplicate overlapping)
            detailed_path.insert(detailed_path.end(), sub_path.begin() + 1, sub_path.end());
        }
    }
    
    return detailed_path;
}

std::string formatDuration(uint32_t seconds) 
{
     uint32_t mins = seconds / 60;
     if (mins < 60) 
     {
         return std::to_string(mins) + " mins";
     }
     uint32_t hours = mins / 60;
     uint32_t rem_mins = mins % 60;
     return std::to_string(hours) + " hr " + std::to_string(rem_mins) + " mins";
}

std::string formatDistance(uint32_t meters) 
{
     if (meters < 1000) 
     {
         return std::to_string(meters) + " m";
     }
     double km = meters / 1000.0;
     char buf[16];
     snprintf(buf, sizeof(buf), "%.1f km", km);
     return std::string(buf);
}

Json::Value build_geojson_response(const std::vector<uint32_t>& detailed_path, 
                                   const GraphDataView& graph_view) 
{
     // 1. Initialize the Root GeoJSON Object
     Json::Value geojson;
     geojson["type"] = "FeatureCollection";
    
     // 2. Initialize the single Feature
     Json::Value feature;
     feature["type"] = "Feature";
    
     // 3. Initialize the Geometry
     Json::Value geometry;
     geometry["type"] = "LineString";
    
     // 4. Trackers for total trip metrics
     uint32_t total_time_seconds = 0;
     uint32_t total_distance_meters = 0;

     // 5. Build the Coordinate Array and Accumulate Metrics
     Json::Value coordinates(Json::arrayValue);

     for (size_t i = 0; i < detailed_path.size(); ++i) 
     {
         uint32_t ebg_node_id = detailed_path[i];
        
         // Safety check against mapped memory size
         if (ebg_node_id >= graph_view.total_nodes) 
         {
             continue;
         }

         // Accumulate metrics from node_metadata
         total_time_seconds += graph_view.node_metadata[ebg_node_id].wt;
         total_distance_meters += graph_view.node_metadata[ebg_node_id].distance_meters;

         bool reverse_order = graph_view.node_metadata[ebg_node_id].is_reverse_geometry;
         uint32_t offset = graph_view.node_metadata[ebg_node_id].geometry_offset;
         uint32_t size = graph_view.node_metadata[ebg_node_id].geometry_size;

         for (uint32_t j = 0; j < size; ++j) 
         {
             uint32_t index = reverse_order ? (offset + size - 1 - j) : (offset + j);
            
             // Create a [lon, lat] pair
             Json::Value coord_pair(Json::arrayValue);
             coord_pair.append(graph_view.coordinates[index].lon);
             coord_pair.append(graph_view.coordinates[index].lat);
             
             // Append to the main coordinates array
             coordinates.append(coord_pair);
         }
     }
    
     // 6. Populate Feature Properties (Timing & Distance metadata)
     Json::Value properties;
     properties["name"] = "Route";
     properties["duration_seconds"] = total_time_seconds;
     properties["duration_string"] = formatDuration(total_time_seconds);
     properties["distance_meters"] = total_distance_meters;
     properties["distance_string"] = formatDistance(total_distance_meters);

     feature["properties"] = properties;

     // 7. Assemble the JSON Hierarchy
     geometry["coordinates"] = coordinates;
     feature["geometry"] = geometry;
    
     Json::Value features(Json::arrayValue);
     features.append(feature);
    
     geojson["features"] = features;

     return geojson;
}


Json::Value getPathJson(uint32_t startNodeId,uint32_t destNodeId,const GraphDataView&data)
{
     std::vector<uint32_t> abstract_path;  

     //auto start_math = std::chrono::high_resolution_clock::now();
     multi_level_dijsktra(data,startNodeId,destNodeId,abstract_path);
     //auto end_math = std::chrono::high_resolution_clock::now();

     //auto start_json = std::chrono::high_resolution_clock::now();
     Json::Value json=build_geojson_response(unpack_abstract_path(abstract_path,data),data) ;
     //auto end_json = std::chrono::high_resolution_clock::now();

     //auto math_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_math - start_math).count();
     //auto json_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_json - start_json).count();

     //std::cout<< "Route: " << startNodeId << " -> " << destNodeId<<"\n";
     //std::cout<< "Pathfinding + Unpack Time: " << math_time << " ms\n";
     //std::cout<< "JSON Serialization Time: " << json_time << " ms\n";
     return json;//unpack_abstract_path(abstract_path,data)
}
/*
//load all the data from the pre processed files
void load_all_routing_data(const std::string& directory, RoutingData& data) 
{
     std::cout << "Loading Routing Engine Data into RAM...\n";

     // --------------------------------------------------------
     // 1. Load the Base Graph (Forward)
     // --------------------------------------------------------
     std::string routing_path = directory + "/routing_csr.bin";
     std::ifstream route_in(routing_path, std::ios::binary);
     if (!route_in) throw std::runtime_error("Failed to open " + routing_path);

     uint32_t total_edges = 0;
     route_in.read(reinterpret_cast<char*>(&data.total_nodes), sizeof(uint32_t));
     route_in.read(reinterpret_cast<char*>(&total_edges), sizeof(uint32_t));

     data.routing_head.resize(data.total_nodes + 1);
     data.routing_edges.resize(total_edges);

     route_in.read(reinterpret_cast<char*>(data.routing_head.data()), data.routing_head.size() * sizeof(uint32_t));
     route_in.read(reinterpret_cast<char*>(data.routing_edges.data()), data.routing_edges.size() * sizeof(RoutingEdge));
     route_in.close();
     std::cout << "Loaded Forward Base Graph: " << data.total_nodes << " nodes, " << total_edges << " edges.\n";

     // --------------------------------------------------------
     // 2. Load the Base Graph (Reverse)
     // --------------------------------------------------------
     std::string rev_routing_path = directory + "/reverse_routing_csr.bin";
     std::ifstream rev_route_in(rev_routing_path, std::ios::binary);
     if (!rev_route_in) throw std::runtime_error("Failed to open " + rev_routing_path);

     uint32_t rev_total_nodes = 0, rev_total_edges = 0;
     rev_route_in.read(reinterpret_cast<char*>(&rev_total_nodes), sizeof(uint32_t));
     rev_route_in.read(reinterpret_cast<char*>(&rev_total_edges), sizeof(uint32_t));

     data.reverse_routing_head.resize(rev_total_nodes + 1);
     data.reverse_routing_edges.resize(rev_total_edges);

     rev_route_in.read(reinterpret_cast<char*>(data.reverse_routing_head.data()), data.reverse_routing_head.size() * sizeof(uint32_t));
     rev_route_in.read(reinterpret_cast<char*>(data.reverse_routing_edges.data()), data.reverse_routing_edges.size() * sizeof(RoutingEdge));
     rev_route_in.close();
     std::cout << "Loaded Reverse Base Graph.\n";

     // --------------------------------------------------------
     // 3. Load the Node Cell Mapping 
     // --------------------------------------------------------
     std::string mapping_path = directory + "/node_cell_mapping.bin";
     std::ifstream map_in(mapping_path, std::ios::binary);
     if (!map_in) throw std::runtime_error("Failed to open " + mapping_path);
    
     data.node_cell_mapping.resize(data.total_nodes);
     map_in.read(reinterpret_cast<char*>(data.node_cell_mapping.data()), data.total_nodes * sizeof(uint16_t));
     map_in.close();
     std::cout << "Loaded Node Cell Mapping.\n";

     // --------------------------------------------------------
     // 4. Load the Overlay Graph (Forward)
     // --------------------------------------------------------
     std::string fwd_ov_head_path = directory + "/overlay_head.bin";
     std::string fwd_ov_edges_path = directory + "/overlay_edges.bin";
     
     std::ifstream fwd_ov_head_in(fwd_ov_head_path, std::ios::binary);
     if (!fwd_ov_head_in) throw std::runtime_error("Failed to open " + fwd_ov_head_path);
     
     uint32_t total_ov_nodes = 0, total_ov_edges = 0;
     fwd_ov_head_in.read(reinterpret_cast<char*>(&total_ov_nodes), sizeof(uint32_t));
     fwd_ov_head_in.read(reinterpret_cast<char*>(&total_ov_edges), sizeof(uint32_t));
     
     data.overlay_head.resize(total_ov_nodes + 1);
     fwd_ov_head_in.read(reinterpret_cast<char*>(data.overlay_head.data()), data.overlay_head.size() * sizeof(uint32_t));
     fwd_ov_head_in.close();

     std::ifstream fwd_ov_edges_in(fwd_ov_edges_path, std::ios::binary);
     if (!fwd_ov_edges_in) throw std::runtime_error("Failed to open " + fwd_ov_edges_path);
     
     data.overlay_edges.resize(total_ov_edges);
     fwd_ov_edges_in.read(reinterpret_cast<char*>(data.overlay_edges.data()), data.overlay_edges.size() * sizeof(ShortcutEdge));
     fwd_ov_edges_in.close();
     std::cout << "Loaded Forward Overlay Graph: " << total_ov_edges << " shortcut edges.\n";

     // --------------------------------------------------------
     // 5. Load the Overlay Graph (Reverse)
     // --------------------------------------------------------
     std::string rev_ov_head_path = directory + "/reverse_overlay_head.bin";
     std::string rev_ov_edges_path = directory + "/reverse_overlay_edges.bin";
     
     std::ifstream rev_ov_head_in(rev_ov_head_path, std::ios::binary);
     if (!rev_ov_head_in) throw std::runtime_error("Failed to open " + rev_ov_head_path);
     
     uint32_t rev_total_ov_nodes = 0, rev_total_ov_edges = 0;
     rev_ov_head_in.read(reinterpret_cast<char*>(&rev_total_ov_nodes), sizeof(uint32_t));
     rev_ov_head_in.read(reinterpret_cast<char*>(&rev_total_ov_edges), sizeof(uint32_t));
     
     data.reverse_overlay_head.resize(rev_total_ov_nodes + 1);
     rev_ov_head_in.read(reinterpret_cast<char*>(data.reverse_overlay_head.data()), data.reverse_overlay_head.size() * sizeof(uint32_t));
     rev_ov_head_in.close();

     std::ifstream rev_ov_edges_in(rev_ov_edges_path, std::ios::binary);
     if (!rev_ov_edges_in) throw std::runtime_error("Failed to open " + rev_ov_edges_path);
     
     data.reverse_overlay_edges.resize(rev_total_ov_edges);
     rev_ov_edges_in.read(reinterpret_cast<char*>(data.reverse_overlay_edges.data()), data.reverse_overlay_edges.size() * sizeof(ShortcutEdge));
     rev_ov_edges_in.close();
     std::cout << "Loaded Reverse Overlay Graph.\n";


     std::string coord_path = directory + "/geometry.bin";
     std::ifstream coord_in(coord_path, std::ios::binary);
     if (coord_in) 
     {
         uint32_t saved_total_coords = 0;
         coord_in.read(reinterpret_cast<char*>(&saved_total_coords), sizeof(uint32_t));

         // 2. NOW safely read ALL 139M coordinates
         data.coordinates.resize(saved_total_coords);
         coord_in.read(reinterpret_cast<char*>(data.coordinates.data()), saved_total_coords * sizeof(Coordinate));
         
         coord_in.close();
         std::cout << "Loaded Node Coordinates successfully!\n";
     }
     else 
     {
         std::cout << "Warning: geometry.bin not found. GeoJSON export will be disabled.\n";
     }

    
    std::ifstream meta_in(directory + "/metadata.bin", std::ios::binary);
    if (meta_in) 
    {
        size_t total_ebg_nodes = 0;
        meta_in.read(reinterpret_cast<char*>(&total_ebg_nodes), sizeof(total_ebg_nodes));
        data.node_metadata.resize(total_ebg_nodes);

        for (size_t i = 0; i < total_ebg_nodes; ++i) 
        {
            // We must skip over the other properties to extract just offset and size.
            // Based on your parser Phase 3.9, the data layout is:
            // id(4), originalId(8), start_node(8), end_node(8), wt(4), dist(4), cell(2), type(1), name(4), oneway(1)
            // Total bytes to skip = 44 bytes
            
            meta_in.seekg(44, std::ios::cur); 
            
            // Now read the two fields we actually need
            meta_in.read(reinterpret_cast<char*>(&data.node_metadata[i].is_reverse_geometry), sizeof(bool));
            meta_in.read(reinterpret_cast<char*>(&data.node_metadata[i].geometry_offset), sizeof(uint32_t));
            meta_in.read(reinterpret_cast<char*>(&data.node_metadata[i].geometry_size), sizeof(uint32_t));
        }
        meta_in.close();
        std::cout << "Loaded EBG Metadata for " << total_ebg_nodes << " roads.\n";
    }

     std::cout << "All structures successfully loaded and completely unblocked!\n";
}


void export_path_to_geojson(const std::vector<uint32_t>& detailed_path, 
                            const std::vector<Coordinate>& global_coords, 
                            const std::vector<EBG_Metadata>& ebg_metadata, 
                            const std::string& output_filename) 
{
    std::ofstream out(output_filename);
    if (!out) return;

    out << "{\n  \"type\": \"FeatureCollection\",\n  \"features\": [\n    {\n      \"type\": \"Feature\",\n";
    out << "      \"properties\": {\"name\": \"Route\"},\n      \"geometry\": {\n        \"type\": \"LineString\",\n        \"coordinates\": [\n";

    bool first_coord = true;

    for (size_t i = 0; i < detailed_path.size(); ++i) 
    {
        uint32_t ebg_node_id = detailed_path[i];
        if (ebg_node_id >= ebg_metadata.size()) continue;

        bool reverse_order = ebg_metadata[ebg_node_id].is_reverse_geometry;
        uint32_t offset = ebg_metadata[ebg_node_id].geometry_offset;
        uint32_t size = ebg_metadata[ebg_node_id].geometry_size;

        for (uint32_t j = 0; j < size; ++j) 
        {
            uint32_t index = reverse_order ? (offset + size - 1 - j) : (offset + j);
            if (index >= global_coords.size()) continue;

            if (!first_coord) out << ",\n";
            out << "          [" << global_coords[index].lon << ", " << global_coords[index].lat << "]";
            first_coord = false;
        }
    }

    out << "\n        ]\n      }\n    }\n  ]\n}\n";
    out.close();
}



int main()
{
     RoutingData data;
     std::string directory = "/home/aritra/MultiLevelDijsktra/parsing";

     // 1. Load Everything
     load_all_routing_data(directory, data);
     
     // Note: Pick actual valid node IDs from your dataset for testing!
     uint32_t source_node = 36235753; 
     uint32_t target_node = 1776583; 
     
     std::vector<uint32_t> abstract_path;
     
     // 2. Run Engine
     multi_level_dijsktra(data, source_node, target_node, abstract_path);
     
     if (!abstract_path.empty()) 
     {
         // 3. Unpack Shortcuts
         std::vector<uint32_t> detailed_path = unpack_abstract_path(abstract_path, data);
         
         // 4. Export to GeoJSON
         export_path_to_geojson(detailed_path,data.coordinates,data.node_metadata,"route_output.geojson");
     }
     else
     {
         std::cout << "Engine failed to find a path.\n";
     }
     
     return 0;
}
*/


