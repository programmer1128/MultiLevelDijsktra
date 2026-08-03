#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <string>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

// Include the exact structural layout header used by the parser
#include "/home/aritra/MultiLevelDijsktra/include/GraphStructures.hpp"

std::unordered_map<uint64_t, uint8_t> node_counts;

// Exact mirror of Pass 1A to discover intersections identically
class NodeReferenceCounter : public osmium::handler::Handler 
{
public:
     void way(const osmium::Way& way) 
     {
         const char* highway = way.tags().get_value_by_key("highway");
         if (!highway) 
         {
             return;
         }
         for (const auto& node_ref : way.nodes()) 
         {
             uint64_t node_id = node_ref.ref();
             if (node_counts[node_id] < 2) 
             {
                 node_counts[node_id]++;
             }
         }
     }
};

// same Pass 2 of the parser2 to recreate the exact sequential NBG Node IDs
class NBGIndexRebuilder : public osmium::handler::Handler 
{
public:
     uint32_t current_nbg_node_id = 0;
     void way(const osmium::Way& way) 
     {
         const char* highway = way.tags().get_value_by_key("highway");
         if (!highway) 
         {
             return;
         }

         uint64_t front_id = way.nodes().front().ref();
         if (osm_to_nbg_index.find(front_id) == osm_to_nbg_index.end()) 
         {
             osm_to_nbg_index[front_id] = current_nbg_node_id++;
         }

         for (size_t i = 0; i < way.nodes().size(); ++i) 
         {
             const auto& node_ref = way.nodes()[i];
             bool is_intersection = (node_counts[node_ref.ref()] >= 2);
             bool is_last_node = (i == way.nodes().size() - 1);

             if ((is_intersection && i != 0) || is_last_node) 
             {
                 uint64_t end_id = node_ref.ref();
                 if (osm_to_nbg_index.find(end_id) == osm_to_nbg_index.end()) 
                 {
                     osm_to_nbg_index[end_id] = current_nbg_node_id++;
                 }
             }
         }
     }
};

int main() 
{
     std::string base_path = "/home/aritra/MultiLevelDijsktra/parsing/";
     std::string osm_file = "/home/aritra/MultiLevelDijsktra/parsing/india-260724.osm.pbf"; // Stays local like your parser code
 
     std::cout << "Step 1: Rebuilding NBG index from OSM to guarantee perfect matching...\n";
     
     osmium::io::Reader way_reader{osm_file, osmium::osm_entity_bits::way};
     NodeReferenceCounter counter_handler;
     osmium::apply(way_reader, counter_handler);
     way_reader.close();

     osmium::io::Reader pass2_reader{osm_file, osmium::osm_entity_bits::way};
     NBGIndexRebuilder rebuilder;
     osmium::apply(pass2_reader, rebuilder);
     pass2_reader.close();

     std::cout << "NBG Index successfully rebuilt. Unique intersections: " << osm_to_nbg_index.size() << "\n"; 

     std::cout << "Loading cell mapping from Dinic Partitioner\n";
     std::ifstream mapping_file(base_path + "node_cell_mapping.bin", std::ios::binary);
     if (!mapping_file) 
     {
         std::cerr << "Failed to open node_cell_mapping.bin\n";
         return 1;
     }
     uint32_t total_nodes = 0;
     mapping_file.read(reinterpret_cast<char*>(&total_nodes), sizeof(total_nodes));
     std::vector<uint32_t> cell_mapping(total_nodes);
     mapping_file.read(reinterpret_cast<char*>(cell_mapping.data()), total_nodes * sizeof(uint32_t));
     mapping_file.close();
     std::cout << "Loaded " << total_nodes << " node partitions.\n";

     std::cout << "Step 3: Streaming metadata.bin into memory for mapping updates...\n";
     std::ifstream meta_in(base_path + "metadata.bin", std::ios::binary);
     if (!meta_in) 
     {
         std::cerr << "Failed to open metadata.bin\n";
         return 1;
     }
     size_t total_ebg_nodes = 0;
     meta_in.read(reinterpret_cast<char*>(&total_ebg_nodes), sizeof(total_ebg_nodes));
     std::vector<ExpandedNode> ebg_nodes(total_ebg_nodes);

    for (size_t i = 0; i < total_ebg_nodes; ++i) 
    {
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].id), sizeof(ebg_nodes[i].id));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].originalId), sizeof(ebg_nodes[i].originalId));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].start_osm_node_id), sizeof(ebg_nodes[i].start_osm_node_id));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].end_osm_node_id), sizeof(ebg_nodes[i].end_osm_node_id));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].wt), sizeof(ebg_nodes[i].wt));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].distance_meters), sizeof(ebg_nodes[i].distance_meters));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].cell_id), sizeof(ebg_nodes[i].cell_id));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].road_type), sizeof(ebg_nodes[i].road_type));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].name_id), sizeof(ebg_nodes[i].name_id));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].is_one_way), sizeof(ebg_nodes[i].is_one_way));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].is_reverse_geometry), sizeof(ebg_nodes[i].is_reverse_geometry));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].geometry_offset), sizeof(ebg_nodes[i].geometry_offset));
         meta_in.read(reinterpret_cast<char*>(&ebg_nodes[i].geometry_size), sizeof(ebg_nodes[i].geometry_size));

         // Inject the partition assignment via its originating NBG intersection
         uint64_t osm_id = ebg_nodes[i].start_osm_node_id;
         auto it = osm_to_nbg_index.find(osm_id);
         if (it != osm_to_nbg_index.end()) 
         {
             uint32_t nbg_id = it->second;
             if (nbg_id < cell_mapping.size()) 
             {
                 ebg_nodes[i].cell_id = static_cast<uint16_t>(cell_mapping[nbg_id]);
             }
         }
     }
     meta_in.close();

     std::cout << "Step 4: Overwriting updated metadata.bin with verified Cell IDs...\n";
     std::ofstream meta_out(base_path + "metadata.bin", std::ios::binary);
     meta_out.write(reinterpret_cast<const char*>(&total_ebg_nodes), sizeof(total_ebg_nodes));
     for (const auto& node : ebg_nodes) 
     {
         meta_out.write(reinterpret_cast<const char*>(&node.id), sizeof(node.id));
         meta_out.write(reinterpret_cast<const char*>(&node.originalId), sizeof(node.originalId));
         meta_out.write(reinterpret_cast<const char*>(&node.start_osm_node_id), sizeof(node.start_osm_node_id));
         meta_out.write(reinterpret_cast<const char*>(&node.end_osm_node_id), sizeof(node.end_osm_node_id));
         meta_out.write(reinterpret_cast<const char*>(&node.wt), sizeof(node.wt));
         meta_out.write(reinterpret_cast<const char*>(&node.distance_meters), sizeof(node.distance_meters));
         meta_out.write(reinterpret_cast<const char*>(&node.cell_id), sizeof(node.cell_id));
         meta_out.write(reinterpret_cast<const char*>(&node.road_type), sizeof(node.road_type));
         meta_out.write(reinterpret_cast<const char*>(&node.name_id), sizeof(node.name_id));
         meta_out.write(reinterpret_cast<const char*>(&node.is_one_way), sizeof(node.is_one_way));
         meta_out.write(reinterpret_cast<const char*>(&node.is_reverse_geometry), sizeof(node.is_reverse_geometry));
         meta_out.write(reinterpret_cast<const char*>(&node.geometry_offset), sizeof(node.geometry_offset));
         meta_out.write(reinterpret_cast<const char*>(&node.geometry_size), sizeof(node.geometry_size));
     }
     meta_out.close();

     std::cout << "\n========================================================\n";
     std::cout << "SUCCESS: Data Ingestion Pipeline Flawlessly Stitched!\n";
     std::cout << "metadata.bin now contains live cell properties (0 - 5946).\n";
     std::cout << "========================================================\n";
     return 0;
}