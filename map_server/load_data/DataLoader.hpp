#pragma once
#include <memory>
#include "MMapFile.hpp"
#include "GraphDataView.hpp"

class DataLoader 
{
private:
     // Store the MMap objects to keep the memory active
     std::unique_ptr<MMapFile> fwd_route_map;
     std::unique_ptr<MMapFile> rev_route_map;
     std::unique_ptr<MMapFile> fwd_ov_head_map;
     std::unique_ptr<MMapFile> fwd_ov_edges_map;
     std::unique_ptr<MMapFile> rev_ov_head_map;
     std::unique_ptr<MMapFile> rev_ov_edges_map;
     std::unique_ptr<MMapFile> cell_map;
     std::unique_ptr<MMapFile> geom_map;
     std::unique_ptr<MMapFile> meta_map;

     GraphDataView view;

public:
     const GraphDataView& load_all(const std::string& dir) 
     {
         std::cout << "Memory-Mapping Graph Data...\n";
 
         //Map Forward Routing CSR
         fwd_route_map = std::make_unique<MMapFile>(dir + "/routing_csr.bin");
         const uint8_t* ptr = fwd_route_map->data();
         view.total_nodes = *reinterpret_cast<const uint32_t*>(ptr);
         uint32_t fwd_edges = *reinterpret_cast<const uint32_t*>(ptr + 4);
         view.routing_head = reinterpret_cast<const uint32_t*>(ptr + 8);
         view.routing_edges = reinterpret_cast<const RoutingEdge*>(ptr + 8 + (view.total_nodes + 1) * 4);

         //Map Reverse Routing CSR
         rev_route_map = std::make_unique<MMapFile>(dir + "/reverse_routing_csr.bin");
         ptr = rev_route_map->data();
         view.reverse_routing_head = reinterpret_cast<const uint32_t*>(ptr + 8);
         view.reverse_routing_edges = reinterpret_cast<const RoutingEdge*>(ptr + 8 + (view.total_nodes + 1) * 4);

         // Map Node Cell Mapping
         cell_map = std::make_unique<MMapFile>(dir + "/node_cell_mapping.bin");
         // cell map is just: total_nodes(4) + array
         view.node_cell_mapping = reinterpret_cast<const uint16_t*>(cell_map->data() + 4);

         //Map Geometry
         geom_map = std::make_unique<MMapFile>(dir + "/geometry.bin");
         // geometry is: total_coords(4) + array
         view.coordinates = reinterpret_cast<const Coordinate*>(geom_map->data() + 4);

         //Map Metadata
         meta_map = std::make_unique<MMapFile>(dir + "/metadata.bin");
         // metadata is: total_nodes(8 bytes because size_t in Phase 3.9) + array
         view.node_metadata = reinterpret_cast<const PackedMetadata*>(meta_map->data() + 8);

         fwd_ov_head_map = std::make_unique<MMapFile>(dir + "/overlay_head.bin");
         view.overlay_head = reinterpret_cast<const uint32_t*>(fwd_ov_head_map->data() + 8);
        
         fwd_ov_edges_map = std::make_unique<MMapFile>(dir + "/overlay_edges.bin");
         view.overlay_edges = reinterpret_cast<const ShortcutEdge*>(fwd_ov_edges_map->data()); // No header on edges file
 
         // 7. Map Reverse Overlay
         rev_ov_head_map = std::make_unique<MMapFile>(dir + "/reverse_overlay_head.bin");
         view.reverse_overlay_head = reinterpret_cast<const uint32_t*>(rev_ov_head_map->data() + 8);
         
         rev_ov_edges_map = std::make_unique<MMapFile>(dir + "/reverse_overlay_edges.bin");
         view.reverse_overlay_edges = reinterpret_cast<const ShortcutEdge*>(rev_ov_edges_map->data());
        
         std::cout << "Engine successfully Memory-Mapped in 0 seconds.\n";
         return view;
     }
};