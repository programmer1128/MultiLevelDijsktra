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
     const GraphDataView& load_all(const std::string& dir) ;
};