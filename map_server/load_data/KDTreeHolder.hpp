#pragma once
#include <vector>
#include <iostream>
#include "search/kd_tree.h"
// Include the headers where GraphDataView and KDNode are defined
#include "GraphDataView.hpp" 
#include <drogon/drogon.h>
// #include "kd_tree.h"

class KDTreeHolder 
{
private:
    std::vector<KDNode> kd_nodes_array;

public:
    // Only declare the methods
    void initialize(const GraphDataView& view);
    
    // Tiny inline getters are okay in the header
    const std::vector<KDNode>& getArray() const { return kd_nodes_array; }
};