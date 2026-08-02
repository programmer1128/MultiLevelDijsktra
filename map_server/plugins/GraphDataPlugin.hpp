#pragma once
#include <drogon/plugins/Plugin.h>
#include "../load_data/DataLoader.hpp"
#include "../load_data/KDTreeHolder.hpp"

class GraphDataPlugin : public drogon::Plugin<GraphDataPlugin>
{
private:
    DataLoader loader;
    GraphDataView view;
    KDTreeHolder kd_tree_holder;

public:
    // Only declare the lifecycle methods here
    void initAndStart(const Json::Value &config) override;
    void shutdown() override;

    // Getters can safely remain inline
    const GraphDataView& getView() const { return view; }
    const std::vector<KDNode>& getKDArray() const { return kd_tree_holder.getArray(); } 
};