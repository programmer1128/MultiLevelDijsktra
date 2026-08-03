#include "GraphDataPlugin.hpp"
#include <drogon/drogon.h>

void GraphDataPlugin::initAndStart(const Json::Value &config) 
{
    std::string dir = config.get("graph_dir", "/home/aritra/MultiLevelDijsktra/parsing").asString();
    
    LOG_INFO << "Initializing MMap Graph Data from: " << dir;
    
    try {
        // Map the OS memory
        view = loader.load_all(dir);
        
        // Build the KD-Tree array
        kd_tree_holder.initialize(view);
        
        LOG_INFO << "Plugin initialized and ready for concurrent queries!";
    } catch (const std::exception& e) {
        LOG_FATAL << "Failed to load graph data or build KD-Tree: " << e.what();
        abort();
    }
}

void GraphDataPlugin::shutdown() 
{
    LOG_INFO << "Unmapping Graph Data and shutting down Plugin...";
}