#include "GraphDataPlugin.hpp"
#include <drogon/drogon.h>
#include <cstdlib> // Included for std::getenv

void GraphDataPlugin::initAndStart(const Json::Value &config) 
{
     std::string dir;

     // 1. Priority: Check for Environment Variable
     if(const char* env_path = std::getenv("MAP_DATA_DIR")) 
     {
         dir = env_path;
     } 
     // 2. Second Priority: config.json (Only if the path physically exists on disk)
     else if (config.isMember("graph_dir") && 
              std::filesystem::exists(config["graph_dir"].asString())) 
     {
         dir = config["graph_dir"].asString();
     } 
     // 3. Fallback Priority: Your local Fedora folder
     else 
     {
         dir = "/home/aritra/MultiLevelDijsktra/parsing";
     }
    
     LOG_INFO << "Initializing MMap Graph Data from: " << dir;
    
     try 
     {
         // Map the OS memory
         view = loader.load_all(dir);
        
         // Build the KD-Tree array
         kd_tree_holder.initialize(view);
        
         LOG_INFO << "Plugin initialized and ready for concurrent queries!";
     } 
     catch (const std::exception& e) 
     {
         LOG_FATAL << "Failed to load graph data or build KD-Tree: " << e.what();
         abort();
     }
}

void GraphDataPlugin::shutdown() 
{
     LOG_INFO << "Unmapping Graph Data and shutting down Plugin...";
}