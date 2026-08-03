//this class uses the kd tree to get the start and end node coordinates and then
//applies the multi-level dijkstra to get the geojson of the entire path
#include<drogon/drogon.h>
#include "kd_tree.h"
#include "SearchService.h"


Json::Value normal_routing(double startLat,double startLon,double destLat,double destLon)
{
     //get the global plugin
     auto plugin=drogon::app().getPlugin<GraphDataPlugin>();
     //query the kd tree with the query method to get the node ids     
     
     uint32_t startNodeId=findClosestNode(plugin->getKDArray(),startLat,startLon);
     uint32_t destNodeId=findClosestNode(plugin->getKDArray(),destLat,destLon);

     Json::Value geoJson=getPathJson(startNodeId,destNodeId,plugin->getView());

     return geoJson;
}
