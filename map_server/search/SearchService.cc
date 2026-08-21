//this class uses the kd tree to get the start and end node coordinates and then
//applies the multi-level dijkstra to get the geojson of the entire path
#include<drogon/drogon.h>
#include "kd_tree.h"
#include "SearchService.h"


Json::Value serialize_to_json(const NavigationPayload& payload) 
{
      //root geojson value
      Json::Value geojson;
      geojson["type"] = "FeatureCollection";
      Json::Value feature;
      feature["type"] = "Feature";
      Json::Value properties;
      properties["name"] = "Route";
      properties["duration_seconds"] = payload.total_duration_seconds;
      properties["duration_string"] = payload.duration_string;
      properties["distance_meters"] = payload.total_distance_meters;
      properties["distance_string"] = payload.distance_string;
      feature["properties"] = properties;

      Json::Value coordinates(Json::arrayValue);
      for (const auto& coord : payload.route_geometry) 
      {
           Json::Value coord_pair(Json::arrayValue);
           coord_pair.append(coord.lon);
           coord_pair.append(coord.lat);
          coordinates.append(coord_pair);
      }
      Json::Value geometry;
      geometry["type"] = "LineString";
      geometry["coordinates"] = coordinates;
      feature["geometry"] = geometry;
    
      Json::Value features(Json::arrayValue);
      features.append(feature);
      geojson["features"] = features;

      //corridor graph
      Json::Value corridor;
      Json::Value routePathJson(Json::arrayValue);
      for (uint32_t u : payload.route_path_ids) routePathJson.append(u);
    
      Json::Value edgesJson(Json::arrayValue);
      for (const auto& edge : payload.corridor_edges) 
      {
           Json::Value edgeObj;
           edgeObj["from"] = edge.from;
           edgeObj["to"] = edge.to;
           edgesJson.append(edgeObj);
      }

      Json::Value nodesJson(Json::objectValue);
      for (const auto& [node_id, node_data] : payload.corridor_nodes) 
      {
           Json::Value nodeObj;
           nodeObj["lat"] = node_data.lat;
           nodeObj["lon"] = node_data.lon;
           nodeObj["bearing"] = node_data.bearing;
           nodesJson[std::to_string(node_id)] = nodeObj;
      }

      corridor["route_path"] = routePathJson;
      corridor["edges"] = edgesJson;
      corridor["nodes"] = nodesJson;

      
      Json::Value final_response;
      final_response["geojson"] = geojson;
      final_response["corridor"] = corridor;

      return final_response;
}


Json::Value getPathJson(uint32_t startNodeId,uint32_t destNodeId,const GraphDataView&data)
{
      std::vector<uint32_t> abstract_path;  

      //auto start_math = std::chrono::high_resolution_clock::now();
      multi_level_dijsktra(data,startNodeId,destNodeId,abstract_path);
      //auto end_math = std::chrono::high_resolution_clock::now();

      //auto start_json = std::chrono::high_resolution_clock::now();
      if (abstract_path.empty()) 
      {
           return Json::Value(Json::objectValue); 
      }

      std::vector<uint32_t> detailed_path = unpack_abstract_path(abstract_path, data);
     
      //Extract pure data
      NavigationPayload payload = extract_navigation_payload(detailed_path, data);
     
      //Serialize to chosen format
      return serialize_to_json(payload);
}


Json::Value normal_routing(double startLat,double startLon,double destLat,double destLon,double compass)
{
      //get the global plugin
      auto plugin=drogon::app().getPlugin<GraphDataPlugin>();
      //query the kd tree with the query method to get the node ids     
     
      uint32_t startNodeId=findBestNodeWithHeading(plugin->getKDArray(),startLat,startLon,compass);
      uint32_t destNodeId=findClosestNode(plugin->getKDArray(),destLat,destLon);

      Json::Value geoJson=getPathJson(startNodeId,destNodeId,plugin->getView());

      return geoJson;
}
