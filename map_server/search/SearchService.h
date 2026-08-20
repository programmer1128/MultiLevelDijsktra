#pragma once
#include<drogon/drogon.h>
#include "/home/aritra/MultiLevelDijsktra/map_server/plugins/GraphDataPlugin.hpp"
#include "multi_level_dijkstra.h"

Json::Value normal_routing(double startLat,double startLon,double destLat,double destLon,double compass);