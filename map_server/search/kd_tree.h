#pragma once
#include<cstdint>
#include<vector>

typedef struct KDNode 
{
     double lat;
     double lon;
     uint32_t node_id;
     //this bearing field is for the user direction compass
     double bearing;
}KDNode;

void build_KD_Tree(std::vector<KDNode>& kd_nodes, int l, int h, bool along_X_Axis);

void query_kd_tree(const std::vector<KDNode>&kd_nodes,double lat,double lon,int l,int h,
     bool alongXAxis, double& minDist,KDNode& closestNode);

void query_kd_tree_radius(const std::vector<KDNode>& kd_nodes, double lat, double lon, int l, int h,
     bool alongXAxis, double radius_squared, std::vector<KDNode>& candidates);

uint32_t findClosestNode(const std::vector<KDNode>&kd_nodes,double lat,double lon);

uint32_t findBestNodeWithHeading(const std::vector<KDNode>& kd_nodes, 
      double lat, double lon, double user_heading);

