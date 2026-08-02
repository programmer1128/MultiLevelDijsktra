#pragma once
#include<cstdint>
#include<vector>

typedef struct KDNode 
{
     double lat;
     double lon;
     uint32_t node_id;
}KDNode;

void build_KD_Tree(std::vector<KDNode>& kd_nodes, int l, int h, bool along_X_Axis);

void query_kd_tree(const std::vector<KDNode>&kd_nodes,double lat,double lon,int l,int h,
     bool alongXAxis, double& minDist,KDNode& closestNode);

uint32_t findClosestNode(const std::vector<KDNode>&kd_nodes,double lat,double lon);

