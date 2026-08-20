#include "KDTreeHolder.hpp"

uint32_t calculate_distance(double lat1, double lon1, double lat2, double lon2) 
{
     const double R = 6371e3; // Earth radius in meters
     double phi1 = lat1 * M_PI / 180;
     double phi2 = lat2 * M_PI / 180;
     double dphi = (lat2 - lat1) * M_PI / 180;
     double dlambda = (lon2 - lon1) * M_PI / 180;

     double a = std::sin(dphi/2) * std::sin(dphi/2) +
              std::cos(phi1) * std::cos(phi2) *
              std::sin(dlambda/2) * std::sin(dlambda/2);
     double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
     return static_cast<uint32_t>(R * c);
}


double calculateBearing(double lat1, double lon1, double lat2, double lon2) 
{
     double dLon = (lon2 - lon1) * M_PI / 180.0;
     lat1 = lat1 * M_PI / 180.0;
     lat2 = lat2 * M_PI / 180.0;
    
     double x = std::sin(dLon) * std::cos(lat2);
     double y = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
    
     double bearing = std::atan2(x, y) * 180.0 / M_PI;
     return std::fmod(bearing + 360.0, 360.0); // Normalize to 0-360 degrees
}

void KDTreeHolder::initialize(const GraphDataView& view) 
{
     LOG_INFO<< "Constructing KD-Tree array in RAM\n";
     kd_nodes_array.resize(view.total_nodes);

     for (uint32_t i = 0; i < view.total_nodes; ++i) 
     {
         const auto& meta = view.node_metadata[i];
         uint32_t offset = meta.geometry_offset;
 
         kd_nodes_array[i].node_id = meta.id;
         kd_nodes_array[i].lat = view.coordinates[offset].lat;
         kd_nodes_array[i].lon = view.coordinates[offset].lon;

         if (meta.geometry_size > 1) 
         {
             double next_lat = view.coordinates[offset + 1].lat;
             double next_lon = view.coordinates[offset + 1].lon;
             
             // Calling the calculateBearing spherical trig helper function
             kd_nodes_array[i].bearing = calculateBearing(
                 kd_nodes_array[i].lat, kd_nodes_array[i].lon, 
                 next_lat, next_lon
             );
         } 
         else 
         {
             // Fallback for single-point anomalies in OSM data
             kd_nodes_array[i].bearing = 0.0; 
         }
         if (i > 0 && i % 1000000 == 0) 
         {
             LOG_INFO<< "Loaded " << i << " nodes into RAM\n";
         }
     }

     if (!kd_nodes_array.empty()) 
     {
         LOG_INFO<< "Starting heavy spatial sort for " << kd_nodes_array.size() << " nodes\n";
         build_KD_Tree(kd_nodes_array, 0, kd_nodes_array.size() - 1, true);
     } 
     LOG_INFO<< "KD-Tree array ready\n";
}