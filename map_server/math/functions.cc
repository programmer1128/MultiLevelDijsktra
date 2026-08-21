#include "functions.h"

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