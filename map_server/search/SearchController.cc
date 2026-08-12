//class file for the implementation of the header
#include"SearchController.h"
#include<drogon/drogon.h>
#include"SearchService.h"

drogon::Task<drogon::HttpResponsePtr> SearchController::getRoute(drogon::HttpRequestPtr req,
                                                                double start_lat, 
                                                                double start_lon, 
                                                                double dest_lat, 
                                                                double dest_lon,double compass)
{
         LOG_INFO << "Routing requested from (" << start_lat << ", " << start_lon 
             << ") to (" << dest_lat << ", " << dest_lon << ")";

         //call to service 
         Json::Value geoJson = normal_routing(start_lat, start_lon, dest_lat, dest_lon);

          auto resp = drogon::HttpResponse::newHttpJsonResponse(geoJson);

         // Return the response using C++20 coroutine syntax
         co_return resp;
}