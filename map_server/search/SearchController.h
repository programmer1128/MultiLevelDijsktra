#pragma once

#include<drogon/drogon.h>
#include <drogon/utils/coroutine.h>
#include<drogon/HttpController.h>

using namespace drogon;

class SearchController : public HttpController<SearchController>
{
     public:
    
     METHOD_LIST_BEGIN
     
     ADD_METHOD_TO(SearchController::getRoute, "/api/route?start_lat={1}&start_lon={2}&dest_lat={3}&dest_lon={4}&compass={%}", drogon::Get);

     METHOD_LIST_END

     drogon::Task<drogon::HttpResponsePtr> getRoute(drogon::HttpRequestPtr req,
                                                   double start_lat, 
                                                   double start_lon, 
                                                   double dest_lat, 
                                                   double dest_lon,double compass);
};
