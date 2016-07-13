#include "vlp16webserver.h"


#define ROS_WARNING_STREAM(...)


VLP16WebServer::VLP16WebServer(HTTPClient * _ptrHTTPClient, const std::string & _network_sensor_ip)
{
    ptr_httpclient = _ptrHTTPClient;
    network_sensor_ip_ = _network_sensor_ip;
    init();
}

void VLP16WebServer::init()
{
    // url: http://aantron.github.io/better-enums/tutorial/Iteration.html
    for (auto name : eVLP16WebServerRequests::_names())
        map_requests_get_[name] = std::string("/cgi/") + std::string(name) + std::string(".json");
}

std::string VLP16WebServer::generate_path_for_get(const std::string _request_name) const
{
    std::string path="";
    try {
        path = map_requests_get_.at(_request_name);
    }
    catch(std::exception &exc){
        ROS_WARNING_STREAM(_request_name << " n'est pas gere!");
    }
    return path;
}

std::string VLP16WebServer::get(const std::string &_request_name)
{
    std::string response="";
    try {
        const std::string path = generate_path_for_get(_request_name);
        ptr_httpclient->get(network_sensor_ip_, path);
        response = ptr_httpclient->get_response();
    }
    catch(std::exception &exc)
    {
    }
    return response;
}

std::string VLP16WebServer::get(eVLP16WebServerRequests _request_id)
{
    return get(_request_id._to_string());
}

void VLP16WebServer::post(const std::string &_xwwwformcoded)
{
    try {
        const std::string path = "/cgi/setting";
        ptr_httpclient->post(network_sensor_ip_, path, _xwwwformcoded);
    }
    catch(std::exception &exc)
    {

    }
}

