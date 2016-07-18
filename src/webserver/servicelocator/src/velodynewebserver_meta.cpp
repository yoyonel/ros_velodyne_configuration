//#include "velodynewebserver_meta.h"
//
//#include "velodynewebserver_meta.instantiation"


//#define ROS_WARNING_STREAM(...)


//template<typename _>
//VelodyneWebServerMeta<_>::VelodyneWebServerMeta(HTTPClient * _ptrHTTPClient, const std::string & _network_sensor_ip)
//{
//    ptr_httpclient = _ptrHTTPClient;
//    network_sensor_ip_ = _network_sensor_ip;
//    init();
//}

//template<typename eListRequests>
//void VelodyneWebServerMeta<eListRequests>::init()
//{
//    // url: http://aantron.github.io/better-enums/tutorial/Iteration.html
//    for (auto name : eListRequests::_names())
//        map_requests_get_[name] = build_path_for_get(name);

//}

//template<typename _>
//std::string VelodyneWebServerMeta<_>::build_path_for_get(const std::string _request_name)
//{
//    return std::string("/cgi/") + std::string(_request_name) + std::string(".json");
//}

//template<typename _>
//std::string VelodyneWebServerMeta<_>::get_path_for_get(const std::string _request_name) const
//{
//    std::string path="";
//    try {
//        path = map_requests_get_.at(_request_name);
//    }
//    catch(std::exception &exc){
//        ROS_WARNING_STREAM(_request_name << " n'est pas gere!");
//    }
//    return path;
//}

//template<typename _>
//std::string VelodyneWebServerMeta<_>::get(const std::string &_request_name)
//{
//    std::string response="";
//    try {
//        const std::string path = get_path_for_get(_request_name);
//        ptr_httpclient->get(network_sensor_ip_, path);
//        response = ptr_httpclient->get_response();
//    }
//    catch(std::exception &exc)
//    {
//    }
//    return response;
//}

//template<typename eCmd>
//std::string VelodyneWebServerMeta<eCmd>::get(eCmd _request_id)
//{
//    return get(_request_id._to_string());
//}

//template<typename _>
//std::string VelodyneWebServerMeta<_>::build_path_for_post()
//{
//    return std::string("/cgi/setting");
//}

//template<typename _>
//void VelodyneWebServerMeta<_>::post(const std::string &_xwwwformcoded)
//{
//    try {
//        const std::string path = build_path_for_post();
//        ptr_httpclient->post(network_sensor_ip_, path, _xwwwformcoded);
//    }
//    catch(std::exception &exc)
//    {

//    }
//}
