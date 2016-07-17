#ifndef VELODYNEWEBSERVER_META_H
#define VELODYNEWEBSERVER_META_H


#include "velodynewebserver.h"  // for: VelodyneWebServer
#include <map>  // for: std:map
#include <httpclient.h> // for: HTTPClient


template< typename eListRequests >
class VelodyneWebServerMeta : public VelodyneWebServer {
public:
    // ----------------
    // CONSTRUCTOR
    // ----------------
    VelodyneWebServerMeta(HTTPClient * _ptrHTTPClient, const std::string & _network_sensor_ip="");
    // ----------------
    // VIRTUAL DESTRUCTOR
    // ----------------
    ~VelodyneWebServerMeta() {}

    // ----------------
    // OVERRIDES
    // ----------------
    std::string get(const std::string &_request_name) override;
    virtual void post(const std::string &_xwwwformcoded) override;

    // ----------------
    // SERVICE
    // ----------------
    std::string get(eListRequests _request_id);

    // ----------------
    // STATICS
    // ----------------
    static inline std::string build_path_for_get(const std::string _request_name);
    static inline std::string build_path_for_post();

private:
    void init();
    std::string get_path_for_get(const std::string _request_name) const;


private:
    std::map<std::string, std::string> map_requests_get_;
};


#endif // VELODYNEWEBSERVER_META_H
