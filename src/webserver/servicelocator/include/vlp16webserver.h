#ifndef VLP16WEBSERVER_H
#define VLP16WEBSERVER_H

#include "velodynewebserver.h"  // for: VelodyneWebServer
#include <enum.h>   // for: BETTER_ENUM
#include <httpclient.h> // for: HTTPClient
#include <map>  // for: std:map


BETTER_ENUM( eVLP16WebServerRequests, uint8_t,
             settings=1,
             status,
             info,
             diag
             );


class VLP16WebServer : public VelodyneWebServer {
public:
    // ----------------
    // CONSTRUCTOR
    // ----------------
    VLP16WebServer(HTTPClient * _ptrHTTPClient, const std::string & _network_sensor_ip="");
    // ----------------
    // VIRTUAL DESTRUCTOR
    // ----------------
    ~VLP16WebServer() {}

    // ----------------
    // OVERRIDES
    // ----------------
    std::string get(const std::string &_request_name) override;
    virtual void post(const std::string &_xwwwformcoded) override;

    // ----------------
    // SERVICE
    // ----------------
    std::string get(eVLP16WebServerRequests _request_id);

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

#endif // VLP16WEBSERVER_H
