#ifndef VLP16WEBSERVER_H
#define VLP16WEBSERVER_H

#include "velodynewebserver.h"  // for: VelodyneWebServer
#include <enum.h>   // for: BETTER_ENUM
#include <httpclient.h> // for: HTTPClient
#include <map>  // for: std:map


BETTER_ENUM( _eVLP16WebServerRequests_, uint8_t,
             settings=1,
             status,
             info,
             diag
             );


class VLP16WebServer : public VelodyneWebServer {
public:
    typedef _eVLP16WebServerRequests_ eVLP16WebServerRequests;

public:
    VLP16WebServer(HTTPClient * _ptrHTTPClient, const std::string & _network_sensor_ip="");

    std::string get(const std::string &_request_name) override;
    virtual void post(const std::string &_xwwwformcoded) override;

    std::string get(eVLP16WebServerRequests _request_id);

private:
    void init();
    std::string generate_path_for_get(const std::string _request_name) const;

private:
    std::map<std::string, std::string> map_requests_get_;
};

#endif // VLP16WEBSERVER_H
