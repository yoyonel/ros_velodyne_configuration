#ifndef VELODYNEWEBSERVER_H
#define VELODYNEWEBSERVER_H

#include <string>   // for: std::string


// Prototype for using HTTPClient
class HTTPClient;


class VelodyneWebServer {
public:
    virtual ~VelodyneWebServer() {}

    virtual std::string get(const std::string &_request_name) = 0;
    virtual void post(const std::string &_xwwwformcoded) = 0;

    inline void setNetworkSensorIP(const std::string & _network_sensor_ip) { network_sensor_ip_ = _network_sensor_ip; }

protected:
    HTTPClient* ptr_httpclient;
    std::string network_sensor_ip_;
};


#endif // VELODYNEWEBSERVER_H
