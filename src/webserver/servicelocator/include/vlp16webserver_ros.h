#ifndef __VLP16WEBSERVER_ROS__
#define __VLP16WEBSERVER_ROS__

#include "vlp16webserver.h" // for: VLP16WebServer
#include <string>   // for: std::to_string


// Prototype de velodyne_configuration::VLP16_settingsConfig
namespace velodyne_configuration { class VLP16_settingsConfig; }
using namespace velodyne_configuration;

BETTER_ENUM( eReturnType, uint8_t,
             Strongest=0,
             Last,
             Dual
             );

class VLP16WebServerROS : public VLP16WebServer {
public:
    VLP16WebServerROS(HTTPClient * _ptrHTTPClient, const std::string & _network_sensor_ip="");

    void post(const VLP16_settingsConfig& _vlp16config);
    inline void post(const std::string &_xwwwformcoded) override { VelodyneWebServerMeta::post(_xwwwformcoded); }

private:
    static std::string convert_config_to_xwwwformcoded(const VLP16_settingsConfig& _config);
};


#endif
