#include "vlp16webserver_ros.h"
#include <velodyne_configuration/VLP16_settingsConfig.h>    // for: VLP16_settingsConfig
using namespace velodyne_configuration;


#define BOOL2XWWWWFORMCODED(_bool) \
    std::string(_bool ? "on":"off")

#define RETURNTYPE2XWWWWFORMCODED(_returntype) \
    eReturnType::_from_integral(_returntype)._to_string()


VLP16WebServerROS::VLP16WebServerROS(HTTPClient * _ptrHTTPClient, const std::string & _network_sensor_ip)
    : VelodyneWebServerMeta(_ptrHTTPClient, _network_sensor_ip)
{
}

std::string VLP16WebServerROS::convert_config_to_xwwwformcoded(const VLP16_settingsConfig &_config)
{
    const std::string result = \
            "laser="    + BOOL2XWWWWFORMCODED(_config.laser_state)          + "&" \
            "returns="  + RETURNTYPE2XWWWWFORMCODED(_config.return_type)    + "&" \
            "rpm="      + std::to_string(_config.rpm);

    return result;
}

void VLP16WebServerROS::post(const VLP16_settingsConfig& _vlp16config)
{
    post(convert_config_to_xwwwformcoded(_vlp16config));
}
