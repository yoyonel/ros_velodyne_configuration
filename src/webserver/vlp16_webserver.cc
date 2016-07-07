#include <vlp16_webserver.h>
#include <boost/asio.hpp>


namespace vlp16_webserver {

VLP16_WebServer::VLP16_WebServer()
{
    network_sensor_ip_ = DEFAULT_NETWORK_SENSOR_IP;
    max_delay_for_cmd_ = DEFAULT_MAX_DELAY_FOR_CMD;
}

std::string VLP16_WebServer::convert_config_to_xwwwformcoded(const VLP16_settingsConfig& _config) const
{
    const std::string result = \
            "laser=" + std::string(_config.laser_state?"on":"off") + "&" \
            "returns=" + LaserReturns::_from_integral(_config.return_type)._to_string() + "&" \
            "rpm=" + std::to_string(_config.rpm);

    return result;
}

bool VLP16_WebServer::get_ip_from_ros_params(const ros::NodeHandle &_n, const std::string &_param_name)
{
    bool return_get_param = _n.getParam(_param_name, network_sensor_ip_);
    ROS_INFO_STREAM("getParam -> VLP16_NETWORK_SENSOR_IP: " << network_sensor_ip_);
    return return_get_param;
}

}
