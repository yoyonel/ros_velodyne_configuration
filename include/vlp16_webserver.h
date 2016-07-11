#ifndef VELODYNE_WEBSERVER_VLP16_H
#define VELODYNE_WEBSERVER_VLP16_H

#include <velodyne_webserver.h>

//#include <velodyne_configuration/VLP16_StatusService.h>
//#include <velodyne_configuration/VLP16_DiagnosticsService.h>
//#include <velodyne_configuration/VLP16_DiagnosticsRawService.h>
//#include <velodyne_configuration/VLP16_SettingsService.h>
#include <velodyne_configuration/VLP16_settingsConfig.h>

#include <boost/assign.hpp>

namespace vlp16_webserver {

using namespace velodyne_configuration;
using namespace velodyne_webserver;


/**
 * @brief The VLP16_WebServer class
 */
class VLP16_WebServer: public Velodyne_WebServer
{
public:
    // url: http://stackoverflow.com/questions/2290733/initialize-parents-protected-members-with-initialization-list-c
    VLP16_WebServer();

    virtual int send(const VLP16_settingsConfig& _config) const = 0;
    virtual std::string request(const WebServerCommands & _cmd) const = 0;

    //
    std::string convert_config_to_xwwwformcoded(const VLP16_settingsConfig& _config) const;

    bool get_ip_from_ros_params(const ros::NodeHandle &_n, const std::string &_param_name="VLP16_NETWORK_SENSOR_IP");

protected:
    //
    std::string request_webserver_curl(const WebServerCommands& _cmd) const;
    std::string request_webserver_asio_synch(const WebServerCommands& _cmd) const;
    std::string request_webserver_asio_asynch(const WebServerCommands& _cmd) const;
    //
    int send_settings_to_webserver_curl(const VLP16_settingsConfig& _config) const;
    int send_settings_to_webserver_asio_asynch(const VLP16_settingsConfig& _config) const;
    int send_settings_to_webserver_asio_synch(const VLP16_settingsConfig& _config) const;
};

/**/
template< Velodyne_WebServer::WebServerConnectionType >
struct VLP16_WebServer_Template
        : VLP16_WebServer {
};

typedef VLP16_WebServer_Template< Velodyne_WebServer::WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS > VLP16_WebServer_BOOST_ASIO_ASYNCHRONOUS;
typedef VLP16_WebServer_Template< Velodyne_WebServer::WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS > VLP16_WebServer_BOOST_ASIO_SYNCHRONOUS;
typedef VLP16_WebServer_Template< Velodyne_WebServer::WebServerConnectionType::CURL > VLP16_WebServer_CURL;

#include "../src/webserver/connections/vlp16_webserver.inl"

}

#endif // VELODYNE_WEBSERVER_VLP16_H
