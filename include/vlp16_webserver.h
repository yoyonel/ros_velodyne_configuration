#ifndef VELODYNE_WEBSERVER_VLP16_H
#define VELODYNE_WEBSERVER_VLP16_H

#include <velodyne_webserver.h>
#include <velodyne_configuration/VLP16_settingsConfig.h>
//#include <boost/assign.hpp>

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
    virtual ~VLP16_WebServer() {}

    virtual int send(const VLP16_settingsConfig& _config) const = 0;
    virtual std::string request(const WebServerCommands & _cmd) const = 0;

protected:
    std::string convert_config_to_xwwwformcoded(const VLP16_settingsConfig& _config) const;
    bool get_ip_from_ros_params(const ros::NodeHandle &_n, const std::string &_param_name="VLP16_NETWORK_SENSOR_IP");
};

/**/
template< Velodyne_WebServer::WebServerConnectionType >
struct VLP16_WebServer_Template : public VLP16_WebServer, std::false_type
{
    virtual std::string request (const WebServerCommands    & _cmd)         const override;
    virtual int         send    (const VLP16_settingsConfig & _config)      const override;
};

typedef VLP16_WebServer_Template< Velodyne_WebServer::WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS > VLP16_WebServer_BOOST_ASIO_ASYNCHRONOUS;
typedef VLP16_WebServer_Template< Velodyne_WebServer::WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS > VLP16_WebServer_BOOST_ASIO_SYNCHRONOUS;
typedef VLP16_WebServer_Template< Velodyne_WebServer::WebServerConnectionType::CURL > VLP16_WebServer_CURL;

#include "../src/webserver/connections/vlp16_webserver.inl"

}

#endif // VELODYNE_WEBSERVER_VLP16_H
