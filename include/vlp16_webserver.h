#ifndef VELODYNE_WEBSERVER_VLP16_H
#define VELODYNE_WEBSERVER_VLP16_H

#include <velodyne_webserver.h>

#include <velodyne_configuration/VLP16_SettingsService.h>
#include <velodyne_configuration/VLP16_StatusService.h>
#include <velodyne_configuration/VLP16_DiagnosticsService.h>
#include <velodyne_configuration/VLP16_DiagnosticsRawService.h>
#include <velodyne_configuration/VLP16_settingsConfig.h>

#include <boost/assign.hpp>

namespace vlp16_webserver {

using namespace velodyne_configuration;
using namespace velodyne_webserver;

//#define USE_BOOST_FOR_FUNCTION_BINDING
#define USE_STD_FOR_FUNCTION_BINDING

#define DEFAULT_WEBSERVER_CONNECTION_TYPE   WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS
//#define DEFAULT_WEBSERVER_CONNECTION_TYPE   WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS


/**
 * @brief The VLP16_WebServer class
 */
class VLP16_WebServer: public Velodyne_WebServer
{
public:
    // url: http://stackoverflow.com/questions/2290733/initialize-parents-protected-members-with-initialization-list-c
    VLP16_WebServer();

    std::string request_webserver(
            const WebServerCommands &_cmd,
            WebServerConnectionType _typeConnection = DEFAULT_WEBSERVER_CONNECTION_TYPE
            ) const override;

    //
    int send_settings_to_webserver(
            const VLP16_settingsConfig& _config,
            WebServerConnectionType _typeConnection = DEFAULT_WEBSERVER_CONNECTION_TYPE
            ) const;
    //
    std::string convert_config_to_xwwwformcoded(const VLP16_settingsConfig& _config) const;

    bool get_ip(const ros::NodeHandle &_n, const std::string &_param_name="VLP16_NETWORK_SENSOR_IP");

    //
    bool parse_JSON_for_settings(const std::string &res_request, VLP16_SettingsServiceResponse &res);
    bool parse_JSON_for_status(const std::string &res_request, VLP16_StatusServiceResponse &res);
    bool parse_JSON_for_diagnostics_raw(const std::string &res_request, VLP16_DiagnosticsRawServiceResponse &res);

    /**
     * @brief scale_volt_temp
     *  inspired by 'js/diag.js' dump from the VLP-16 webserver
     * @param msg_raw
     * @param msg
     */
    bool scale_volt_temp(VLP16_DiagnosticsRawMessage &_msg_raw, VLP16_DiagnosticsMessage &_msg);

protected:
    //
    std::string request_webserver_curl(const WebServerCommands& _cmd) const;
    std::string request_webserver_asio_synch(const WebServerCommands& _cmd) const;
    std::string request_webserver_asio_asynch(const WebServerCommands& _cmd) const;
    //
    int send_settings_to_webserver_curl(const VLP16_settingsConfig& _config) const;
    int send_settings_to_webserver_asio_asynch(const VLP16_settingsConfig& _config) const;
    int send_settings_to_webserver_asio_synch(const VLP16_settingsConfig& _config) const;

private:
#ifdef USE_BOOST_FOR_FUNCTION_BINDING
    // ------------------------
    // Boost Version
    // ------------------------
    typedef boost::function<std::string(const WebServerCommands&)> fun_wsc_t;
    const std::map<WebServerConnectionType, fun_wsc_t> map_WSC_FuncRequest_ = boost::assign::map_list_of
            ( WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS, boost::bind(&VLP16_WebServer::request_webserver_asio_asynch,    this, _1))
            ( WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS,  boost::bind(&VLP16_WebServer::request_webserver_asio_synch,     this, _1))
            ( WebServerConnectionType::CURL,                    boost::bind(&VLP16_WebServer::request_webserver_curl,           this, _1))
            ;
    //
    typedef boost::function<int(const VLP16_settingsConfig&)> fun_st_t;
    const std::map<WebServerConnectionType, fun_st_t> map_ST_FuncRequest_ = boost::assign::map_list_of
            ( WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS, boost::bind(&VLP16_WebServer::send_settings_to_webserver_asio_asynch,   this, _1))
            ( WebServerConnectionType::CURL,                    boost::bind(&VLP16_WebServer::send_settings_to_webserver_curl,          this, _1))
            ;
#elif defined(USE_STD_FOR_FUNCTION_BINDING)
    // ------------------------
    // STD Version
    // ------------------------
    // urls:
    // - http://stackoverflow.com/questions/9281172/how-do-i-write-a-pointer-to-member-function-with-stdfunction
    // - http://stackoverflow.com/a/9281802
    // - http://stackoverflow.com/a/8688615
    // -
    typedef std::function<std::string(const WebServerCommands&)> fun_wsc_t;
    const std::map<WebServerConnectionType, fun_wsc_t> map_WSC_FuncRequest_ = {
        { WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS, std::bind(&VLP16_WebServer::request_webserver_asio_asynch,    this, std::placeholders::_1) },
        { WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS,  std::bind(&VLP16_WebServer::request_webserver_asio_synch,     this, std::placeholders::_1) },
        { WebServerConnectionType::CURL,                    std::bind(&VLP16_WebServer::request_webserver_curl,           this, std::placeholders::_1) }
    };
    //
    typedef std::function<int(const VLP16_settingsConfig&)> fun_st_t;
    const std::map<WebServerConnectionType, fun_st_t> map_ST_FuncRequest_ = {
        { WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS, std::bind(&VLP16_WebServer::send_settings_to_webserver_asio_asynch, this, std::placeholders::_1) },
        { WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS,  std::bind(&VLP16_WebServer::send_settings_to_webserver_asio_synch,  this, std::placeholders::_1) },
        { WebServerConnectionType::CURL,                    std::bind(&VLP16_WebServer::send_settings_to_webserver_curl, this, std::placeholders::_1) }
    };
#endif
};

}

#endif // VELODYNE_WEBSERVER_VLP16_H
