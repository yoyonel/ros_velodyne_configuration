#include <vlp16_webserver_services_settings.h>

using namespace vlp16_webserver_services;


Velodyne_WebServer_Settings::Velodyne_WebServer_Settings(const std::string& _name) : Velodyne_WebServer_Services(_name )
{
}

bool Velodyne_WebServer_Settings::get_response(velodyne_configuration::VLP16_SettingsServiceResponse &_res)
{
    const std::string res_request = request(velodyne_webserver::Velodyne_WebServer::WebServerCommands::settings);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
    return parse_JSON(res_request, _res);
}

bool Velodyne_WebServer_Settings::parse_JSON(const std::string & _res_request, VLP16_SettingsServiceResponse & _res) const
{
    // ------------------------------------
    // Manual JSON file parsing
    // ------------------------------------
    // urls:
    // - http://zenol.fr/blog/boost-property-tree/en.html
    // - https://gist.github.com/mloskot/1509935
    try
    {
        JSON_INIT(root, _res_request);

        JSON_READ_BOOL   (root, laser,                _res, laser_state);
        JSON_READ_STRING (root, returns,              _res, returns);
        JSON_READ_UINT16 (root, rpm,                  _res, rpm);
        JSON_READ_UINT16 (root, fov.start,            _res, fov_start);
        JSON_READ_UINT16 (root, fov.end,              _res, fov_end);
        JSON_READ_STATE  (root, phaselock.enabled,    _res, phaselock_enabled);
        JSON_READ_UINT16 (root, phaselock.offset,     _res, phaselock_offset);
        JSON_READ_STRING (root, host.addr,            _res, host_addr);
        JSON_READ_UINT16 (root, host.dport,           _res, host_dport);
        JSON_READ_UINT16 (root, host.tport,           _res, host_tport);
        JSON_READ_STRING (root, net.addr,             _res, net_addr);
        JSON_READ_STRING (root, net.mask,             _res, net_mask);
        JSON_READ_STRING (root, net.gateway,          _res, net_gateway);
        JSON_READ_BOOL   (root, net.dhcp,             _res, net_dhcp);

    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    // ------------------------------------

    return true;

}
