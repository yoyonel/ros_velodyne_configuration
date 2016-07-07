#ifndef VLP16_WEBSERVER_SERVICES_SETTINGS_H
#define VLP16_WEBSERVER_SERVICES_SETTINGS_H

#include <vlp16_webserver_services.h>


namespace vlp16_webserver_services {
using namespace vlp16_webserver;

/**
 * @brief The Velodyne_WebServer_Settings class
 */
class Velodyne_WebServer_Settings : public S_VWS_Settings
{
public:
    Velodyne_WebServer_Settings(const std::string& _name="settings");

protected:
    bool get_response(velodyne_configuration::VLP16_SettingsServiceResponse& _res) override;
    bool parse_JSON(const std::string & _res_request, VLP16_SettingsServiceResponse & _res) const override;

private:
    ros::Subscriber velodyne_settings_sub_;
};

}
#endif // VLP16_WEBSERVER_SERVICES_SETTINGS_H
