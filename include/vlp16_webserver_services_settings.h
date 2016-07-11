#ifndef VLP16_WEBSERVER_SERVICES_SETTINGS_H
#define VLP16_WEBSERVER_SERVICES_SETTINGS_H

#include <vlp16_webserver_services.h>
//
#include <velodyne_configuration/VLP16_SettingsService.h>

namespace vlp16_webserver_services {
using namespace vlp16_webserver;

// Settings
BOOST_PP_STRUCT_FOR_PACK_TYPES((Settings), (Service, Message, ServiceResponse), velodyne_configuration::VLP16_)
//struct TRosSettingsService    : public velodyne_configuration::VLP16_SettingsService        { typedef velodyne_configuration::VLP16_SettingsService value_type; };
//struct TRosSettingsMessage    : public velodyne_configuration::VLP16_SettingsMessage        { typedef velodyne_configuration::VLP16_SettingsMessage value_type; };
//struct TRosSettingsResponse   : public velodyne_configuration::VLP16_SettingsServiceResponse{ typedef velodyne_configuration::VLP16_SettingsServiceResponse value_type; };

typedef TTripletROS < TRosSettingsService, TRosSettingsMessage, TRosSettingsServiceResponse > TTripletROS_Settings;     // ok

typedef Velodyne_WebServer_Services< TTripletROS_Settings > S_VWS_Settings;


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
