#ifndef VLP16_WEBSERVER_SERVICES_H
#define VLP16_WEBSERVER_SERVICES_H

#include <vlp16_webserver.h>

namespace vlp16_webserver_services {

using namespace vlp16_webserver;

//---------------------------------------------------------------------------
template<class Service, class Message>
class Velodyne_WebServer_Services : public VLP16_WebServer
{
public:
    Velodyne_WebServer_Services(
            const std::string& _service_name,
            const std::string& _nh="~",
            const double& _loop_rate_value=DEFAULT_LOOP_RATE_VALUE
            );

    // urls:
    // - http://stackoverflow.com/questions/26267115/boost-function-instantion-with-nothing
    // - http://www.radmangames.com/programming/how-to-use-boost-function
    // - http://fr.cppreference.com/w/cpp/language/lambda
    virtual void run(
            boost::function<bool()> _prePublish=[] () { return true; },
            boost::function<void()> _postPublish=[] () {}
    );

    virtual void run_with_test_sub(boost::function<void()> _postPublish=[] () {});

    inline const std::string& get_topic_name_pub() const { return topic_name_pub_; }
    inline const std::string& get_node_name_srv() const { return node_name_srv_; }

    // ------------------
    // GETTER/SETTER
    // ------------------
    defaults_getter_setter(double, loop_rate_value_);

protected:
    virtual bool get_response(typename Service::Response& _res) = 0;
    inline bool get_response(typename Service::Request&, typename Service::Response& _res)
    {
        return get_response(_res);
    }

protected:
    //
    ros::NodeHandle nh_;
    //
    ros::Publisher velodyne_service_pub_;
    std::string topic_name_pub_;
    double loop_rate_value_;
    //
    ros::ServiceServer velodyne_service_srv_;
    std::string node_name_srv_;

private:
    //! Private NodeHandle
    typename Service::Response laser_data_;
};

//----------------------
// Typedef sur les spécialisations de la classe template Velodyne_WebServer_Services
// Spécialisation pour gérer les :
// - 'Settings'
// - 'Status'
// - 'Diagnostics'
typedef Velodyne_WebServer_Services<velodyne_configuration::VLP16_DiagnosticsService, velodyne_configuration::VLP16_DiagnosticsMessage> S_VWS_Diagnostics;
typedef Velodyne_WebServer_Services<velodyne_configuration::VLP16_StatusService, velodyne_configuration::VLP16_StatusMessage> S_VWS_Status;
typedef Velodyne_WebServer_Services<velodyne_configuration::VLP16_SettingsService, velodyne_configuration::VLP16_SettingsMessage> S_VWS_Settings;

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class Velodyne_WebServer_Status : public S_VWS_Status
{
public:
    Velodyne_WebServer_Status(
            const std::string&  _name="status",
            const std::string&  _nh="~",
            const double&       _loop_rate_val=DEFAULT_LOOP_RATE_VALUE
            );

    void run();

protected:
    bool get_response(velodyne_configuration::VLP16_StatusServiceResponse& _res);
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class Velodyne_WebServer_Settings : public S_VWS_Settings
{
public:
    Velodyne_WebServer_Settings(const std::string& _name="settings");

protected:
    bool get_response(velodyne_configuration::VLP16_SettingsServiceResponse& _res);

private:
    ros::Subscriber velodyne_settings_sub_;
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class Velodyne_WebServer_Diagnostics : public S_VWS_Diagnostics
{
public:
    Velodyne_WebServer_Diagnostics(const std::string& _name="diagnostics");

    void run();

protected:
    bool get_response(velodyne_configuration::VLP16_DiagnosticsServiceResponse& _res);

private:
    bool get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse& _res);
};
//---------------------------------------------------------------------------

}

#endif // VLP16_WEBSERVER_SERVICES_H
