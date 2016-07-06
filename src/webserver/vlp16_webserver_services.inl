#include <vlp16_webserver_services.h>

namespace vlp16_webserver_services {

template<class Service, class Message>
Velodyne_WebServer_Services<Service, Message>::Velodyne_WebServer_Services(
        const std::string& _name,
        const std::string& _nh,
        const double& _loop_rate_value
        ) : nh_(_nh), loop_rate_value_(_loop_rate_value)
{
    get_ip(nh_);

    //----------------------------------------------------
    // Services
    //----------------------------------------------------
    node_name_srv_ = "get_" + _name;
    velodyne_service_srv_ = nh_.advertiseService(node_name_srv_, &Velodyne_WebServer_Services::get_response, this);
    //
    ROS_INFO("Ready to get velodyne %s.\t[SERVICE]", _name.c_str());
    //----------------------------------------------------

    //----------------------------------------------------
    // Publisher
    //----------------------------------------------------
    topic_name_pub_ = _name + "_pub";
    velodyne_service_pub_ = nh_.advertise<Message>(topic_name_pub_, 1000);
    //
    ROS_INFO("Publish %s messages.\t[PUBLISHER]", _name.c_str());
    //----------------------------------------------------
}

template<class Service, class Message>
void Velodyne_WebServer_Services<Service, Message>::run(
        boost::function<bool()> _prePublish,
        boost::function<void()> _postPublish
        )
{
    //----------------------------------------
    // Boucle d'exécution ROS
    //----------------------------------------
    ros::Rate loop_rate(loop_rate_value_);   // Hz
    while (ros::ok())
    {
        if( _prePublish() ) {
            if( get_response(laser_data_) ) {
                velodyne_service_pub_.publish(laser_data_.msg);
            }
            _postPublish();
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    //----------------------------------------
}

template<class Service, class Message>
void Velodyne_WebServer_Services<Service, Message>::run_with_test_sub(
        boost::function<void()> _postPublish
        )
{
    run( boost::function<bool()>( [this](){ return velodyne_service_pub_.getNumSubscribers() != 0; }, _postPublish ) );
}


//----------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------
Velodyne_WebServer_Status::Velodyne_WebServer_Status(
        const std::string& _name,
        const std::string& _nh,
        const double& _loop_rate_value
        ) :
    Velodyne_WebServer_Services(_name, _nh, _loop_rate_value)
{
}

bool Velodyne_WebServer_Status::get_response(velodyne_configuration::VLP16_StatusServiceResponse &_res)
{
    const std::string res_request = request_webserver(velodyne_webserver::Velodyne_WebServer::WebServerCommands::status);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );

    return parse_JSON_for_status(res_request, _res);
}

void Velodyne_WebServer_Status::run()
{
    // url: http://fr.cppreference.com/w/cpp/language/lambda
    Velodyne_WebServer_Services<velodyne_configuration::VLP16_StatusService, velodyne_configuration::VLP16_StatusMessage>::run(
                boost::function<bool()>( [this](){ return velodyne_service_pub_.getNumSubscribers() != 0; } )
                );
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
Velodyne_WebServer_Settings::Velodyne_WebServer_Settings(const std::string& _name) : Velodyne_WebServer_Services(_name )
{
}

bool Velodyne_WebServer_Settings::get_response(velodyne_configuration::VLP16_SettingsServiceResponse &_res)
{
    const std::string res_request = request_webserver(velodyne_webserver::Velodyne_WebServer::WebServerCommands::settings);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );

    return parse_JSON_for_settings(res_request, _res);
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
Velodyne_WebServer_Diagnostics::Velodyne_WebServer_Diagnostics(const std::string& _name) : Velodyne_WebServer_Services(_name )
{
}

void Velodyne_WebServer_Diagnostics::run()
{
    Velodyne_WebServer_Services<VLP16_DiagnosticsService, VLP16_DiagnosticsMessage>::run_with_test_sub();
}

bool Velodyne_WebServer_Diagnostics::get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse& _res)
{
    const std::string res_request = request_webserver(velodyne_webserver::Velodyne_WebServer::WebServerCommands::diag);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
    return parse_JSON_for_diagnostics_raw(res_request, _res);
}

bool Velodyne_WebServer_Diagnostics::get_response(velodyne_configuration::VLP16_DiagnosticsServiceResponse& _res)
{
    velodyne_configuration::VLP16_DiagnosticsRawServiceResponse res_raw;
    get_diagnostics_raw(res_raw);
    return scale_volt_temp(res_raw.msg, _res.msg);
}
//---------------------------------------------------------------------------
}
