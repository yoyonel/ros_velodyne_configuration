#include <vlp16_webserver_services_status.h>


using namespace vlp16_webserver_services;


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
    const std::string res_request = request(velodyne_webserver::Velodyne_WebServer::WebServerCommands::status);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
    return parse_JSON(res_request, _res);
}

void Velodyne_WebServer_Status::run()
{
    // url: http://fr.cppreference.com/w/cpp/language/lambda
    Velodyne_WebServer_Services<
//            velodyne_configuration::VLP16_StatusService,
//            velodyne_configuration::VLP16_StatusMessage,
//            velodyne_configuration::VLP16_StatusServiceResponse
            TTripletROS_Status
            >::run(
                boost::function<bool()>( [this](){ return velodyne_service_pub_.getNumSubscribers() != 0; } )
                );
}

bool Velodyne_WebServer_Status::parse_JSON(const std::string & _res_request, VLP16_StatusServiceResponse & _res) const
{
    // ------------------------------------
    // Manual JSON file parsing
    // ------------------------------------
    // urls:
    // - http://zenol.fr/blog/boost-property-tree/en.html
    // - https://gist.github.com/mloskot/1509935
    try
    {
        JSON_INIT(root, _res_request );

        /**
             JSON: { ... "gps":{"pps_state":"Locked","position":"49 00.00N 200 .00W"} ... }
             - gps
             -- pps_state
             -- position
            /**/
        JSON_READ_STRING(root, gps.pps_state,  _res, gps_state);
        JSON_READ_STRING(root, gps.position,   _res, gps_position);
        /**
             JSON: { ... "motor":{"state":"On","rpm":0,"lock":"Off","phase":30755} ... }
             - motor
             -- state
             -- rpm
             -- lock
             -- phase
            /**/
        JSON_READ_BOOL(root, motor.state, _res, motor_state);
        JSON_READ_UINT16(root, motor.rpm, _res, motor_rpm);
        JSON_READ_BOOL(root, motor.lock,  _res, motor_lock);
        JSON_READ_UINT16(root, motor.phase, _res, motor_phase);
        /**
             JSON: { ... "laser":{"state":"Disabled"} ... }
             - laser
             -- state
            /**/
        JSON_READ_BOOL(root, laser.state, _res, laser_state);
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    // ------------------------------------
    return true;
}
