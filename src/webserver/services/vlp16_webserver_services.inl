#include <vlp16_webserver_services.h>

namespace vlp16_webserver_services {

#define DECL_TEMPLATE_VELODYNEWEBSERVERSERVICES_CONSTRUCTOR()                                       \
    template<typename TripletTypes, class TWebServerConnectionType>                                 \
    Velodyne_WebServer_Services<TripletTypes, TWebServerConnectionType>::Velodyne_WebServer_Services

#define DECL_TEMPLATE_VELODYNEWEBSERVERSERVICES(_return_typ, _member_name)                          \
    template<typename TripletTypes, class TWebServerConnectionType>                                 \
    _return_typ Velodyne_WebServer_Services<TripletTypes, TWebServerConnectionType>::_member_name


DECL_TEMPLATE_VELODYNEWEBSERVERSERVICES_CONSTRUCTOR()(
        const std::string& _name,
        const std::string& _nh,
        const double& _loop_rate_value
        ) : nh_(_nh), loop_rate_value_(_loop_rate_value)
{
    // url: https://web.archive.org/web/20130423054841/http://www.agapow.net/programming/cpp/no-arguments-that-depend-on-a-template-parameter
    VLP16_WebServer::get_ip_from_ros_params(nh_);

    //----------------------------------------------------
    // Services
    //----------------------------------------------------
    node_name_srv_ = "get_" + _name;
    velodyne_service_srv_ = nh_.advertiseService(
                node_name_srv_,
                &Velodyne_WebServer_Services::get_response,
                this);
    //
    ROS_INFO("Ready to get velodyne %s.\t[SERVICE]", _name.c_str());
    //----------------------------------------------------

    //----------------------------------------------------
    // Publisher
    //----------------------------------------------------
    topic_name_pub_ = _name + "_pub";
    velodyne_service_pub_ = nh_.advertise<Velodyne_WebServer_Services::t_ROS_Message>(topic_name_pub_, 1000);
    //
    ROS_INFO("Publish %s messages.\t[PUBLISHER]", _name.c_str());
    //----------------------------------------------------
}

DECL_TEMPLATE_VELODYNEWEBSERVERSERVICES(void, run)(
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

/**
 * @brief DECL_TEMPLATE_VELODYNEWEBSERVERSERVICES
 */
DECL_TEMPLATE_VELODYNEWEBSERVERSERVICES(void, run_with_test_sub)(
        boost::function<void()> _postPublish
        )
{
    run( boost::function<bool()>( [this](){ return velodyne_service_pub_.getNumSubscribers() != 0; }, _postPublish ) );
}

}
