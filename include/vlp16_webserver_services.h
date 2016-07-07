#ifndef VLP16_WEBSERVER_SERVICES_H
#define VLP16_WEBSERVER_SERVICES_H

#include <vlp16_webserver.h>

namespace vlp16_webserver_services {

using namespace vlp16_webserver;

//---------------------------------------------------------------------------

/**
 * @brief The Velodyne_WebServer_Services class
 */
template< class ROS_Service, class ROS_Message, class ROS_Response,
        class TWebServerConnectionType=VLP16_WebServer_BOOST_ASIO_ASYNCHRONOUS >
class Velodyne_WebServer_Services : public TWebServerConnectionType
{
public:
    /**
     * @brief Velodyne_WebServer_Services
     * @param _service_name
     * @param _nh
     * @param _loop_rate_value
     */
    Velodyne_WebServer_Services(
            const std::string&  _service_name,
            const std::string&  _nh="~",
            const double&       _loop_rate_value=DEFAULT_LOOP_RATE_VALUE
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
    
    /**
     * @brief get_topic_name_pub
     * @return
     */
    inline const std::string& get_topic_name_pub() const { return topic_name_pub_; }
    /**
     * @brief get_node_name_srv
     * @return
     */
    inline const std::string& get_node_name_srv() const { return node_name_srv_; }
    
    // ------------------
    // GETTER/SETTER
    // ------------------
    /**
     * @brief defaults_getter_setter
     */
    defaults_getter_setter(double, loop_rate_value_);
    
protected:    
    // ----------------------
    // PURE VIRTUAL METHODS
    // ----------------------
    virtual bool parse_JSON(const std::string & _res_request, ROS_Response & _res) const    = 0;
    virtual bool get_response(typename ROS_Service::Response& _res)                         = 0;
    // ----------------------
    // wrapper pour le bind ROS (2 arguments -> 1)
    inline bool get_response(typename ROS_Service::Request&, typename ROS_Service::Response& _res) { return get_response(_res); }

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
    //
    typename ROS_Service::Response laser_data_;
};


#define VLP16_ROS_CONSTRUCT_TUPLE(_ns, _name, _service, _message, _serviceresponse)    \
    _ns::VLP16_##_name##_service,        \
    _ns::VLP16_##_name##_message,        \
    _ns::VLP16_##_name##_serviceresponse \

#define VLP16_ROS_TRIPLET_SMSR(_ns, _name)    \
    VLP16_ROS_CONSTRUCT_TUPLE(_ns, _name, Service, Message, ServiceResponse)

//----------------------
// Typedef sur les spécialisations de la classe template Velodyne_WebServer_Services
// Spécialisation pour gérer les :
// - 'Settings'
// - 'Status'
// - 'Diagnostics'
typedef Velodyne_WebServer_Services< VLP16_ROS_TRIPLET_SMSR(velodyne_configuration, Status)   > S_VWS_Status;
typedef Velodyne_WebServer_Services< VLP16_ROS_TRIPLET_SMSR(velodyne_configuration, Settings) > S_VWS_Settings;
typedef Velodyne_WebServer_Services< VLP16_ROS_CONSTRUCT_TUPLE(velodyne_configuration, Diagnostics, Service, Message, RawServiceResponse) > S_VWS_Diagnostics;
}

#include "../src/webserver/services/vlp16_webserver_services.inl"

#endif // VLP16_WEBSERVER_SERVICES_H
