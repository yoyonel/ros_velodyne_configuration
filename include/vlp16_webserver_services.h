#ifndef VLP16_WEBSERVER_SERVICES_H
#define VLP16_WEBSERVER_SERVICES_H

#include <vlp16_webserver.h>

namespace vlp16_webserver_services {

using namespace vlp16_webserver;

//---------------------------------------------------------------------------

template <class T>
struct value_type
{
    typedef typename T::value_type type;
};

template< class CRosService, class CRosMessage, class CRosResponse >
struct TTripletROS {
    typedef typename value_type<CRosService>::type value_type_rosservice;
    typedef typename value_type<CRosMessage>::type value_type_rosmessage;
    typedef typename value_type<CRosResponse>::type value_type_rosresponse;
};


/**
 *
 */
template<typename tn_ROSTriplet, class TWebServerConnectionType=VLP16_WebServer_BOOST_ASIO_SYNCHRONOUS >
//template<typename tn_ROSTriplet, class TWebServerConnectionType=VLP16_WebServer_BOOST_ASIO_ASYNCHRONOUS >
class Velodyne_WebServer_Services : public TWebServerConnectionType
{
    // On recupere les types packes dans le typename TripletTypes
    typedef typename tn_ROSTriplet::value_type_rosservice  t_ROS_Service;
    typedef typename tn_ROSTriplet::value_type_rosmessage  t_ROS_Message;
    typedef typename tn_ROSTriplet::value_type_rosresponse t_ROS_Response;

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
    virtual bool parse_JSON(const std::string & _res_request, t_ROS_Response & _res) const    = 0;
    virtual bool get_response(typename t_ROS_Service::Response& _res)                         = 0;
    // ----------------------
    // wrapper pour le bind ROS (2 arguments -> 1)
    inline bool get_response(typename t_ROS_Service::Request&, typename t_ROS_Service::Response& _res) { return get_response(_res); }

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
    typename t_ROS_Service::Response laser_data_;
};
}


#include "../src/webserver/services/vlp16_webserver_services.inl"


// TODO: Utilisation Meta-Prog avec BOOST
// urls:
// - http://www.boost.org/doc/libs/1_47_0/libs/preprocessor/doc/ref/list_for_each_product.html
// - http://www.boost.org/doc/libs/1_38_0/libs/preprocessor/doc/ref/seq_for_each.html
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/for_each_i.hpp>
#include <boost/preprocessor/list/for_each_product.hpp>
#include <boost/preprocessor/seq/enum.hpp>

#define PPCAT_NX(A, B) A ## B
#define PPCAT(A, B) PPCAT_NX(A, B)

#define BOOST_PP_SFPT_BUILD_STRUCT_DECL(_ros_nameclass, _prefix)         \
    struct PPCAT(TRos, _ros_nameclass)                  \
    : public PPCAT(_prefix, _ros_nameclass) {           \
    typedef PPCAT(_prefix, _ros_nameclass) value_type;  \
    };

#define BOOST_PP_TUPLE_ELEM2(index, tuple) BOOST_PP_TUPLE_ELEM(2, index, tuple)

#define BOOST_PP_STRINGIFY_TUPLE2(tuple) \
    PPCAT(BOOST_PP_TUPLE_ELEM2(0, tuple), BOOST_PP_TUPLE_ELEM2(1, tuple))

#define BOOST_PP_SFPT_BUILD_STRUCT(tuple_ros_nameclass, _prefix) \
    BOOST_PP_SFPT_BUILD_STRUCT_DECL(BOOST_PP_STRINGIFY_TUPLE2(tuple_ros_nameclass), _prefix)

#define BOOST_PP_SFPT_MACRO(r, product)                                                                   \
    BOOST_PP_SFPT_BUILD_STRUCT((BOOST_PP_TUPLE_ELEM(2, 0, product), BOOST_PP_TUPLE_ELEM(2, 1, product)),  \
    velodyne_configuration::VLP16_                                                          \
    )

#define BOOST_PP_STRUCT_FOR_PACK_TYPES(L1, L2, _prefix) \
    BOOST_PP_LIST_FOR_EACH_PRODUCT(BOOST_PP_SFPT_MACRO, 2, (BOOST_PP_TUPLE_TO_LIST(L1), BOOST_PP_TUPLE_TO_LIST(L2)))


#endif // VLP16_WEBSERVER_SERVICES_H
