#ifndef VLP16_WEBSERVER_SERVICES_STATUS_H
#define VLP16_WEBSERVER_SERVICES_STATUS_H

#include <vlp16_webserver_services.h>
//
#include <velodyne_configuration/VLP16_StatusService.h>


namespace vlp16_webserver_services {
using namespace vlp16_webserver;


// Status
BOOST_PP_STRUCT_FOR_PACK_TYPES((Status), (Service, Message, ServiceResponse), velodyne_configuration::VLP16_)
//struct TRosStatusService    : public velodyne_configuration::VLP16_StatusService        { typedef velodyne_configuration::VLP16_StatusService value_type; };
//struct TRosStatusMessage    : public velodyne_configuration::VLP16_StatusMessage        { typedef velodyne_configuration::VLP16_StatusMessage value_type; };
//struct TRosStatusResponse   : public velodyne_configuration::VLP16_StatusServiceResponse{ typedef velodyne_configuration::VLP16_StatusServiceResponse value_type; };

typedef TTripletROS < TRosStatusService, TRosStatusMessage, TRosStatusServiceResponse > TTripletROS_Status;       // ok

typedef Velodyne_WebServer_Services< TTripletROS_Status > S_VWS_Status;


/**
 * @brief The Velodyne_WebServer_Status class
 */
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
    bool get_response(velodyne_configuration::VLP16_StatusServiceResponse& _res) override;

    bool parse_JSON(const std::string & _res_request, VLP16_StatusServiceResponse & _res) const override;
};

}
#endif // VLP16_WEBSERVER_SERVICES_STATUS_H
