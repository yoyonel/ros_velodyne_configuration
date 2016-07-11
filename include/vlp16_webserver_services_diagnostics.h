#ifndef VLP16_WEBSERVER_SERVICES_DIAGNOSTICS_H
#define VLP16_WEBSERVER_SERVICES_DIAGNOSTICS_H

#include <vlp16_webserver_services.h>

#include <velodyne_configuration/VLP16_DiagnosticsService.h>
#include <velodyne_configuration/VLP16_DiagnosticsRawService.h>


namespace vlp16_webserver_services {
using namespace vlp16_webserver;

// Diagnostics
BOOST_PP_STRUCT_FOR_PACK_TYPES((Diagnostics), (Service, Message, RawServiceResponse), velodyne_configuration::VLP16_)
//
//struct TRosDiagnosticsService : public velodyne_configuration::VLP16_DiagnosticsService { typedef velodyne_configuration::VLP16_DiagnosticsService value_type; };
//struct TRosDiagnosticsMessage : public velodyne_configuration::VLP16_DiagnosticsMessage { typedef velodyne_configuration::VLP16_DiagnosticsMessage value_type; };
//struct TRosDiagnosticsServiceRawResponse : public velodyne_configuration::VLP16_DiagnosticsRawServiceResponse { typedef velodyne_configuration::VLP16_DiagnosticsRawServiceResponse value_type; };

typedef TTripletROS < TRosDiagnosticsService, TRosDiagnosticsMessage, TRosDiagnosticsRawServiceResponse > TTripletROS_Diagnostics;  // ok

typedef Velodyne_WebServer_Services< TTripletROS_Diagnostics > S_VWS_Diagnostics;


/**
 * @brief The Velodyne_WebServer_Diagnostics class
 */
class Velodyne_WebServer_Diagnostics : public S_VWS_Diagnostics
{
public:
    Velodyne_WebServer_Diagnostics(const std::string& _name="diagnostics");

    void run();

protected:
    bool get_response(velodyne_configuration::VLP16_DiagnosticsServiceResponse& _res) override;
    bool parse_JSON(const std::string & _res_request, VLP16_DiagnosticsRawServiceResponse & _res) const override;

private:
    bool get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse & _res) const;
    bool scale_volt_temp(VLP16_DiagnosticsRawMessage & _msg_raw, VLP16_DiagnosticsMessage & _msg) const;
};

}
#endif // VLP16_WEBSERVER_SERVICES_DIAGNOSTICS_H
