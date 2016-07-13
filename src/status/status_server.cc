#include <ros/ros.h>
//
#include <vlp16_webserver_services_status.h>

#include "../webserver/servicelocator/include/test_httpclient.h"
//extern void test(const std::string & _ip);
//#include "../webserver/servicelocator/include/locator_httpclient.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_status_server");

    // ----------------------------
    // TODO: Service Locator test
    // ----------------------------
    servicelocator_test("172.20.0.191", "/cgi/status.json");
    servicelocator_test("172.20.0.191", "/cgi/settings.json");
    servicelocator_test("172.20.0.191", "/cgi/diag.json");
    // ---------------
    test_velodyne_webserver("172.20.0.191");
    // ----------------------------

    vlp16_webserver_services::Velodyne_WebServer_Status v_ws_s;
    v_ws_s.run();

    return 0;
}

