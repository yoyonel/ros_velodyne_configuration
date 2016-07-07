#include <ros/ros.h>
//
#include <vlp16_webserver_services_diagnostics.h>
#include <ros_macros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_diagnostics_server");

    vlp16_webserver_services::Velodyne_WebServer_Diagnostics v_ws_d;
    v_ws_d.run();

    return 0;
}

