#include <ros/ros.h>
//
#include <vlp16_webserver_services.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_status_server");

    vlp16_webserver_services::Velodyne_WebServer_Status v_ws_s;
    v_ws_s.run();

    return 0;
}

