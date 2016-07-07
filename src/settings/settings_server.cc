#include <ros/ros.h>
#include <vlp16_settings.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_settings_server");

    VLP16_Settings vs;   // instanciation de l'object (à faire) APRES ros::init !

    vs.set_loop_rate_value_(10);    // 10Hz     [OK]
    vs.set_max_delay_for_cmd_(0.500f);  // => 500ms de delay max

    vs.run();

    return 0;
}

