#include <ros/ros.h>
#include <velodyne_tools.h>
#include <string>

namespace velodyne_tools
{

//using namespace velodyne_configuration;

std::string exec_cmd(const char* cmd)
{
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

} // namespace velodyne_settings
