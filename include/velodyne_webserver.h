#ifndef VELODYNE_WEBSERVER_H
#define VELODYNE_WEBSERVER_H

#include <velodyne_tools.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
//
#include "enum.h"

#include <ros/ros.h>
#include <ros/service.h>


namespace pt = boost::property_tree;

namespace velodyne_webserver {

//using namespace velodyne_configuration;

//// -------------------------------------------
//// url: http://stackoverflow.com/a/13188585
//// -------------------------------------------
//#define stringify( name ) # name

/**
 * @brief BETTER_ENUM
 */
BETTER_ENUM( _WebServerCommands, char,
             settings=1,
             status,
             info,
             diag
             );

/**
 * @brief BETTER_ENUM
 */
BETTER_ENUM( _LaserReturns, char,
             Strongest=0,
             Last,
             Dual
             );

/**
 * @brief The _WebServerConnectionType enum
 */
enum _WebServerConnectionType {
    BOOST_ASIO_SYNCHRONOUS=0,
    BOOST_ASIO_ASYNCHRONOUS,
    CURL
};

/**
 * @brief The Velodyne_WebServer class
 */
class Velodyne_WebServer
{
public:
    // url: http://aantron.github.io/better-enums/
    // usage: https://raw.githubusercontent.com/aantron/better-enums/master/doc/image/sample.gif
    typedef _WebServerCommands WebServerCommands;
    typedef _LaserReturns LaserReturns;
    typedef _WebServerConnectionType WebServerConnectionType;

public:
    // -------------------------------------------
    // GETTER/SETTER
    // -------------------------------------------
    defaults_getter_setter(float, max_delay_for_cmd_)
    defaults_getter_setter(std::string, network_sensor_ip_);

protected:
    virtual std::string request(const WebServerCommands &_cmd) const = 0;

protected:
    std::string network_sensor_ip_;
    float max_delay_for_cmd_;
};

}

#endif // VELODYNE_WEBSERVER_H
