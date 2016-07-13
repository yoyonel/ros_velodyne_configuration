#include "velodynewebserver.h"
#include "vlp16webserver.h"
//
#include <string>   // for: std::string
#include <asynch_httpclient.h>  // for: ASynchHTTPClient
#include <synch_httpclient.h>   // for:  SynchHTTPClient
//
#include "vlp16webserver_ros.h"
//
#include <velodyne_configuration/VLP16_settingsConfig.h>    // for: VLP16_settingsConfig

#define ROS_WARNING_STREAM(...)


void test_velodyne_webserver(const std::string & _server)
{
    std::string response;

    boost::asio::io_service io_service;
    auto ptrHTTPClient = new ASynchHTTPClient(io_service);
//    auto ptrHTTPClient = new SynchHTTPClient(io_service);
    auto ptrVelodyneWebServer = new VLP16WebServer(ptrHTTPClient, _server);

    response = ptrVelodyneWebServer->get(std::string("status"));
    response = ptrVelodyneWebServer->get("settings");
    response = ptrVelodyneWebServer->get("diag");
    response = ptrVelodyneWebServer->get("toto");
    response = ptrVelodyneWebServer->get(VLP16WebServer::eVLP16WebServerRequests::settings);

    ptrVelodyneWebServer->post("rpm=333");    // ok with ASynch&Synch

    auto ptrVelodyneWebServerROS = new VLP16WebServerROS(ptrHTTPClient, _server);
    ptrVelodyneWebServerROS->post("rpm=0"); // ok with Synch&ASynch

    velodyne_configuration::VLP16_settingsConfig config;
    config.return_type = eReturnType::Strongest;
    config.laser_state = true;
    config.rpm = 322;
    ptrVelodyneWebServerROS->post(config);  // ok with Synch&ASynch
}
