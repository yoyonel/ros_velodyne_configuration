#include <vlp16_webserver.h>
#include <boost/asio.hpp>


using namespace vlp16_webserver;

// ----------------------------------
// ASIO SYNCH
// ----------------------------------
std::string VLP16_WebServer_BOOST_ASIO_SYNCHRONOUS::request(
        const WebServerCommands & _cmd
        ) const
{
    std::string response_json = "";
    try {
        boost::asio::io_service io_service;
        velodyne_tools::boost_asio::client_synch c(
                    io_service,
                    network_sensor_ip_,
                    std::string("/cgi/") + _cmd._to_string() + ".json"
                    );
        response_json = c.get_response();
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
    }
    return response_json;
}

int VLP16_WebServer_BOOST_ASIO_SYNCHRONOUS::send(
        const VLP16_settingsConfig& _config
        ) const
{
    // url: http://stackoverflow.com/questions/36141746/boost-asio-http-client-post
    int retour = 1;
    try {
        const std::string xwwwformcoded =  convert_config_to_xwwwformcoded(_config);
        boost::asio::io_service io_service;
        velodyne_tools::boost_asio::client_synch c(
                    io_service,
                    network_sensor_ip_,
                    "/cgi/setting",
                    xwwwformcoded
                    );
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
        retour = 0;
    }
    return retour;
}

