#include <vlp16_webserver.h>
#include <boost/asio.hpp>

#include <memory>   // for: make_shared

#include <velodyne_tools_boost_asio_clientasynch.h> // for: ClientASynch


using namespace vlp16_webserver;
using namespace velodyne_tools::boost_asio; // for: ClientASynch
using namespace std;    // for: make_shared


// ----------------------------------
// ASIO ASYNCH
// ----------------------------------
std::string VLP16_WebServer_BOOST_ASIO_ASYNCHRONOUS::request(
        const WebServerCommands & _cmd
        ) const
{
    std::string response_json = "";
    try {
        boost::asio::io_service io_service;

        // c++11 style
        auto http_client = make_shared<ClientASynch>(io_service);
        http_client->get(network_sensor_ip_, std::string("/cgi/") + _cmd._to_string() + ".json");

        io_service.run();

        response_json = http_client->get_response();
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
    }
    return response_json;
}

int VLP16_WebServer_BOOST_ASIO_ASYNCHRONOUS::send(
        const VLP16_settingsConfig& _config
        ) const
{
    // url: http://stackoverflow.com/questions/36141746/boost-asio-http-client-post
    int retour = 1;
    try {
        boost::asio::io_service io_service;

        const std::string xwwwformcoded =  convert_config_to_xwwwformcoded(_config);

        // c++11 style
        auto http_client = make_shared<ClientASynch>(io_service);
        http_client->post(network_sensor_ip_, "/cgi/setting", xwwwformcoded);

        io_service.run();
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
        retour = 0;
    }
    return retour;
}
