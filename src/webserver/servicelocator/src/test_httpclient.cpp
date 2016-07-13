#include "test_httpclient.h"
#include "httpclient.h" // for: HTTPClient
#include "locator_httpclient.h" // for: LocatorHTTPClient
#include "asynch_httpclient.h"  // for: ASynchHTTPClient
#include  "synch_httpclient.h"  // for:  SynchHTTPClient

#include <ros/ros.h>

HTTPClient* LocatorHTTPClient::service_;
NullHTTPClient LocatorHTTPClient::nullService_;


void set_servicelocator(HTTPClient* service_provider_);
void use_servicelocator(const std::string & _server, const std::string & _path);

// ----------------
// Test
// ----------------
// TODO: Mettre en place les timeouts sur les requetes !!!

void servicelocator_test(const std::string & _server, const std::string & _path)
{
    // initialize locator
    LocatorHTTPClient::initialize();

    // On instancie des providers de services
    boost::asio::io_service io_service;
    ASynchHTTPClient *ptrASynchClient = new ASynchHTTPClient(io_service);
    SynchHTTPClient  * ptrSynchClient = new  SynchHTTPClient(io_service);
    HTTPClient * ptrClient = ptrASynchClient; // ok
//    HTTPClient * ptrClient = ptrSynchClient;    // ok

    // On assigne un service provider au locator
    set_servicelocator(ptrClient);

    // Utilisation du Locator pour fournir un provider
    // qui nous permettra de faire une requete HTTP
    use_servicelocator(_server, "/cgi/setting");
}

void use_servicelocator(const std::string & _server, const std::string & _path)
{
    // On demande au locator de nous fournir
    // un provider de service
    HTTPClient *client = LocatorHTTPClient::getHTTPClient();
    client->get(_server, _path);
    ROS_INFO_STREAM("client->get_response(): " << client->get_response());

    // test post
    client->post(_server, _path, "rpm=444");    // ok: pour Synch & ASynch
}

void set_servicelocator(HTTPClient* service_provider_)
{
    // On assigne un service provider au locator
    LocatorHTTPClient::provide(service_provider_);
}
