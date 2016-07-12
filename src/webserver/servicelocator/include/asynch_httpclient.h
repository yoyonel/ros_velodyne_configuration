#ifndef ASYNCH_HTTPCLIENT_H
#define ASYNCH_HTTPCLIENT_H

#include "httpclient.h" // for: HTTPClient
#include <boost/asio.hpp>   // for: boost::asio::io_service

// Prototype de la classe d'implementation
class ASynchHTTPClientImp;

// ----------
// PSEUDO-IMP
// ----------
class ASynchHTTPClient : public HTTPClient
{
public:
    // ----------------
    // CONSTRUCTOR
    // ----------------
    ASynchHTTPClient(boost::asio::io_service& _io_service);

    // ----------------
    // OVERRIDES
    // ----------------
    void get(const std::string &_server, const std::string &_path) override;
    void post(const std::string &_server, const std::string &_path, const std::string &_xwwwformcoded) override;
    const std::string & get_response() const override;
    // ----------------

private:
    // ----------------
    // PTR TO IMPLEMENTATION
    // ----------------
    ASynchHTTPClientImp * ptrClientImp_;
    // ----------------
};


#endif // ASYNCH_HTTPCLIENT_H
