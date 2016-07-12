#ifndef ASYNCH_HTTPCLIENT_H
#define ASYNCH_HTTPCLIENT_H

#include "httpclient.h" // for: HTTPClient
#include "asynch_httpclient_imp.h"  // for: ASynchHTTPClientImp

// ----------
// PSEUDO-IMP
// ----------
class ASynchHTTPClient : public HTTPClient
{
public:
    // ----------------
    // CONSTRUCTOR
    // ----------------
    ASynchHTTPClient(boost::asio::io_service& _io_service)
        : ptrClientImp_(new ASynchHTTPClientImp(_io_service)) {}

    // ----------------
    // OVERRIDES
    // ----------------
    void get(const std::string &_server, const std::string &_path) override;
    void post(const std::string &_server, const std::string &_path, const std::string &_xwwwformcoded) override;
    const std::string & get_response() const override { ptrClientImp_->get_response(); }
    // ----------------

private:
    // ----------------
    // PTR TO IMPLEMENTATION
    // ----------------
    ASynchHTTPClientImp * ptrClientImp_;
    // ----------------
};


#endif // ASYNCH_HTTPCLIENT_H
