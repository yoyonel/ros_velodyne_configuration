#ifndef ASYNCH_HTTPCLIENT_IMP_H
#define ASYNCH_HTTPCLIENT_IMP_H

#include "boost_httpclient.h"

// ----------------
// SERVICE Providers
// ----------------

// ----------
// IMPlementation
// ----------
class ASynchHTTPClientImp : public BoostHTTPClient
{
public:
    // ----------------
    // CONSTRUCTOR
    // ----------------
    ASynchHTTPClientImp(boost::asio::io_service& _io_service) : BoostHTTPClient(_io_service) {}

    // ----------------
    // OVERRIDES
    // ----------------
    int handle_request_for_GET(const std::string& _server, const std::string& _path) override;
    int handle_request_for_POST(const std::string& _server, const std::string& _path, const std::string& _xwwwformcoded) override;
    // ----------------

private:
    int  perform_request(const std::string& _server);
    void handle_resolve(const boost::system::error_code& err, tcp::resolver::iterator endpoint_iterator);
    void handle_connect(const boost::system::error_code& err);
    void handle_write_request(const boost::system::error_code& err);
    void handle_read_status_line(const boost::system::error_code& err);
    void handle_read_headers(const boost::system::error_code& err);
    void handle_read_content(const boost::system::error_code& err);

private:
    boost::asio::streambuf response_;
};


#endif // ASYNCH_HTTPCLIENT_IMP_H
