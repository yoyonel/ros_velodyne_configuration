#ifndef BOOST_HTTPCLIENT_H
#define BOOST_HTTPCLIENT_H

#include <boost/asio.hpp>   // for: tcp::resolver, tcp::socket, boost::asio::streambuf, std::ostream

using boost::asio::ip::tcp;

// ----------------
// SERVICE Providers
// ----------------

// ----------
// INTERFACE + SERVICES
// ----------
class BoostHTTPClient
{
public:
    BoostHTTPClient(boost::asio::io_service& _io_service)
        : resolver_(_io_service), socket_(_io_service), request_stream_(&request_), io_service_(_io_service)
    { }   

    // ----------------
    // PURES VIRTUALS
    // ----------------
    virtual int get(const std::string& _server, const std::string& _path) = 0;
    virtual int post(const std::string& _server, const std::string& _path, const std::string& _xwwwformcoded) = 0;
    // ----------------

    // ----------------
    // SERVICES
    // ----------------
    inline const std::string & get_response() const { return str_response_; }
    inline void run_io_service() const { io_service_.run(); }
    // ----------------

protected:
    tcp::resolver resolver_;
    tcp::socket socket_;
    boost::asio::streambuf request_;
    std::ostream request_stream_;
    std::string str_response_;
    boost::asio::io_service & io_service_;
};

// Form the request. We specify the "Connection: close" header so that the
// server will close the socket after transmitting the response. This will
// allow us to treat all data up until the EOF as the content.
#define BUILD_REQUEST_GET(request_stream, server, path)     \
    request_stream << "GET " << path << " HTTP/1.0\r\n";   \
    request_stream << "Host: " << server << "\r\n";        \
    request_stream << "Accept: */*\r\n";                   \
    request_stream << "Connection: close\r\n\r\n";

#define BUILD_REQUEST_POST(request_stream, server, path, xwwwformcoded)         \
    request_stream << "POST " << path << " HTTP/1.0\r\n";                       \
    request_stream << "Host: " << server << "\r\n";                             \
    request_stream << "User-Agent: C/1.0";                                      \
    request_stream << "Accept: */*\r\n";                                        \
    request_stream << "Referer: rbose\r\n";                                     \
    request_stream << "Content-Length: " <<  xwwwformcoded.length() << "\r\n";  \
    request_stream << "Content-Type: application/x-www-form-urlencoded\r\n";    \
    request_stream << "Connection: close\r\n\r\n";                              \
    request_stream << xwwwformcoded;

#endif // BOOST_HTTPCLIENT_H
