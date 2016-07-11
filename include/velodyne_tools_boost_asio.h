#ifndef VELODYNE_TOOLS_BOOST_ASIO_H
#define VELODYNE_TOOLS_BOOST_ASIO_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>

//#include <ros/ros.h>

namespace velodyne_tools {
namespace boost_asio {

using boost::asio::ip::tcp;

const size_t kMaxSizeResponse = 172;

class IClient
{
public:
    IClient(boost::asio::io_service& _io_service,
           const size_t _maxSizeResponse = kMaxSizeResponse)
        : resolver_(_io_service), socket_(_io_service), request_stream_(&request_)
    {
        str_response_.reserve(_maxSizeResponse);
        str_response_.clear();
    }

    virtual void get(const std::string& server,
                     const std::string& path) = 0;
    virtual void post(const std::string& server,
                     const std::string& path,
                     const std::string& xwwwformcoded) = 0;

    inline const std::string& get_response() const { return str_response_; }

protected:

protected:
    std::string str_response_;

    tcp::resolver resolver_;
    tcp::socket socket_;
    boost::asio::streambuf request_;
    std::ostream request_stream_;
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

} // namespace boost_asio
} // namespace velodyne_tools

#endif // VELODYNE_TOOLS_BOOST_ASIO_H
