#ifndef VELODYNE_TOOLS_BOOST_ASIO_H
#define VELODYNE_TOOLS_BOOST_ASIO_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

namespace velodyne_tools {
namespace boost_asio {

using boost::asio::ip::tcp;

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

class client_synch
{
public:
    /**
     * @brief client_synch
     * @param io_service
     * @param server
     * @param path
     */
    client_synch(
            boost::asio::io_service& io_service,
            const std::string& server,
            const std::string& path
            );

    /**
     * @brief client_synch
     * @param io_service
     * @param server
     * @param path
     * @param xwwwformcoded
     */
    client_synch(
            boost::asio::io_service& io_service,
            const std::string& server,
            const std::string& path,
            const std::string& xwwwformcoded
            );

    /**
     * @brief get_response
     * @return
     */
    inline const std::string& get_response() const { return str_response_; }

protected:
    /**
     * @brief handle_request
     * @param server
     * @param path
     * @return
     */
    int handle_request_for_GET(const std::string& server, const std::string& path);

    int handle_request_for_POST(const std::string& server, const std::string& path, const std::string& xwwwformcoded);

    int perform_request(const std::string& _server);

private:
    std::string str_response_;
    tcp::resolver resolver_;
    tcp::socket socket_;
    boost::asio::streambuf request_;
    std::ostream request_stream_;
};

// url: http://www.boost.org/doc/libs/1_49_0/doc/html/boost_asio/example/http/client/async_client.cpp
class client_asynch
{
public:
    /**
     * @brief client_asynch
     * @param io_service
     * @param server
     * @param path
     * @param http_resquet
     */
    client_asynch(
            boost::asio::io_service& io_service,
            const std::string& server,
            const std::string& path
            );

    /**
     * @brief client_asynch
     * @param io_service
     * @param server
     * @param path
     * @param xwwwformcoded
     *
     * url: http://stackoverflow.com/questions/36141746/boost-asio-http-client-post
     */
    client_asynch(
            boost::asio::io_service& io_service,
            const std::string& server,
            const std::string& path,
            const std::string& xwwwformcoded
            );

    /**
     * @brief get_response
     * @return
     */
    const std::string& get_response() const { return str_response_; }

protected:
    /**
     * @brief handle_resolve
     * @param err
     * @param endpoint_iterator
     */
    void handle_resolve(const boost::system::error_code& err, tcp::resolver::iterator endpoint_iterator);

    /**
     * @brief handle_connect
     * @param err
     */
    void handle_connect(const boost::system::error_code& err);

    /**
     * @brief handle_write_request
     * @param err
     */
    void handle_write_request(const boost::system::error_code& err);

    /**
     * @brief handle_read_status_line
     * @param err
     */
    void handle_read_status_line(const boost::system::error_code& err);

    /**
     * @brief handle_read_headers
     * @param err
     */
    void handle_read_headers(const boost::system::error_code& err);

    /**
     * @brief handle_read_content
     * @param err
     */
    void handle_read_content(const boost::system::error_code& err);

private:
    tcp::resolver resolver_;
    tcp::socket socket_;
    boost::asio::streambuf request_;
    boost::asio::streambuf response_;
    std::string str_response_;
};

} // namespace boost_asio
} // namespace velodyne_tools

#endif // VELODYNE_TOOLS_BOOST_ASIO_H
