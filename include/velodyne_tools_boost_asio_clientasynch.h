#ifndef VELODYNE_TOOLS_BOOST_ASIO_CLIENTASYNCH_H
#define VELODYNE_TOOLS_BOOST_ASIO_CLIENTASYNCH_H


#include <boost/asio.hpp>
#include <velodyne_tools_boost_asio.h>  // for: IClient


namespace velodyne_tools {
namespace boost_asio {

// url: http://www.boost.org/doc/libs/1_49_0/doc/html/boost_asio/example/http/client/async_client.cpp
class ClientASynch : public IClient
{
public:
    ClientASynch(boost::asio::io_service & _io_service)
        : IClient(_io_service)
    {}

    virtual ~ClientASynch() {}

    void get(const std::string& server,
             const std::string& path) override;

    void post(const std::string& server,
             const std::string& path,
             const std::string& xwwwformcoded) override;

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
    boost::asio::streambuf response_;
};

typedef boost::shared_ptr<ClientASynch> ClientASynchPtr;

}
}

#endif // VELODYNE_TOOLS_BOOST_ASIO_CLIENTASYNCH_H
