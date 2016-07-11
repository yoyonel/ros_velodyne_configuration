#ifndef VELODYNE_TOOLS_BOOST_ASIO_CLIENTSYNCH_H
#define VELODYNE_TOOLS_BOOST_ASIO_CLIENTSYNCH_H

#include <boost/asio.hpp>
#include <velodyne_tools_boost_asio.h>  // for: IClient

namespace velodyne_tools {
namespace boost_asio {

class ClientSynch : public IClient
{
public:
    ClientSynch(boost::asio::io_service& io_service)
        : IClient(io_service)
    {}

    virtual ~ClientSynch() {}

    void get(const std::string& server,
             const std::string& path) override;
    void post(const std::string& server,
             const std::string& path,
             const std::string& xwwwformcoded) override;

private:
    int perform_request(const std::string& _server);

    int handle_request_for_GET(const std::string& server, const std::string& path);
    int handle_request_for_POST(const std::string& server, const std::string& path, const std::string& xwwwformcoded);
};

typedef boost::shared_ptr<ClientSynch> ClientSynchPtr;

}
}

#endif // VELODYNE_TOOLS_BOOST_ASIO_CLIENTSYNCH_H
