#include "boost_httpclient.h"

// ----------------
// CONSTRUCTOR
// ----------------
BoostHTTPClient::BoostHTTPClient(boost::asio::io_service& _io_service)
    : resolver_(_io_service),
      socket_(_io_service),
      request_stream_(&request_),
      io_service_(_io_service)
{
}

// ----------------
// SERVICES
// ----------------
const std::string & BoostHTTPClient::get_response() const
{
    return str_response_;
}

void BoostHTTPClient::run_io_service() const
{
    // url: http://www.boost.org/doc/libs/1_61_0/doc/html/boost_asio/reference/io_service/reset.html
    // TODO: revoir cette notion de io_service et taches asynchrones !
    io_service_.reset();
    io_service_.run();
}
