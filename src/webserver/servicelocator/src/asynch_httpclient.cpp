#include "asynch_httpclient.h"
#include "asynch_httpclient_imp.h"  // for: SynchHTTPClientImp

#define ROS_WARN_STREAM(...)

ASynchHTTPClient::ASynchHTTPClient(boost::asio::io_service& _io_service)
    : ptrClientImp_(new ASynchHTTPClientImp(_io_service))
{

}

const std::string & ASynchHTTPClient::get_response() const
{
    ptrClientImp_->get_response();
}

void ASynchHTTPClient::get(const std::string &_server,
                           const std::string &_path)
{
    try
    {
        ptrClientImp_->get(_server, _path);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
    }
}

void ASynchHTTPClient::post(const std::string &_server,
                            const std::string &_path,
                            const std::string &_xwwwformcoded)
{
    try
    {
        ptrClientImp_->post(_server, _path, _xwwwformcoded);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
    }
}
